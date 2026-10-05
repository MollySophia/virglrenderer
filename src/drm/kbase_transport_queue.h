/* SPDX-License-Identifier: MIT */
static void queue_update(struct kb_queue *q)
{
   volatile uint64_t *out = (void *)((uint8_t *)q->io + 8192 + q->slot);
   uint64_t *shadow = (void *)((uint8_t *)q->shm + 8192 + q->slot);
   __atomic_store_n(shadow, *out, __ATOMIC_RELEASE);
   /* Always route the next kick through the host instead of a guest doorbell. */
   shadow[1] = 0;
}
static void queue_free(struct kb_ctx *c, struct kb_queue *q)
{
   munmap(q->io, 12288);
   struct kbase_ioctl_cs_queue_terminate term = {.buffer_gpu_addr = q->ring_va};
   ioctl(c->base.fd, KBASE_IOCTL_CS_QUEUE_TERMINATE, &term);
   munmap(q->shm, 12288); close(q->fd);
   memset(q, 0, sizeof(*q));
}
static int queue_command(struct kb_ctx *c, const struct kb_req *req, struct kb_rsp *rsp)
{
   uint64_t va; memcpy(&va, req->data, 8);
   if (req->handle == 3) {
      if (req->size > 20) return -EINVAL;
      struct pollfd p = {.fd = c->base.fd, .events = POLLIN};
      int ret = poll(&p, 1, req->size);
      if (ret < 0) return -errno;
      if (ret > 0 && (p.revents & POLLIN)) {
         uint8_t event[64];
         ssize_t n = read(c->base.fd, event, sizeof(event));
         if (n == 64 && event[0] == BASE_CSF_NOTIFICATION_GPU_QUEUE_GROUP_ERROR) return -EIO;
      }
      for (unsigned i = 0; i < 32; i++) if (c->queue[i].io) queue_update(&c->queue[i]);
      return 0;
   }
   struct kb_queue *q = NULL;
   for (unsigned i = 0; i < 32; i++)
      if (c->queue[i].io && c->queue[i].ring_va == va) { q = &c->queue[i]; break; }
   if (req->handle == 1) {
      if (!q) return -ENOENT;
      queue_free(c, q); return 0;
   }
   if (req->handle == 2) {
      if (!q) return -ENOENT;
      struct kb_bo *bo = NULL;
      for (unsigned i = 0; i < MAX_BO; i++)
         if (c->bo[i].ptr && c->bo[i].handle == q->bo) { bo = &c->bo[i]; break; }
      if (!bo || !bo->shadow) return -EINVAL;
      uint64_t insert = __atomic_load_n((uint64_t *)((uint8_t *)q->shm + 4096 + q->slot), __ATOMIC_ACQUIRE);
      if ((insert & 7) || insert < q->insert || insert - q->insert > bo->size) return -EINVAL;
      memcpy(bo->gpu_mapping, bo->ptr, bo->size);
      struct kbase_ioctl_mem_sync s = {.handle = bo->gpu_va,
         .user_addr = (uintptr_t)bo->gpu_mapping, .size = bo->size, .type = BASE_SYNCSET_OP_MSYNC};
      if (ioctl(c->base.fd, KBASE_IOCTL_MEM_SYNC, &s)) return -errno;
      int sync_ret = shared_sync(c, DMA_BUF_SYNC_END);
      if (sync_ret) return sync_ret;
      __asm__ volatile("dsb sy" ::: "memory");
      *(volatile uint64_t *)((uint8_t *)q->io + 4096 + q->slot) = insert;
      __asm__ volatile("dsb sy" ::: "memory");
      struct kbase_ioctl_cs_queue_kick kick = {.buffer_gpu_addr = va};
      if (ioctl(c->base.fd, KBASE_IOCTL_CS_QUEUE_KICK, &kick)) return -errno;
      q->insert = insert;
      queue_update(q); return 0;
   }
   if (req->handle != 0 || q || req->size != 65536 || req->offset >= 16) return -EINVAL;
   unsigned slot;
   for (slot = 0; slot < 32 && c->queue[slot].io; slot++);
   if (slot == 32 || c->next_queue == 0x7fffffff) return -ENOSPC;
   struct kb_bo *bo = NULL;
   for (unsigned i = 0; i < MAX_BO; i++)
      if (c->bo[i].ptr && c->bo[i].gpu_va == va && c->bo[i].size == req->size && c->bo[i].shadow) {
         bo = &c->bo[i]; break;
      }
   if (!bo) return -EINVAL;
   uint32_t group; memcpy(&group, req->data + 8, 4);
   if (group > 255) return -EINVAL;
   struct kbase_ioctl_cs_queue_register reg = {.buffer_gpu_addr = va, .buffer_size = req->size, .priority = 1};
   if (ioctl(c->base.fd, KBASE_IOCTL_CS_QUEUE_REGISTER, &reg)) return -errno;
   union kbase_ioctl_cs_queue_bind bind = {.in = {.buffer_gpu_addr = va,
      .group_handle = group, .csi_index = req->offset}};
   int fd = -1, ret;
   void *io = MAP_FAILED, *shm = MAP_FAILED;
   if (ioctl(c->base.fd, KBASE_IOCTL_CS_QUEUE_BIND, &bind)) goto fail;
   io = mmap(NULL, 12288, PROT_READ | PROT_WRITE, MAP_SHARED, c->base.fd, bind.out.mmap_handle);
   if (io == MAP_FAILED) goto fail;
   fd = memfd_create("kbase-queue-io", MFD_CLOEXEC | MFD_ALLOW_SEALING);
   if (fd < 0 || ftruncate(fd, 12288) || fcntl(fd, F_ADD_SEALS, F_SEAL_SEAL | F_SEAL_SHRINK | F_SEAL_GROW)) goto fail;
   shm = mmap(NULL, 12288, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
   if (shm == MAP_FAILED) goto fail;
   q = &c->queue[slot];
   *q = (struct kb_queue){.io = io, .shm = shm, .fd = fd, .bo = bo->handle,
      .ring_va = va, .slot = 16 * req->offset, .id = 0x80000000u | ++c->next_queue};
   rsp->handle = q->id; queue_update(q); return 0;
fail:
   ret = -errno;
   if (shm != MAP_FAILED) munmap(shm, 12288);
   if (fd >= 0) close(fd);
   if (io != MAP_FAILED) munmap(io, 12288);
   struct kbase_ioctl_cs_queue_terminate term = {.buffer_gpu_addr = va};
   ioctl(c->base.fd, KBASE_IOCTL_CS_QUEUE_TERMINATE, &term);
   return ret;
}
