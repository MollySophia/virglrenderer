/* SPDX-License-Identifier: MIT */
/* Experimental bounded control transport and validated CSF store probe. */
#include <errno.h>
#include <fcntl.h>
#include <linux/udmabuf.h>
#include <linux/dma-buf.h>
#include <poll.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "drm_context.h"
#include "mali_kbase_renderer.h"
#include "mali_kbase_ioctl.h"

#define MAX_BO 1024
#define MAX_BYTES (64u * 1024u * 1024u)
#define PAYLOAD 1024
struct kb_req {
   struct vdrm_ccmd_req hdr;
   uint32_t handle, offset, size, reserved;
   uint8_t data[PAYLOAD];
};
struct kb_rsp {
   struct vdrm_ccmd_rsp hdr;
   int32_t status;
   uint32_t handle, size;
   uint64_t value;
   uint8_t data[PAYLOAD];
};
struct kb_bo {
   void *ptr, *gpu_mapping;
   uint32_t size, handle;
   int memfd, dmabuf;
   bool shared, exported;
   bool shadow, same_va;
   uint64_t gpu_va;
};
struct kb_ctx {
   struct drm_context base;
   void *tracking;
   uint64_t gpu_id;
   uint32_t allocated, next_handle, limit, bo_limit;
   struct kb_bo bo[MAX_BO];
   struct kb_queue {
      void *io, *shm;
      int fd;
      uint32_t id, bo, slot;
      uint64_t ring_va, insert;
      bool exported;
   } queue[32];
   uint32_t next_queue;
   struct { void *ptr; uint64_t size; } alias[32];
};

static int handshake(int fd, struct kbase_ioctl_version_check *v)
{
   struct kbase_ioctl_set_flags flags = {0};
   if (ioctl(fd, KBASE_IOCTL_VERSION_CHECK_CSF, v) ||
       ioctl(fd, KBASE_IOCTL_SET_FLAGS, &flags)) return -errno;
   return 0;
}
static int gpu_id(int fd, uint64_t *id)
{
   struct kbase_ioctl_get_gpuprops p = {0};
   int n = ioctl(fd, KBASE_IOCTL_GET_GPUPROPS, &p);
   if (n <= 0 || n > 65536) return -EIO;
   uint8_t *buf = calloc(1, n);
   if (!buf) return -ENOMEM;
   p.buffer = (uintptr_t)buf; p.size = n;
   int ret = ioctl(fd, KBASE_IOCTL_GET_GPUPROPS, &p);
   if (ret < 0) { ret = -errno; goto out; }
   ret = -ENODEV;
   for (unsigned i = 0; i + 4 <= (unsigned)n;) {
      uint32_t key; memcpy(&key, buf + i, 4); i += 4;
      unsigned bytes = 1u << (key & 3);
      if (bytes > (unsigned)n - i) break;
      if ((key >> 2) == 55) {
         *id = 0; memcpy(id, buf + i, bytes); ret = 0; break;
      }
      i += bytes;
   }
out:
   free(buf); return ret;
}
static int shared_alloc(struct kb_ctx *c, struct kb_bo *bo, uint32_t size)
{
   int ret, memfd = -1, dma = -1, dev = -1;
   void *cpu = MAP_FAILED, *gpu = MAP_FAILED;
   memfd = memfd_create("kbase-shared", MFD_CLOEXEC | MFD_ALLOW_SEALING);
   if (memfd < 0) return -errno;
   if (ftruncate(memfd, size) ||
       fcntl(memfd, F_ADD_SEALS, F_SEAL_SEAL | F_SEAL_SHRINK | F_SEAL_GROW)) goto fail;
   dev = open("/dev/udmabuf", O_RDWR | O_CLOEXEC);
   if (dev < 0) goto fail;
   struct udmabuf_create u = {.memfd = memfd, .flags = UDMABUF_FLAGS_CLOEXEC, .size = size};
   dma = ioctl(dev, UDMABUF_CREATE, &u);
   if (dma < 0) goto fail;
   close(dev); dev = -1;
   union kbase_ioctl_mem_import imp = {.in = {
      .flags = BASE_MEM_PROT_CPU_RD | BASE_MEM_PROT_CPU_WR |
               BASE_MEM_PROT_GPU_RD | BASE_MEM_PROT_GPU_WR |
               BASE_MEM_IMPORT_SHARED | BASE_MEM_COHERENT_LOCAL | BASE_MEM_CACHED_CPU,
      .phandle = (uintptr_t)&dma, .type = BASE_MEM_IMPORT_TYPE_UMM,
   }};
   ret = ioctl(c->base.fd, KBASE_IOCTL_MEM_IMPORT, &imp);
   if (ret && errno == ENOTTY) ret = ioctl(c->base.fd, KBASE_IOCTL_MEM_IMPORT_LEGACY, &imp);
   if (ret) goto fail;
   if (!(imp.out.flags & (BASE_MEM_SAME_VA | BASE_MEM_NEED_MMAP)) || imp.out.va_pages != size / 4096) {
      struct kbase_ioctl_mem_free f = {.gpu_addr = imp.out.gpu_va};
      ioctl(c->base.fd, KBASE_IOCTL_MEM_FREE, &f); errno = ENOTSUP; goto fail;
   }
   gpu = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, c->base.fd, imp.out.gpu_va);
   if (gpu == MAP_FAILED) {
      int saved = errno;
      struct kbase_ioctl_mem_free f = {.gpu_addr = imp.out.gpu_va};
      ioctl(c->base.fd, KBASE_IOCTL_MEM_FREE, &f); errno = saved; goto fail;
   }
   cpu = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, 0);
   if (cpu == MAP_FAILED) goto fail;
   struct dma_buf_sync sync = {.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW};
   if (ioctl(dma, DMA_BUF_IOCTL_SYNC, &sync)) goto fail;
   *bo = (struct kb_bo){.ptr = cpu, .gpu_mapping = gpu, .size = size,
                        .memfd = memfd, .dmabuf = dma, .shared = true,
                        .same_va = true, .gpu_va = (uintptr_t)gpu};
   return 0;
fail:
   ret = -errno;
   if (gpu != MAP_FAILED) munmap(gpu, size);
   if (cpu != MAP_FAILED) munmap(cpu, size);
   if (dma >= 0) close(dma);
   if (dev >= 0) close(dev);
   close(memfd);
   return ret;
}
static int shadow_alloc(struct kb_ctx *c, struct kb_bo *bo, uint32_t size, uint64_t flags)
{
   union kbase_ioctl_mem_alloc_ex a = {.in = {.va_pages = size / 4096,
      .commit_pages = size / 4096, .flags = flags}};
   int ret = ioctl(c->base.fd, KBASE_IOCTL_MEM_ALLOC_EX, &a);
   if (ret && errno == ENOTTY) ret = ioctl(c->base.fd, KBASE_IOCTL_MEM_ALLOC_EX_1_9, &a);
   if (ret) return -errno;
   void *gpu = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, c->base.fd, a.out.gpu_va);
   if (gpu == MAP_FAILED) {
      ret = -errno;
      struct kbase_ioctl_mem_free f = {.gpu_addr = a.out.gpu_va};
      ioctl(c->base.fd, KBASE_IOCTL_MEM_FREE, &f); return ret;
   }
   bool same = !!(a.out.flags & BASE_MEM_SAME_VA);
   int memfd = memfd_create("kbase-shadow", MFD_CLOEXEC | MFD_ALLOW_SEALING);
   void *cpu = MAP_FAILED;
   if (memfd >= 0 && !ftruncate(memfd, size) &&
       !fcntl(memfd, F_ADD_SEALS, F_SEAL_SEAL | F_SEAL_SHRINK | F_SEAL_GROW))
      cpu = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, 0);
   if (cpu == MAP_FAILED) {
      ret = -errno;
      if (memfd >= 0) close(memfd);
      munmap(gpu, size);
      if (!same) {
         struct kbase_ioctl_mem_free f = {.gpu_addr = a.out.gpu_va};
         ioctl(c->base.fd, KBASE_IOCTL_MEM_FREE, &f);
      }
      return ret;
   }
   *bo = (struct kb_bo){.ptr = cpu, .gpu_mapping = gpu, .size = size,
      .memfd = memfd, .dmabuf = -1, .shared = true, .shadow = true, .same_va = same,
      .gpu_va = same ? (uintptr_t)gpu : a.out.gpu_va};
   return 0;
}
static int shared_sync(struct kb_ctx *c, uint64_t operation)
{
   for (unsigned i = 0; i < MAX_BO; i++) {
      if (!c->bo[i].ptr || !c->bo[i].shared || c->bo[i].shadow) continue;
      struct dma_buf_sync sync = {.flags = DMA_BUF_SYNC_RW | operation};
      if (ioctl(c->bo[i].dmabuf, DMA_BUF_IOCTL_SYNC, &sync)) return -errno;
   }
   return 0;
}
static void free_bo(int fd, struct kb_bo *bo)
{
   munmap(bo->ptr, bo->size);
   if (bo->shared) {
      munmap(bo->gpu_mapping, bo->size);
      if (bo->dmabuf >= 0) close(bo->dmabuf);
      close(bo->memfd);
      if (bo->shadow && !bo->same_va) {
         struct kbase_ioctl_mem_free f = {.gpu_addr = bo->gpu_va};
         ioctl(fd, KBASE_IOCTL_MEM_FREE, &f);
      }
   }
   memset(bo, 0, sizeof(*bo));
}
/* Deliberately bounded CSF experiment: validate one store-only command stream,
 * copy it into a private ring, submit, and retire only after the ring drains.
 * No arbitrary guest command buffers, addresses, loops, or shader dispatch. */
static int csf_store(struct kb_ctx *c, struct kb_bo *bo, const struct kb_req *q,
                     uint64_t *extract_out)
{
   if (!bo->shared || q->size != 64 || (q->offset & 3) ||
       q->offset > bo->size - 4) return -EINVAL;
   uint64_t words[8]; memcpy(words, q->data, sizeof(words));
   uint64_t addr = (uintptr_t)bo->gpu_mapping + q->offset;
   if (addr >> 48) return -ENOTSUP;
   uint64_t expected[8] = {
      (1ull << 56) | addr,                        /* MOVE48 d0, GPU address */
      (2ull << 56) | (2ull << 48) | (uint32_t)words[1], /* MOVE32 r2, value */
      (21ull << 56) | (2ull << 48) | (1ull << 16), /* STORE_MULTIPLE r2, [d0] */
      (3ull << 56) | (0xffffull << 16),           /* WAIT all scoreboards */
      (2ull << 56) | (4ull << 48),                /* latest flush id = 0 */
      (36ull << 56) | (1ull << 48) | (4ull << 40) | 0x33, /* clean/invalidate */
      (3ull << 56) | (0xffffull << 16),
      0,                                         /* NOP */
   };
   if (memcmp(words, expected, sizeof(words))) return -EINVAL;
   int fd = c->base.fd, ret = 0;
   void *ring = MAP_FAILED, *io = MAP_FAILED;
   bool group_created = false, registered = false;
   uint8_t group = 0;
   union kbase_ioctl_mem_alloc_ex alloc = {.in = {
      .va_pages = 16, .commit_pages = 16,
      .flags = BASE_MEM_PROT_CPU_RD | BASE_MEM_PROT_CPU_WR |
               BASE_MEM_PROT_GPU_RD | BASE_MEM_PROT_GPU_WR |
               BASE_MEM_CACHED_CPU | BASE_MEM_COHERENT_LOCAL | BASE_MEM_SAME_VA,
   }};
   ret = ioctl(fd, KBASE_IOCTL_MEM_ALLOC_EX, &alloc);
   if (ret && errno == ENOTTY) ret = ioctl(fd, KBASE_IOCTL_MEM_ALLOC_EX_1_9, &alloc);
   if (ret) return -errno;
   if (!(alloc.out.flags & BASE_MEM_SAME_VA)) {
      struct kbase_ioctl_mem_free f = {.gpu_addr = alloc.out.gpu_va};
      ioctl(fd, KBASE_IOCTL_MEM_FREE, &f); return -ENOTSUP;
   }
   ring = mmap(NULL, 65536, PROT_READ | PROT_WRITE, MAP_SHARED, fd, alloc.out.gpu_va);
   if (ring == MAP_FAILED) {
      ret = -errno;
      struct kbase_ioctl_mem_free f = {.gpu_addr = alloc.out.gpu_va};
      ioctl(fd, KBASE_IOCTL_MEM_FREE, &f); return ret;
   }
   memcpy(ring, words, sizeof(words));
   struct kbase_ioctl_mem_sync sync = {.handle = (uintptr_t)ring,
      .user_addr = (uintptr_t)ring, .size = 65536, .type = BASE_SYNCSET_OP_MSYNC};
   if (ioctl(fd, KBASE_IOCTL_MEM_SYNC, &sync)) { ret = -errno; goto out; }
   union kbase_ioctl_cs_queue_group_create_1_18 g = {.in = {
      .tiler_mask = 1, .fragment_mask = ~0ull, .compute_mask = ~0ull,
      .cs_min = 1, .priority = 1, .tiler_max = 1, .fragment_max = 16, .compute_max = 16,
   }};
   if (ioctl(fd, KBASE_IOCTL_CS_QUEUE_GROUP_CREATE_1_18, &g)) { ret = -errno; goto out; }
   group = g.out.group_handle; group_created = true;
   struct kbase_ioctl_cs_queue_register reg = {.buffer_gpu_addr = (uintptr_t)ring,
      .buffer_size = 65536, .priority = 1};
   if (ioctl(fd, KBASE_IOCTL_CS_QUEUE_REGISTER, &reg)) { ret = -errno; goto out; }
   registered = true;
   union kbase_ioctl_cs_queue_bind bind = {.in = {
      .buffer_gpu_addr = (uintptr_t)ring, .group_handle = group, .csi_index = 0}};
   if (ioctl(fd, KBASE_IOCTL_CS_QUEUE_BIND, &bind)) { ret = -errno; goto out; }
   io = mmap(NULL, 12288, PROT_READ | PROT_WRITE, MAP_SHARED, fd, bind.out.mmap_handle);
   if (io == MAP_FAILED) { ret = -errno; goto out; }
   /* The guest ends its CPU access through KB_CPU_SYNC before submission. */
   __asm__ volatile("dsb sy" ::: "memory");
   *(volatile uint64_t *)((uint8_t *)io + 4096) = sizeof(words);
   __asm__ volatile("dsb sy" ::: "memory");
   struct kbase_ioctl_cs_queue_kick kick = {.buffer_gpu_addr = (uintptr_t)ring};
   if (ioctl(fd, KBASE_IOCTL_CS_QUEUE_KICK, &kick)) { ret = -errno; goto out; }
   ret = -ETIMEDOUT;
   struct timespec start, now;
   clock_gettime(CLOCK_MONOTONIC, &start);
   for (;;) {
      *extract_out = *(volatile uint64_t *)((uint8_t *)io + 8192);
      if (*extract_out == sizeof(words)) { ret = 0; break; }
      struct pollfd p = {.fd = fd, .events = POLLIN};
      if (poll(&p, 1, 1) > 0 && (p.revents & POLLIN)) {
         uint8_t event[64];
         ssize_t n = read(fd, event, sizeof(event));
         if (n == 64 && event[0] != 0) { ret = -EIO; break; }
      }
      clock_gettime(CLOCK_MONOTONIC, &now);
      if ((now.tv_sec - start.tv_sec) * 1000000000ll + now.tv_nsec - start.tv_nsec > 2000000000ll) break;
   }
   struct dma_buf_sync ds = {.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW};
   if (ioctl(bo->dmabuf, DMA_BUF_IOCTL_SYNC, &ds) && !ret) ret = -errno;
out:
   if (io != MAP_FAILED) munmap(io, 12288);
   if (registered) {
      struct kbase_ioctl_cs_queue_terminate term = {.buffer_gpu_addr = (uintptr_t)ring};
      ioctl(fd, KBASE_IOCTL_CS_QUEUE_TERMINATE, &term);
   }
   if (group_created) {
      struct kbase_ioctl_cs_queue_group_term term = {.group_handle = group};
      ioctl(fd, KBASE_IOCTL_CS_QUEUE_GROUP_TERMINATE, &term);
   }
   munmap(ring, 65536);
   return ret;
}
int mali_kbase_renderer_probe(int fd, struct virgl_renderer_capset_drm *caps)
{
   const char *opt = getenv("VKR_TEST_KBASE_NATIVE");
   if (!opt || strcmp(opt, "1")) return -ENODEV;
   struct kbase_ioctl_version_check v = {0};
   uint64_t id;
   int ret = handshake(fd, &v);
   if (!ret) ret = gpu_id(fd, &id);
   if (ret) return ret;
   caps->wire_format_version = 2;
   caps->version_major = v.major; caps->version_minor = v.minor;
   return 0;
}
#include "kbase_transport_queue.h"

static int command(struct drm_context *base, struct vdrm_ccmd_req *hdr)
{
   struct kb_ctx *c = (void *)base;
   const struct kb_req *q = (const void *)hdr;
   if (!base->shmem) return -EINVAL;
   struct kb_rsp *r = drm_context_rsp(base, hdr, sizeof(*r));
   if (!r) return -EINVAL;
   if (hdr->len != sizeof(*q) || (q->reserved && (hdr->cmd != 11 || q->reserved != 1))) {
      r->status = -EINVAL; return 0;
   }
   if (hdr->cmd == 0) { r->value = c->gpu_id; r->size = c->allocated; return 0; }
   if (hdr->cmd == 14) { r->status = queue_command(c, q, r); return 0; }
   if (hdr->cmd == 15) {
      uint64_t va; memcpy(&va, q->data, 8);
      if (q->handle == 1) {
         for (unsigned i = 0; i < 32; i++) {
            if ((uintptr_t)c->alias[i].ptr != va || !va) continue;
            munmap(c->alias[i].ptr, c->alias[i].size);
            memset(&c->alias[i], 0, sizeof(c->alias[i])); return 0;
         }
         r->status = -ENOENT; return 0;
      }
      if (q->handle || !q->size || (q->size & 4095) || !q->offset || q->offset > 4) {
         r->status = -EINVAL; return 0;
      }
      struct kb_bo *bo = NULL;
      for (unsigned i = 0; i < MAX_BO; i++)
         if (c->bo[i].ptr && c->bo[i].gpu_va == va && q->size <= c->bo[i].size && c->bo[i].shadow)
            bo = &c->bo[i];
      if (!bo) { r->status = -EINVAL; return 0; }
      unsigned slot;
      for (slot = 0; slot < 32 && c->alias[slot].ptr; slot++);
      if (slot == 32) { r->status = -ENOSPC; return 0; }
      struct base_mem_aliasing_info ai[4] = {0};
      for (unsigned i = 0; i < q->offset; i++)
         ai[i] = (struct base_mem_aliasing_info){.handle = va, .length = q->size / 4096};
      uint64_t total = (uint64_t)q->size * q->offset;
      for (unsigned attempt = 0; attempt < 16; attempt++) {
         union kbase_ioctl_mem_alias a = {.in = {
            .flags = BASE_MEM_PROT_GPU_RD | BASE_MEM_PROT_GPU_WR | BASE_MEM_PROT_CPU_RD,
            .stride = q->size / 4096, .nents = q->offset, .aliasing_info = (uintptr_t)ai}};
         if (ioctl(base->fd, KBASE_IOCTL_MEM_ALIAS, &a)) { r->status = -errno; return 0; }
         void *ptr = mmap(NULL, total, PROT_READ, MAP_SHARED, base->fd, a.out.gpu_va);
         if (ptr == MAP_FAILED) {
            r->status = -errno;
            struct kbase_ioctl_mem_free f = {.gpu_addr = a.out.gpu_va};
            ioctl(base->fd, KBASE_IOCTL_MEM_FREE, &f); return 0;
         }
         if (((uintptr_t)ptr >> 32) == (((uintptr_t)ptr + total - 1) >> 32)) {
            c->alias[slot].ptr = ptr; c->alias[slot].size = total;
            r->value = (uintptr_t)ptr; return 0;
         }
         munmap(ptr, total);
      }
      r->status = -ENOMEM; return 0;
   }
   if (hdr->cmd == 9) {
      struct kbase_ioctl_get_gpuprops p = {0};
      int n = ioctl(base->fd, KBASE_IOCTL_GET_GPUPROPS, &p);
      if (n <= 0 || n > 65536) { r->status = -EIO; return 0; }
      r->value = n;
      if (q->offset > (uint32_t)n || q->size > PAYLOAD) { r->status = -EINVAL; return 0; }
      if (!q->size) return 0;
      uint8_t *data = calloc(1, n);
      if (!data) { r->status = -ENOMEM; return 0; }
      p.buffer = (uintptr_t)data; p.size = n;
      if (ioctl(base->fd, KBASE_IOCTL_GET_GPUPROPS, &p) < 0) r->status = -errno;
      else {
         r->size = q->size < (uint32_t)n - q->offset ? q->size : (uint32_t)n - q->offset;
         memcpy(r->data, data + q->offset, r->size);
      }
      free(data); return 0;
   }
   if (hdr->cmd == 10) {
      union kbase_ioctl_cs_get_glb_iface g = {0};
      if (ioctl(base->fd, KBASE_IOCTL_CS_GET_GLB_IFACE, &g)) { r->status = -errno; return 0; }
      uint32_t ng = g.out.group_num, ns = g.out.total_stream_num;
      if (!ng || !ns || ng > 64 || ns > 1024) { r->status = -EOVERFLOW; return 0; }
      struct basep_cs_group_control *groups = calloc(ng, sizeof(*groups));
      struct basep_cs_stream_control *streams = calloc(ns, sizeof(*streams));
      if (!groups || !streams) { free(groups); free(streams); r->status = -ENOMEM; return 0; }
      memset(&g, 0, sizeof(g));
      g.in.max_group_num = ng; g.in.max_total_stream_num = ns;
      g.in.groups_ptr = (uintptr_t)groups; g.in.streams_ptr = (uintptr_t)streams;
      if (ioctl(base->fd, KBASE_IOCTL_CS_GET_GLB_IFACE, &g)) r->status = -errno;
      else {
         uint32_t sb = (streams[0].features >> 8) & 255;
         uint32_t info[6] = {ng, groups[0].stream_num, (streams[0].features & 255) + 1,
                            (sb - 1) < 16 ? sb : 8, 4, 0};
         memcpy(r->data, info, sizeof(info)); r->size = sizeof(info);
      }
      free(groups); free(streams); return 0;
   }
   if (hdr->cmd == 13) {
      if (q->size != _IOC_SIZE(q->handle) || q->size > PAYLOAD) { r->status = -EINVAL; return 0; }
      switch (q->handle) {
      case KBASE_IOCTL_MEM_JIT_INIT:
      case KBASE_IOCTL_MEM_EXEC_INIT:
      case KBASE_IOCTL_CS_QUEUE_GROUP_CREATE:
      case KBASE_IOCTL_CS_QUEUE_GROUP_CREATE_1_19:
      case KBASE_IOCTL_CS_QUEUE_GROUP_CREATE_1_18:
      case KBASE_IOCTL_CS_QUEUE_GROUP_CREATE_1_6:
      case KBASE_IOCTL_CS_QUEUE_GROUP_TERMINATE:
      case KBASE_IOCTL_CS_TILER_HEAP_INIT:
      case KBASE_IOCTL_CS_TILER_HEAP_TERM:
      case KBASE_IOCTL_GET_CPU_GPU_TIMEINFO:
         memcpy(r->data, q->data, q->size);
         if (ioctl(base->fd, q->handle, r->data)) r->status = -errno;
         r->size = q->size; return 0;
      default:
         r->status = -ENOTSUP; return 0;
      }
   }
   if (hdr->cmd == 1 || hdr->cmd == 5 || hdr->cmd == 11) {
      if (!q->size || q->size > c->limit || (q->size & 4095) ||
          q->size > c->limit - c->allocated) { r->status = -EINVAL; return 0; }
      unsigned slot;
      for (slot = 0; slot < c->bo_limit && c->bo[slot].ptr; slot++);
      if (slot == c->bo_limit || c->next_handle == UINT32_MAX) { r->status = -ENOSPC; return 0; }
      if (hdr->cmd == 5 || hdr->cmd == 11) {
         uint64_t flags = q->handle;
         const uint64_t allowed = BASE_MEM_PROT_CPU_RD | BASE_MEM_PROT_CPU_WR |
            BASE_MEM_PROT_GPU_RD | BASE_MEM_PROT_GPU_WR | BASE_MEM_PROT_GPU_EX |
            BASE_MEM_COHERENT_LOCAL | BASE_MEM_CACHED_CPU | BASE_MEM_SAME_VA |
            BASE_MEM_CSF_EVENT | BASE_MEM_UNCACHED_GPU | BASE_MEM_COHERENT_SYSTEM;
         if (hdr->cmd == 11 && (flags & ~allowed)) { r->status = -ENOTSUP; return 0; }
         if (q->reserved > 1 || (q->reserved && (flags & (BASE_MEM_PROT_GPU_EX | BASE_MEM_CSF_EVENT)))) {
            r->status = -EINVAL; return 0;
         }
         r->status = hdr->cmd == 11 && !q->reserved ? shadow_alloc(c, &c->bo[slot], q->size, flags) :
                                     shared_alloc(c, &c->bo[slot], q->size);
         if (r->status) return 0;
         c->bo[slot].handle = ++c->next_handle;
         c->allocated += q->size;
         r->handle = c->next_handle; r->size = q->size;
         /* Experimental SAME_VA address, for native GPU commands only. */
         r->value = c->bo[slot].gpu_va;
         return 0;
      }
      union kbase_ioctl_mem_alloc_ex a = {.in = {
         .va_pages = q->size / 4096, .commit_pages = q->size / 4096,
         .flags = BASE_MEM_PROT_CPU_RD | BASE_MEM_PROT_CPU_WR |
                  BASE_MEM_PROT_GPU_RD | BASE_MEM_PROT_GPU_WR |
                  BASE_MEM_COHERENT_LOCAL | BASE_MEM_SAME_VA,
      }};
      int ret = ioctl(base->fd, KBASE_IOCTL_MEM_ALLOC_EX, &a);
      if (ret && errno == ENOTTY) ret = ioctl(base->fd, KBASE_IOCTL_MEM_ALLOC_EX_1_9, &a);
      if (ret) { r->status = -errno; return 0; }
      if (!(a.out.flags & BASE_MEM_SAME_VA)) {
         struct kbase_ioctl_mem_free f = {.gpu_addr = a.out.gpu_va};
         ioctl(base->fd, KBASE_IOCTL_MEM_FREE, &f); r->status = -ENOTSUP; return 0;
      }
      void *ptr = mmap(NULL, q->size, PROT_READ | PROT_WRITE, MAP_SHARED, base->fd, a.out.gpu_va);
      if (ptr == MAP_FAILED) {
         r->status = -errno;
         struct kbase_ioctl_mem_free f = {.gpu_addr = a.out.gpu_va};
         ioctl(base->fd, KBASE_IOCTL_MEM_FREE, &f); return 0;
      }
      /* Do not expose stale memory or host virtual addresses to the guest. */
      memset(ptr, 0, q->size);
      c->bo[slot] = (struct kb_bo){.ptr = ptr, .size = q->size, .handle = ++c->next_handle};
      c->allocated += q->size;
      r->handle = c->next_handle; r->size = q->size;
      return 0;
   }
   struct kb_bo *bo = NULL;
   for (unsigned i = 0; i < MAX_BO; i++)
      if (c->bo[i].ptr && c->bo[i].handle == q->handle) { bo = &c->bo[i]; break; }
   if (!bo) { r->status = -ENOENT; return 0; }
   if (hdr->cmd == 12) {
      if (!bo->shared || q->offset > bo->size || q->size > bo->size - q->offset ||
          (q->data[0] != BASE_SYNCSET_OP_MSYNC && q->data[0] != BASE_SYNCSET_OP_CSYNC)) {
         r->status = -EINVAL; return 0;
      }
      if (!bo->shadow) {
         struct dma_buf_sync sync = {.flags = DMA_BUF_SYNC_RW |
            (q->data[0] == BASE_SYNCSET_OP_CSYNC ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END)};
         if (ioctl(bo->dmabuf, DMA_BUF_IOCTL_SYNC, &sync)) r->status = -errno;
         return 0;
      }
      void *cpu = (uint8_t *)bo->ptr + q->offset;
      void *gpu = (uint8_t *)bo->gpu_mapping + q->offset;
      if (q->data[0] == BASE_SYNCSET_OP_MSYNC) memcpy(gpu, cpu, q->size);
      struct kbase_ioctl_mem_sync s = {.handle = bo->gpu_va, .user_addr = (uintptr_t)gpu,
         .size = q->size, .type = q->data[0]};
      if (ioctl(base->fd, KBASE_IOCTL_MEM_SYNC, &s)) r->status = -errno;
      else if (q->data[0] == BASE_SYNCSET_OP_CSYNC) memcpy(cpu, gpu, q->size);
      if (!r->status && q->data[0] == BASE_SYNCSET_OP_CSYNC)
         r->status = shared_sync(c, DMA_BUF_SYNC_START);
      return 0;
   }
   if (hdr->cmd == 8) {
      r->status = csf_store(c, bo, q, &r->value);
      return 0;
   }
   if (hdr->cmd == 4) {
      c->allocated -= bo->size; free_bo(base->fd, bo); return 0;
   }
   if (hdr->cmd == 6) {
      if (!bo->shared || q->offset > 1) { r->status = -EINVAL; return 0; }
      struct dma_buf_sync s = {.flags = DMA_BUF_SYNC_RW |
         (q->offset ? DMA_BUF_SYNC_END : DMA_BUF_SYNC_START)};
      if (ioctl(bo->dmabuf, DMA_BUF_IOCTL_SYNC, &s)) r->status = -errno;
      return 0;
   }
   if (q->size > PAYLOAD || q->offset > bo->size || q->size > bo->size - q->offset) {
      r->status = -EINVAL; return 0;
   }
   if (hdr->cmd == 2) memcpy((uint8_t *)bo->ptr + q->offset, q->data, q->size);
   else if (hdr->cmd == 3) memcpy(r->data, (uint8_t *)bo->ptr + q->offset, q->size);
   /* Imported kbase mappings establish GPU VA; CPU access can SIGBUS.
    * CPU accesses must use the original memfd/dma-buf mapping. */
   else if (hdr->cmd == 7) { r->status = -ENOTSUP; return 0; }
   else { r->status = -EINVAL; return 0; }
   r->size = q->size;
   return 0;
}
static const struct drm_ccmd dispatch[] = {
   {"KB_INFO", command, sizeof(struct kb_req)},
   {"KB_ALLOC", command, sizeof(struct kb_req)},
   {"KB_WRITE", command, sizeof(struct kb_req)},
   {"KB_READ", command, sizeof(struct kb_req)},
   {"KB_FREE", command, sizeof(struct kb_req)},
   {"KB_SHARED_ALLOC", command, sizeof(struct kb_req)},
   {"KB_CPU_SYNC", command, sizeof(struct kb_req)},
   {"KB_READ_KBASE_MAPPING", command, sizeof(struct kb_req)},
   {"KB_CSF_STORE", command, sizeof(struct kb_req)},
   {"KB_GPU_PROPS", command, sizeof(struct kb_req)},
   {"KB_CSIF_INFO", command, sizeof(struct kb_req)},
   {"KB_NATIVE_ALLOC", command, sizeof(struct kb_req)},
   {"KB_NATIVE_SYNC", command, sizeof(struct kb_req)},
   {"KB_CONTROL_IOCTL", command, sizeof(struct kb_req)},
   {"KB_QUEUE", command, sizeof(struct kb_req)},
   {"KB_ALIAS", command, sizeof(struct kb_req)},
};
static void destroy(struct virgl_context *v)
{
   struct kb_ctx *c = (void *)v;
   for (unsigned i = 0; i < 32; i++)
      if (c->queue[i].io) queue_free(c, &c->queue[i]);
   for (unsigned i = 0; i < 32; i++)
      if (c->alias[i].ptr) munmap(c->alias[i].ptr, c->alias[i].size);
   for (unsigned i = 0; i < MAX_BO; i++)
      if (c->bo[i].ptr) free_bo(c->base.fd, &c->bo[i]);
   munmap(c->tracking, 4096);
   drm_context_deinit(&c->base); free(c);
}
static void attach(struct virgl_context *v, struct virgl_resource *res) { (void)v; (void)res; }
static int blob(struct virgl_context *v, uint32_t res, uint64_t id, uint64_t size,
                uint32_t flags, struct virgl_context_blob *out)
{
   (void)res;
   if (id) {
      struct kb_ctx *c = (void *)v;
      for (unsigned i = 0; i < 32; i++) {
         struct kb_queue *q = &c->queue[i];
         if (!q->io || q->id != id) continue;
         if (q->exported || size != 12288 || flags != VIRGL_RENDERER_BLOB_FLAG_USE_MAPPABLE)
            return -EINVAL;
         int fd = fcntl(q->fd, F_DUPFD_CLOEXEC, 3);
         if (fd < 0) return -errno;
         out->type = VIRGL_RESOURCE_FD_SHM; out->u.fd = fd;
         out->map_info = VIRGL_RENDERER_MAP_CACHE_CACHED;
         q->exported = true; return 0;
      }
      for (unsigned i = 0; i < MAX_BO; i++) {
         struct kb_bo *bo = &c->bo[i];
         if (!bo->ptr || !bo->shared || bo->handle != id) continue;
         if (bo->exported || size != bo->size || flags != VIRGL_RENDERER_BLOB_FLAG_USE_MAPPABLE)
            return -EINVAL;
         int fd = fcntl(bo->memfd, F_DUPFD_CLOEXEC, 3);
         if (fd < 0) return -errno;
         out->type = VIRGL_RESOURCE_FD_SHM; out->u.fd = fd;
         out->map_info = VIRGL_RENDERER_MAP_CACHE_CACHED;
         bo->exported = true; return 0;
      }
      return -ENOENT;
   }
   if (size < 4096 || size > 65536) return -EINVAL;
   return drm_context_get_shmem_blob((void *)v, "mali_kbase", 8, size, flags, out);
}
static int fence(struct virgl_context *v, uint32_t flags, uint32_t ring, uint64_t id)
{
   (void)flags;
   if (ring) return -EINVAL;
   v->fence_retire(v, ring, id); return 0;
}
struct virgl_context *mali_kbase_renderer_create(int fd, size_t len, const char *name)
{
   (void)len; (void)name;
   struct kb_ctx *c = calloc(1, sizeof(*c));
   struct kbase_ioctl_version_check ver = {0};
   if (!c) { close(fd); return NULL; }
   c->limit = MAX_BYTES;
   c->bo_limit = 64;
   const char *objects = getenv("VKR_TEST_KBASE_MAX_BO");
   if (objects) {
      char *end;
      unsigned long count = strtoul(objects, &end, 10);
      if (*end || count < 64 || count > MAX_BO) goto fail;
      c->bo_limit = count;
   }
   const char *budget = getenv("VKR_TEST_KBASE_MAX_MB");
   if (budget) {
      char *end;
      unsigned long mb = strtoul(budget, &end, 10);
      if (*end || mb < 64 || mb > 1024) goto fail;
      c->limit = mb * 1024u * 1024u;
   }
   if (handshake(fd, &ver) || gpu_id(fd, &c->gpu_id)) goto fail;
   c->tracking = mmap(NULL, 4096, PROT_NONE, MAP_SHARED, fd, BASE_MEM_MAP_TRACKING_HANDLE);
   if (c->tracking == MAP_FAILED) goto fail;
   if (!drm_context_init(&c->base, fd, dispatch, sizeof(dispatch) / sizeof(dispatch[0]))) {
      munmap(c->tracking, 4096); goto fail;
   }
   c->base.base.destroy = destroy;
   c->base.base.attach_resource = attach;
   c->base.base.get_blob = blob;
   c->base.base.submit_fence = fence;
   return &c->base.base;
fail:
   close(fd); free(c); return NULL;
}
