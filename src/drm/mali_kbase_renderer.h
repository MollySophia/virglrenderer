/* SPDX-License-Identifier: MIT */
#pragma once
#include "drm_hw.h"
#include "virgl_context.h"

/* Private experiment, not an allocated upstream context type or Panfrost ABI. */
#define MALI_KBASE_CONTEXT 0x4b424153u
int mali_kbase_renderer_probe(int fd, struct virgl_renderer_capset_drm *caps);
struct virgl_context *mali_kbase_renderer_create(int fd, size_t len, const char *name);
