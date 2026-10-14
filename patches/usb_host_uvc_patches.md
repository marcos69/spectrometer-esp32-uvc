# Local patches to `espressif__usb_host_uvc`

This repository does **not** commit Espressif's `usb_host_uvc` component
(Apache-2.0); the component manager fetches it. Three local changes are
required for this camera (a "NOEOF" device: it never toggles FID and never
sets the EoF header bit — frames must be delimitied in software, and its
JPEG encoder restarts unfinished proto-frames ~every period, emitting stray
`FF D8` markers inside entropy data).

After `idf.py` installs the dependency (pinned version: **2.5.1**, see
`dependencies.lock`), reapply these patches to
`managed_components/espressif__usb_host_uvc/`.

> Apache-2.0 §4 notice: the following files of the upstream component are
> modified locally: `uvc_isoc.c`, `private_include/uvc_types_priv.h`,
> `CMakeLists.txt`. The upstream Apache-2.0 license and NOTICE files are kept
> intact within the component.

## 1. `CMakeLists.txt` — add `esp_timer` to PRIV_REQUIRES

```cmake
idf_component_register(SRCS "uvc_isoc.c" ...
                       PRIV_REQUIRES heap esp_timer ...)
```

The timestamps below come from `esp_timer`; upstream never needed it.

## 2. `private_include/uvc_types_priv.h` — two fields in the `single_thread` struct

```c
int64_t frame_start_us;   // When the frame being built started arriving (esp_timer)
int64_t last_append_us;   // When the frame being built last received data (esp_timer)
```

## 3. `uvc_isoc.c` — software frame delimiting + false-SOI gate

### 3a. NOEOF deliver/start helpers (file scope, above `isoc_transfer_callback`)

```c
/* ---- NOEOF helpers: deliver the frame being built, then start fresh ----
 * Below-protocol cameras never set EoF and never toggle FID after stream
 * start; MJPEG is covered by the SOI gate, everything else (YUY2) needs
 * clock-based splitting. Both paths share this deliver/start logic. */
static void noeof_deliver_open_frame(uvc_stream_t *uvc_stream)
{
    UVC_ENTER_CRITICAL();
    uvc_host_frame_t *prev_frame = uvc_stream->dynamic.current_frame;
    uvc_stream->dynamic.current_frame = NULL; // stop writes
    UVC_EXIT_CRITICAL();
    if (prev_frame != NULL && prev_frame->data_len > 0 &&
            uvc_stream->dynamic.streaming && uvc_stream->constant.frame_cb) {
        const bool frame_returned = uvc_stream->constant.frame_cb(
            prev_frame, uvc_stream->constant.cb_arg);
        if (frame_returned) {
            uvc_host_frame_return(uvc_stream, prev_frame);
        }
    } else if (prev_frame != NULL) {
        uvc_host_frame_return(uvc_stream, prev_frame);
    }
    uvc_stream->single_thread.skip_current_frame = false;
}

static void noeof_start_new_frame(uvc_stream_t *uvc_stream, int64_t now_us)
{
    UVC_ENTER_CRITICAL();
    const bool need_new_frame = (uvc_stream->dynamic.streaming &&
                                 !uvc_stream->dynamic.current_frame);
    UVC_EXIT_CRITICAL();
    if (need_new_frame) {
        uvc_stream->dynamic.current_frame = uvc_frame_get_empty(uvc_stream);
        if (uvc_stream->dynamic.current_frame == NULL) {
            uvc_stream->single_thread.skip_current_frame = true;
            uvc_host_stream_callback_t stream_cb = uvc_stream->constant.stream_cb;
            if (stream_cb) {
                const uvc_host_stream_event_data_t event = {
                    .type = UVC_HOST_FRAME_BUFFER_UNDERFLOW,
                };
                stream_cb(&event, uvc_stream->constant.cb_arg);
            }
        }
    }
    uvc_stream->single_thread.frame_start_us = now_us;
}
```

Also initialize per transfer batch: where upstream stamps
`frame_start_us = now_us` when starting a frame, also stamp
`uvc_stream->single_thread.last_append_us = now_us;` (line ~88:
`uvc_stream->single_thread.frame_start_us = now_us;` in the
frame-reset path).

### 3b. Clock-based split for uncompressed (YUY2) — inside the packet loop,
right after the per-packet payload pointer derivation and BEFORE the
zero-length-packet skip (idle packets must also trigger it):

```c
// SOFTWARE FRAME DELIMITING (uncompressed formats): when the camera
// never toggles FID and never sets EoF, only the clock separates raw
// frames. Release when complete, or when the wire goes quiet (>=90%
// received and idle >=100 ms), or after a generous fail-safe.
{
    const int64_t now_us = esp_timer_get_time();
    if (payload_data_len > 0 && uvc_stream->dynamic.current_frame) {
        uvc_stream->single_thread.last_append_us = now_us;
    }
    const int64_t since_start = now_us -
                                uvc_stream->single_thread.frame_start_us;
    const int64_t idle_us = now_us -
                            uvc_stream->single_thread.last_append_us;
    const float fps = uvc_stream->dynamic.vs_format.fps;
    const int64_t period_us = (fps > 0.1f) ? (int64_t)(1000000.0f / fps)
                                           : 200000;
    uvc_host_frame_t *cf = UVC_ATOMIC_LOAD(uvc_stream->dynamic.current_frame);
    const bool complete = cf != NULL &&
        cf->data_len >= uvc_stream->dynamic.dwMaxVideoFrameSize;
    const bool idle_done = cf != NULL &&
        cf->data_len * 10 >= uvc_stream->dynamic.dwMaxVideoFrameSize * 9 &&
        idle_us >= 100000;
    const bool stale = cf != NULL && cf->data_len > 0 &&
                       since_start >= period_us * 15;
    if (uvc_stream->dynamic.vs_format.format != UVC_VS_FORMAT_MJPEG &&
            (complete || idle_done || stale)) {
        noeof_deliver_open_frame(uvc_stream);
        noeof_start_new_frame(uvc_stream, now_us);
        /* fall through: this packet appends to the fresh frame */
    }
}
```

### 3c. False-SOI gate on the MJPEG SOI split — upstream's MJPEG branch
splits the open frame whenever a payload starts with `FF D8`. This camera's
un-stuffed `FF D8` pairs inside the entropy data would shred every frame.
Gate the split on the age of the frame being built (≥ 350 ms = one frame
period — a stray SOI lands a few packets later):

```c
/* False-SOI gate: NOEOF cameras with huge MJPEG frames emit raw
 * FF D8 pairs inside the entropy-coded data. A real frame boundary is
 * >= one frame period after the current frame started. */
const bool frame_open = (uvc_stream->dynamic.current_frame != NULL &&
                         uvc_stream->dynamic.current_frame->data_len > 0);
const bool soi_is_real = !frame_open ||
        (esp_timer_get_time() - uvc_stream->single_thread.frame_start_us) >= 350000;
if (!soi_is_real) {
    goto append_to_frame; /* false SOI: append to the open frame */
}
```

Place it at the top of the existing "SOI at payload start" split branch; label
the append target so the `goto append_to_frame` lands on upstream's normal
append path.

### Why not upstream these?

They encode behavior specific to this camera family (per-frame restarts,
missing FID/EoF, ~300 KB/s encode budget). If you hit the same symptoms
(`soi` counters exploding, frames only deliverable by timeout), this patch
set is the starting point — upstream `uvc_host.c` also assumes EoF/FID
timers that this device ignores.
