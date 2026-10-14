/* USB UVC camera capture for the ESP32-S3 spectrometer.
 *
 * Event-driven flow (mirrors the working basic_uvc_stream example):
 *   usb_lib_task pumps usb_host_lib_handle_events() — REQUIRED: without it
 *   the host library never processes device connections and nothing
 *   enumerates (symptom: uvc_host_stream_open always ESP_ERR_NOT_FOUND).
 *   uvc_host_install(event_cb) -> DEVICE_CONNECTED fires with the camera's
 *   advertised frame list -> camera task opens the stream with the
 *   camera's OWN format (best MJPEG match, else YUY2, fits 640x480).
 * - MJPEG frames decoded to 8-bit grayscale via the vendored libjpeg
 *   (components/jpeg_decode, bit-exact vs PIL)
 * - YUY2: Y plane taken directly (no decode)
 * - The middle row (half height) is fed to spectra_process_frame()
 * - The latest raw MJPEG frame is kept for the /preview endpoint
 */
#include "camera.h"
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "jpegd.h"
#include "spectra.h"
#include "nvs.h"

/* vendored header include chain breaks outside the component; only this
 * one function is public enough to need — declare/link instead */
extern esp_err_t uvc_host_usb_ctrl(uvc_host_stream_hdl_t stream_hdl,
                                   uint8_t bmRequestType, uint8_t bRequest,
                                   uint16_t wValue, uint16_t wIndex,
                                   uint16_t wLength, uint8_t *data);

#define TAG "camera"

#define USB_LIB_TASK_PRIO   15
#define UVC_DRV_TASK_PRIO   (USB_LIB_TASK_PRIO + 1)
#define CAM_TASK_PRIO       (USB_LIB_TASK_PRIO - 2)

#define MAX_FRAME_FORMATS  32

#define UVC_DESC_DWFRAMEINTERVAL_TO_FPS(dw) \
    (((dw) != 0) ? 10000000 / ((float)(dw)) : 0)

/* NVS: camera preferences (raw mode, exposure) */
#define CAM_NVS_NS "cam"
#define CAM_NVS_KEY_MODE "mode"

/* PSRAM frame buffer: decoded gray frame (max 640x480) */
static uint8_t *s_gray;          /* W*H gray, PSRAM */
static size_t   s_gray_cap;      /* allocated capacity of s_gray */
static uint32_t s_w, s_h;
static uint32_t s_gray_w;        /* width of the DECODED gray row (spectra x-axis) */
static uint8_t *s_midrow;        /* W bytes, PSRAM */

/* latest raw MJPEG frame (kept for /preview) */
static SemaphoreHandle_t s_frame_lock;
static uint8_t *s_raw;
static size_t s_raw_len;
static bool s_is_mjpeg = false;

static QueueHandle_t s_frame_q;
static uvc_host_stream_hdl_t s_stream;
static volatile bool s_streaming = false;
static uint32_t s_frame_count = 0;
static uint32_t s_drop_count = 0;    /* incomplete MJPEG frames dropped */
static uint32_t s_drop_logged = 0;   /* drop count at last log line */
static uint32_t s_decode_fail_streak = 0; /* consecutive decode failures */
static bool s_first_dumped = false;  /* one-shot head/tail dump of a good frame */
static int s_diag_bands = 0;         /* YUY2 per-frame band stats (reset per open) */
static int s_diag_win = 0;           /* row-window details (reset per open) */
static int s_diag_rsum = 0;          /* sampled row-sum profile (reset per open) */

/* camera-advertised formats, filled by the driver event callback */
static uvc_host_frame_info_t s_frame_list[MAX_FRAME_FORMATS];
static size_t s_frame_list_n = 0;
static volatile bool s_dev_connected = false;
static SemaphoreHandle_t s_fmt_signal; /* given on DEVICE_CONNECTED */

/* chosen format for status */
static volatile int s_fmt_fmt = 0;   /* enum uvc_host_stream_format */
static float s_fps_req = 0;

static void apply_exposure(void);   /* fwd: used by settings setters */

/* camera preferences: raw YUY2 (no JPEG) + manual exposure */
static bool s_raw_yuy2 = false;
static int s_exposure_us = 0;    /* 0 = camera auto */
static volatile bool s_reopen_request = false; /* raw toggle -> restart stream */
static uint32_t s_gray_w_expect = 0;

static void cam_prefs_save(void)
{
    nvs_handle_t h;
    if (nvs_open(CAM_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    uint8_t buf[8];
    buf[0] = s_raw_yuy2 ? 1 : 0;
    memcpy(buf + 1, &s_exposure_us, 4);
    nvs_set_blob(h, CAM_NVS_KEY_MODE, buf, sizeof(buf));
    nvs_commit(h);
    nvs_close(h);
}

static void cam_prefs_load(void)
{
    nvs_handle_t h;
    if (nvs_open(CAM_NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t buf[8] = {0};
    size_t len = sizeof(buf);
    if (nvs_get_blob(h, CAM_NVS_KEY_MODE, buf, &len) == ESP_OK && len == 8) {
        s_raw_yuy2 = buf[0] != 0;
        memcpy(&s_exposure_us, buf + 1, 4);
        if (s_exposure_us < 0 || s_exposure_us > 3000000) s_exposure_us = 0;
        ESP_LOGI(TAG, "camera prefs: raw=%d exposure_us=%d", s_raw_yuy2, s_exposure_us);
    }
    nvs_close(h);
}

void camera_set_raw(bool on)
{
    s_raw_yuy2 = on;
    cam_prefs_save();
    /* live re-open: break the frame loop so the task stops/closes the
     * stream and re-opens with the updated preference (pick_best_format
     * decides MJPEG-vs-YUY2 from s_raw_yuy2) */
    s_reopen_request = true;
    ESP_LOGI(TAG, "raw YUY2 mode %s (reopening stream)", on ? "ON" : "off");
}

bool camera_get_raw(void) { return s_raw_yuy2; }

void camera_set_exposure_us(int us)
{
    if (us < 0) us = 0;
    if (us > 3000000) us = 3000000;
    s_exposure_us = us;
    cam_prefs_save();
    if (s_stream && s_streaming) apply_exposure();
}

int camera_get_exposure_us(void) { return s_exposure_us; }

/* UVC exposure control. Unit/interface IDs are desriptor-specific; cheap
 * webcams almost always use Camera Terminal unit 1 on VS interface 1.
 * Try a small candidate list once, remember the winner, then just use it. */
static int s_exp_conf = -1;      /* index into candidates, -1 = unknown */
static const uint8_t EXP_UNITS[] = {1, 2};
static const uint8_t EXP_IFACES[] = {1, 0};

static bool uvc_set_cur(uvc_host_stream_hdl_t st, uint8_t cs, uint8_t unit,
                        uint8_t iface, const uint8_t *data, uint16_t len)
{
    uint16_t wValue = (uint16_t)(cs << 8);
    uint16_t wIndex = (uint16_t)(iface | (unit << 8));
    esp_err_t err = uvc_host_usb_ctrl(st, 0x21, 0x01, wValue, wIndex, len,
                                      (uint8_t *)data);
    return err == ESP_OK;
}

static bool uvc_get_cur(uvc_host_stream_hdl_t st, uint8_t cs, uint8_t unit,
                        uint8_t iface, uint8_t *data, uint16_t len)
{
    uint16_t wValue = (uint16_t)(cs << 8);
    uint16_t wIndex = (uint16_t)(iface | (unit << 8));
    esp_err_t err = uvc_host_usb_ctrl(st, 0xA1, 0x01, wValue, wIndex, len, data);
    return err == ESP_OK;
}

/* Apply manual exposure (AEC off + absolute time). Best effort: any
 * failure = keep camera auto mode. Called after each stream open.
 * Cameras that STALL every candidate are remembered and not retried. */
static bool s_exp_rejected = false;

static void apply_exposure(void)
{
    if (s_exposure_us == 0 || s_exp_rejected) return;
    uvc_host_stream_hdl_t st = s_stream;
    if (!st) return;

    uint8_t aec_off[2] = {0, 0};   /* CT-2 AEMode is a 2-byte bitmap */
    uint8_t exp_le[4];
    uint32_t units100us = (uint32_t)s_exposure_us / 100;
    exp_le[0] = units100us & 0xFF;
    exp_le[1] = (units100us >> 8) & 0xFF;
    exp_le[2] = (units100us >> 16) & 0xFF;
    exp_le[3] = (units100us >> 24) & 0xFF;

    int last_err = 0;
    (void)last_err;
    for (size_t ci = 0; ci < sizeof(EXP_UNITS) * sizeof(EXP_IFACES) / sizeof(EXP_UNITS[0]); ci++) {
        if (ci == 0 && s_exp_conf >= 0) ci = s_exp_conf; /* fast-path winner */
        uint8_t unit = EXP_UNITS[ci % 2];
        uint8_t iface = EXP_IFACES[ci / 2];
        bool ok = uvc_set_cur(st, 0x02, unit, iface, aec_off, 2) &&
                  uvc_set_cur(st, 0x04, unit, iface, exp_le, 4);
        if (ok) {
            /* verify the camera actually took the value */
            uint8_t rb[4] = {0};
            if (uvc_get_cur(st, 0x04, unit, iface, rb, 4) &&
                rb[0] == exp_le[0] && rb[1] == exp_le[1]) {
                s_exp_conf = (int)ci;
                ESP_LOGI(TAG, "exposure set: %d us (unit %u iface %u)",
                         s_exposure_us, unit, iface);
                return;
            }
            ESP_LOGI(TAG, "exposure accepted but no readback (unit %u iface %u)",
                     unit, iface);
            return;
        }
        if (s_exp_conf >= 0) break; /* winner failed this time; stop */
    }
    ESP_LOGW(TAG, "exposure control not accepted by camera (keeping auto)");
    s_exp_rejected = true;
}

static void stream_event_cb(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGW(TAG, "USB transfer error %d", event->transfer_error.error);
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "camera disconnected (task cleans up via stall check)");
        s_streaming = false;
        s_dev_connected = false;
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW: {
        /* bursts fire dozens/sec while the wire outpaces decode; log at
         * most one line per 2 s with a running total */
        static uint32_t n_over = 0, n_under = 0, n_last = 0;
        static TickType_t t_last = 0;
        if (event->type == UVC_HOST_FRAME_BUFFER_OVERFLOW) n_over++;
        else n_under++;
        const TickType_t now = xTaskGetTickCount();
        if (now - t_last > pdMS_TO_TICKS(2000)) {
            t_last = now;
            ESP_LOGW(TAG, "buffer exhaustion +%u (u%u/o%u total)",
                     (unsigned)(n_under + n_over - n_last),
                     (unsigned)n_under, (unsigned)n_over);
            n_last = n_under + n_over;
        }
        break;
    }
    default:
        break;
    }
}

/* Frame callback: push pointer into queue, return false so the caller
 * returns the buffer to the driver after processing. Tiny queue: if full,
 * the frame is dropped (returned to the driver immediately). */
static bool frame_cb(const uvc_host_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    uvc_host_frame_t *f = (uvc_host_frame_t *)frame;
    if (xQueueSend(s_frame_q, &f, 0) != pdPASS) {
        return true; /* drop */
    }
    return false;
}

static const char *fmt_str(int fmt)
{
    switch (fmt) {
    case UVC_VS_FORMAT_MJPEG: return "MJPEG";
    case UVC_VS_FORMAT_YUY2:  return "YUY2";
    case UVC_VS_FORMAT_H264:  return "H264";
    case UVC_VS_FORMAT_H265:  return "H265";
    default:                  return "UNDEF";
    }
}

/* Trim a delivered MJPEG frame: must start with SOI (FF D8); if an EOI
 * (FF D9) is present, everything after the FIRST one is cut (trailing
 * garbage / glued second image). NOEOF cameras send no EOI at all — those
 * frames are accepted as-is (the decoder consumes to end of data). */
static size_t jpeg_frame_len(const uint8_t *d, size_t len)
{
    if (len < 100 || d[0] != 0xFF || d[1] != 0xD8) return 0;
    for (size_t i = 2; i + 1 < len; i++) {
        if (d[i] == 0xFF && d[i + 1] == 0xD9) return i + 2;
    }
    return len;
}

/* Pick the best camera-advertised format.
 * Wire-budget reality (this camera): ~300 KB/s total encode/stream rate,
 * and it restarts an unfinished JPEG every frame interval — so a format
 * only works when its typical JPEG fits the per-frame budget. Bigger is
 * NOT better here: prefer the resolution closest to 320x240 (among MJPEG),
 * falling back to the old largest-that-fits logic if nothing matches. */
static int pick_best_format(void)
{
    int best = -1;
    int best_prio = 999;
    uint32_t best_px = 0;
    int mjpeg_fit = -1;
    uint32_t mjpeg_fit_px = 0;
    int yuy2 = -1;
    uint32_t yuy2_px = 0;
    for (size_t i = 0; i < s_frame_list_n; i++) {
        const uvc_host_frame_info_t *f = &s_frame_list[i];
        int prio;
        if (f->format == UVC_VS_FORMAT_MJPEG) prio = 0;
        else if (f->format == UVC_VS_FORMAT_YUY2) prio = 1;
        else continue; /* H264/H265: cannot decode */
        if (f->h_res == 0 || f->v_res == 0 ||
            f->h_res > 640 || f->v_res > 480) continue; /* never fits */
        uint32_t px = (uint32_t)f->h_res * f->v_res;
        if (prio < best_prio || (prio == best_prio && px > best_px)) {
            best = (int)i;
            best_prio = prio;
            best_px = px;
        }
        /* raw mode: uncompressed YUY2 beats any MJPEG (no JPEG quantization;
         * frame rate suffers, that is accepted for spectrometer precision) */
        if (prio == 1 && s_raw_yuy2 && (yuy2 < 0 || px > yuy2_px)) {
            yuy2 = (int)i;
            yuy2_px = px;
        }
        /* MJPEG closest to 320x240 -> best odds the encoder finishes a frame */
        if (prio == 0) {
            uint32_t d = (f->h_res > 320 ? f->h_res - 320 : 320 - f->h_res) +
                         (f->v_res > 240 ? f->v_res - 240 : 240 - f->v_res);
            if (mjpeg_fit < 0 || d < mjpeg_fit_px) {
                mjpeg_fit = (int)i;
                mjpeg_fit_px = d;
            }
        }
    }
    if (yuy2 >= 0) return yuy2;
    ESP_LOGI(TAG, "pick: yuy2=%d mfit=%d best=%d raw=%d n=%u",
             yuy2, mjpeg_fit, best, (int)s_raw_yuy2, (unsigned)s_frame_list_n);
    return (mjpeg_fit >= 0) ? mjpeg_fit : best;
}

/* Driver-level event callback (driver context: no blocking!). */
static void uvc_driver_event_cb(const uvc_host_driver_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    switch (event->type) {
    case UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED: {
        size_t n = MAX_FRAME_FORMATS;
        esp_err_t err = uvc_host_get_frame_list(event->device_connected.dev_addr,
                                                event->device_connected.uvc_stream_index,
                                                &s_frame_list, &n);
        if (err != ESP_OK || n == 0) {
            ESP_LOGE(TAG, "connected dev_addr=%d but frame list failed (%s)",
                     event->device_connected.dev_addr, esp_err_to_name(err));
            break;
        }
        s_frame_list_n = n;
        s_dev_connected = true;
        ESP_LOGI(TAG, "camera connected (addr %d): %u advertised format(s):",
                 event->device_connected.dev_addr, (unsigned)n);
        for (size_t i = 0; i < n; i++) {
            ESP_LOGI(TAG, "  [%u] %s %ux%u@%.1f",
                     (unsigned)i, fmt_str(s_frame_list[i].format),
                     s_frame_list[i].h_res, s_frame_list[i].v_res,
                     UVC_DESC_DWFRAMEINTERVAL_TO_FPS(s_frame_list[i].default_interval));
        }
        xSemaphoreGive(s_fmt_signal);
        break;
    }
    default:
        break;
    }
}

/* REQUIRED: pump the USB host library events. Without this, device
 * connections are never processed and nothing enumerates.
 * Politely: short timeout + vTaskDelay(1) so IDLE0 always gets scheduled —
 * a continuously-pending event stream (e.g. a flapping device) must not
 * starve the idle task (task watchdog). */
static void usb_lib_task(void *arg)
{
    (void)arg;
    while (1) {
        uint32_t event_flags = 0;
        usb_host_lib_handle_events(pdMS_TO_TICKS(50), &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        vTaskDelay(1); /* never spin: worst-case event latency ~51 ms */
    }
}

/* Log every enumerated USB device (any class) once per failed wait cycle. */
static void probe_usb_devices(void)
{
    uint8_t addr[8];
    int num = 0;
    esp_err_t err = usb_host_device_addr_list_fill(8, addr, &num);
    if (err != ESP_OK || num == 0) {
        ESP_LOGW(TAG, "USB probe: 0 devices on the bus "
                 "(check camera port / cable / VBUS power)");
        return;
    }
    ESP_LOGI(TAG, "USB probe: %d device(s) on the bus (no usable camera yet)",
             num);
}

static void free_buffers(void)
{
    if (s_gray) heap_caps_free(s_gray);
    if (s_midrow) heap_caps_free(s_midrow);
    s_gray = NULL;
    s_gray_cap = 0;
    s_midrow = NULL;
}

static void camera_task(void *arg)
{
    const uvc_host_driver_config_t uvc_cfg = {
        .driver_task_stack_size = 4 * 1024,
        .driver_task_priority = UVC_DRV_TASK_PRIO,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = uvc_driver_event_cb,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_cfg));

    int start_idx = 0; /* format-list index to try first on re-open */
    bool picker_wins = false; /* set when a raw-toggle reopen stops the stream */
    for (;;) {
        /* ---- wait for a camera ---- */
        while (!s_dev_connected || s_frame_list_n == 0) {
            if (xSemaphoreTake(s_fmt_signal, pdMS_TO_TICKS(5000)) != pdTRUE &&
                !s_dev_connected) {
                probe_usb_devices();
            }
        }

        int idx = pick_best_format();
        if (idx < 0) {
            ESP_LOGW(TAG, "camera has no MJPEG/YUY2 format that fits 640x480; "
                     "waiting");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        if (start_idx > 0 && start_idx < (int)s_frame_list_n) {
            ESP_LOGI(TAG, "ladder: start_idx=%d overrides pick %d", start_idx, idx);
            idx = start_idx;
        }
        const uvc_host_frame_info_t *fi = &s_frame_list[idx];

        const uvc_host_stream_config_t scfg = {
            .event_cb = stream_event_cb,
            .frame_cb = frame_cb,
            .user_ctx = NULL,
            .usb = {
                .vid = UVC_HOST_ANY_VID,
                .pid = UVC_HOST_ANY_PID,
                .uvc_stream_index = 0,
            },
            .vs_format = {
                .h_res = fi->h_res,
                .v_res = fi->v_res,
                .fps = UVC_DESC_DWFRAMEINTERVAL_TO_FPS(fi->default_interval),
                .format = fi->format,
            },
            .advanced = {
                .frame_size = 0, /* auto from descriptors */
                .number_of_frame_buffers = 5, /* burst headroom: decode may lag brief wire bursts */
                .number_of_urbs = 4,
                .urb_size = 10 * 1024,
                .frame_heap_caps = MALLOC_CAP_SPIRAM,
            },
        };
        uvc_host_stream_hdl_t st = NULL;
        esp_err_t err = uvc_host_stream_open(&scfg, pdMS_TO_TICKS(8000), &st);
        if (err != ESP_OK) {
            ESP_LOGI(TAG, "open [%d] %s %ux%u failed: %s", idx,
                     fmt_str(fi->format), fi->h_res, fi->v_res,
                     esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000)); /* never spin on instant-fail opens */
            if ((int)s_frame_list_n > 1) {
                start_idx = (idx + 1) % (int)s_frame_list_n;
            } else {
                start_idx = 0;
                probe_usb_devices();
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
            continue;
        }

        s_fmt_fmt = fi->format;
        s_fps_req = UVC_DESC_DWFRAMEINTERVAL_TO_FPS(fi->default_interval);
        s_is_mjpeg = (fi->format == UVC_VS_FORMAT_MJPEG);
        s_w = fi->h_res;
        s_h = fi->v_res;
        free_buffers();
        s_gray = heap_caps_malloc((size_t)fi->h_res * fi->v_res,
                                  MALLOC_CAP_SPIRAM);
        s_midrow = heap_caps_malloc(fi->h_res, MALLOC_CAP_SPIRAM);
        if (s_gray) s_gray_cap = (size_t)fi->h_res * fi->v_res;
        if (!s_gray || !s_midrow) {
            ESP_LOGE(TAG, "PSRAM alloc failed");
            uvc_host_stream_close(st);
            st = NULL;
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "stream opened: %s %ux%u@%.1f (list idx %d)",
                 fmt_str(fi->format), s_w, s_h, s_fps_req, idx);
        /* re-arm one-shot diagnostics so every (re)open dumps its own frame */
        s_first_dumped = false;
        s_diag_bands = 0;
        s_diag_win = 0;
        s_diag_rsum = 0;
        uvc_host_stream_start(st);
        s_stream = st;
        apply_exposure();

        /* ---- process frames until disconnect / stall ---- */
        bool got_first = false;
        TickType_t last = xTaskGetTickCount();
        s_streaming = false;
        for (;;) {
            if (s_reopen_request) {
                s_reopen_request = false;
                start_idx = 0;   /* raw toggle: pick_best_format() must decide */
                picker_wins = true;
                ESP_LOGI(TAG, "stream reopen (raw mode toggle)");
                break;
            }
            uvc_host_frame_t *fr;
            if (xQueueReceive(s_frame_q, &fr, pdMS_TO_TICKS(100)) == pdPASS) {
                last = xTaskGetTickCount();
                got_first = true;
                s_streaming = true;
                /* actual transfer dims come from the frame descriptor */
                uint32_t fw = fr->vs_format.h_res;
                uint32_t fh = fr->vs_format.v_res;
                uint32_t dw = fw, dh = fh; /* decoded gray dims (may be 1/2-scale) */
                size_t need = s_is_mjpeg ? 0 : (size_t)fw * fh * 2;
                /* tail-short tolerance: this camera's last YUY2 packets fall
                 * a few hundred bytes (<0.2%) short of the exact frame size;
                 * the spectrometer reads the middle rows, never the bottom */
                if (fw == 0 || fh == 0 || (size_t)fw * fh > s_gray_cap ||
                    (!s_is_mjpeg && fr->data_len * 50 < need * 49)) {
                    if (fw && fh)
                        ESP_LOGW(TAG, "frame %ux%u len %u: out of bounds — dropped",
                                 fw, fh, (unsigned)fr->data_len);
                    uvc_host_frame_return(st, fr);
                    continue;
                }
                if (fw != s_w || fh != s_h) {
                    ESP_LOGI(TAG, "camera actual format: %ux%u", fw, fh);
                    s_w = fw;
                    s_h = fh;
                    heap_caps_free(s_midrow);
                    s_midrow = heap_caps_malloc(fw, MALLOC_CAP_SPIRAM);
                    if (!s_midrow) {
                        ESP_LOGE(TAG, "midrow realloc failed");
                        uvc_host_frame_return(st, fr);
                        break;
                    }
                }
                if (s_is_mjpeg) {
                    /* sanity-trim first: the decoder must see one clean JPEG */
                    size_t jlen = jpeg_frame_len(fr->data, fr->data_len);
                    if (jlen == 0) {
                        s_drop_count++;
                        /* no-SOI shreds (640x480 proto storm) MUST also feed
                         * the rotation ladder or it sticks drops-only forever */
                        if (++s_decode_fail_streak >= 8) {
                            s_decode_fail_streak = 0;
                            ESP_LOGW(TAG, "%u consecutive unparsable frames -> "
                                     "rotate format", (unsigned)8);
                            break;
                        }
                        if (s_drop_count == 1 || s_drop_count - s_drop_logged >= 30) {
                            s_drop_logged = s_drop_count;
                            ESP_LOGW(TAG, "incomplete frame dropped (%u raw bytes), "
                                     "total dropped %u", (unsigned)fr->data_len,
                                     (unsigned)s_drop_count);
                            ESP_LOGW(TAG, "drop head: %02x %02x %02x %02x %02x "
                                     "%02x %02x %02x %02x %02x %02x %02x",
                                     fr->data[0], fr->data[1], fr->data[2],
                                     fr->data[3], fr->data[4], fr->data[5],
                                     fr->data[6], fr->data[7], fr->data[8],
                                     fr->data[9], fr->data[10], fr->data[11]);
                            ESP_LOGW(TAG, "drop tail: %02x %02x %02x %02x %02x "
                                     "%02x %02x %02x %02x %02x %02x %02x",
                                     fr->data[fr->data_len - 12],
                                     fr->data[fr->data_len - 11],
                                     fr->data[fr->data_len - 10],
                                     fr->data[fr->data_len - 9],
                                     fr->data[fr->data_len - 8],
                                     fr->data[fr->data_len - 7],
                                     fr->data[fr->data_len - 6],
                                     fr->data[fr->data_len - 5],
                                     fr->data[fr->data_len - 4],
                                     fr->data[fr->data_len - 3],
                                     fr->data[fr->data_len - 2],
                                     fr->data[fr->data_len - 1]);
                        }
                        uvc_host_frame_return(st, fr);
                        continue;
                    }
                    /* decode to gray (Y plane), auto 1/2-scale for big frames */
                    if (jpegd_decode_gray_auto(fr->data, jlen, s_gray,
                                               s_gray_cap, &dw, &dh) != 0) {
                        ESP_LOGW(TAG, "mjpeg decode failed (%u bytes)",
                                 (unsigned)jlen);
                        uvc_host_frame_return(st, fr);
                        /* repeated decode failure: this format is not viable
                         * for this camera's wire budget -> rotate formats */
                        if (++s_decode_fail_streak >= 8) {
                            ESP_LOGW(TAG, "%u consecutive decode failures -> "
                                     "rotate format", (unsigned)s_decode_fail_streak);
                            break;
                        }
                        continue;
                    }
                    s_decode_fail_streak = 0;
                    s_gray_w = dw;
                    /* decode sanity (1 Hz): proves the decoded buffer carries
                     * real pixels — distinguishes black decode vs black scene */
                    static uint32_t s_diag_frames = 0;
                    if ((s_diag_frames++ % 16) == 0) {
                        uint32_t row0_max = 0;
                        uint32_t mid_max = 0, mid_min = 255;
                        for (uint32_t x = 0; x < dw; x++) {
                            if (s_gray[x] > row0_max) row0_max = s_gray[x];
                            uint8_t m = s_gray[(size_t)(dh / 2) * dw + x];
                            if (m > mid_max) mid_max = m;
                            if (m < mid_min) mid_min = m;
                        }
                        ESP_LOGI(TAG, "decoded %ux%u: row0_max=%u mid=%u..%u (jlen=%u)",
                                 dw, dh, (unsigned)row0_max,
                                 (unsigned)mid_min, (unsigned)mid_max,
                                 (unsigned)jlen);
                    }
                    if (!s_first_dumped) {
                        s_first_dumped = true;
                        ESP_LOGI(TAG, "first good frame: %u bytes", (unsigned)jlen);
                        ESP_LOGI(TAG, "good head: %02x %02x %02x %02x %02x %02x %02x "
                                 "%02x %02x %02x %02x %02x",
                                 fr->data[0], fr->data[1], fr->data[2], fr->data[3],
                                 fr->data[4], fr->data[5], fr->data[6], fr->data[7],
                                 fr->data[8], fr->data[9], fr->data[10], fr->data[11]);
                        ESP_LOGI(TAG, "good tail: %02x %02x %02x %02x %02x %02x %02x "
                                 "%02x %02x %02x %02x %02x",
                                 fr->data[jlen - 12], fr->data[jlen - 11],
                                 fr->data[jlen - 10], fr->data[jlen - 9],
                                 fr->data[jlen - 8], fr->data[jlen - 7],
                                 fr->data[jlen - 6], fr->data[jlen - 5],
                                 fr->data[jlen - 4], fr->data[jlen - 3],
                                 fr->data[jlen - 2], fr->data[jlen - 1]);
                    }
                    /* keep the raw frame for the preview endpoint */
                    xSemaphoreTake(s_frame_lock, portMAX_DELAY);
                    if (!s_raw || s_raw_len < jlen) {
                        heap_caps_free(s_raw);
                        s_raw = heap_caps_malloc(jlen + 1024,
                                                 MALLOC_CAP_SPIRAM);
                    }
                    if (s_raw) {
                        memcpy(s_raw, fr->data, jlen);
                        s_raw_len = jlen;
                    }
                    xSemaphoreGive(s_frame_lock);
                    s_frame_count++;
                } else {
                    /* YUY2: Y plane = every other byte (bounds pre-checked) */
                    uint8_t *y = s_gray;
                    for (uint32_t yy = 0; yy < fh; yy++) {
                        for (uint32_t xx = 0; xx < fw; xx++) {
                            y[yy * fw + xx] = fr->data[(yy * fw + xx) * 2];
                        }
                    }
                    s_frame_count++;
                    s_gray_w = fw;
                    /* first frame after open: raw bytes head/tail (header
                     * offset? padding? metadata?) — where does content end */
                    if (!s_first_dumped) {
                        s_first_dumped = true;
                        size_t dl = fr->data_len;
                        if (dl >= 16)
                            ESP_LOGI(TAG, "yuy2 first: bytes=%u head="
                                     "%02x %02x %02x %02x %02x %02x %02x %02x"
                                     " tail=%02x %02x %02x %02x %02x %02x %02x %02x",
                                     (unsigned)dl,
                                     fr->data[0], fr->data[1], fr->data[2],
                                     fr->data[3], fr->data[4], fr->data[5],
                                     fr->data[6], fr->data[7],
                                     fr->data[dl - 8], fr->data[dl - 7],
                                     fr->data[dl - 6], fr->data[dl - 5],
                                     fr->data[dl - 4], fr->data[dl - 3],
                                     fr->data[dl - 2], fr->data[dl - 1]);
                    }
                    /* frame-content diagnostic: uniform 16s = black/lost sync;
                     * band MAXima + MEANs show WHERE content lives: a white
                     * footer (bottom band) poisons brightest-band search */
                    if (s_diag_bands < 8) {
                        s_diag_bands++;
                        int mn = 255, mx = 0, bands_max[10] = {0};
                        int bands_n[10] = {0};
                        for (uint32_t yy = 0; yy < fh; yy++) {
                            int bmax = 0;
                            for (uint32_t xx = 0; xx < fw; xx++) {
                                int v = y[yy * fw + xx];
                                if (v < mn) mn = v;
                                if (v > mx) mx = v;
                                if (v > bmax) bmax = v;
                            }
                            int bband = (int)(yy * 10 / fh);
                            if (bband < 10) {
                                if (bmax > bands_max[bband]) bands_max[bband] = bmax;
                                bands_n[bband]++;
                            }
                        }
                        /* second pass for per-band row means (cheap enough) */
                        uint32_t band_row_sum[10] = {0};
                        for (uint32_t yy = 0; yy < fh; yy++) {
                            uint32_t rs = 0;
                            for (uint32_t xx = 0; xx < fw; xx++)
                                rs += y[yy * fw + xx];
                            band_row_sum[yy * 10 / fh] += rs;
                        }
                        ESP_LOGI(TAG, "yuy2 f%d: full %d..%d maxb=%d/%d/%d/%d/%d/%d/%d/%d/%d/%d"
                                 " meanb=%d/%d/%d/%d/%d/%d/%d/%d/%d/%d bytes=%u",
                                 s_diag_bands, mn, mx,
                                 bands_max[0], bands_max[1], bands_max[2],
                                 bands_max[3], bands_max[4], bands_max[5],
                                 bands_max[6], bands_max[7], bands_max[8],
                                 bands_max[9],
                                 (int)(band_row_sum[0] / bands_n[0] / fw),
                                 (int)(band_row_sum[1] / bands_n[1] / fw),
                                 (int)(band_row_sum[2] / bands_n[2] / fw),
                                 (int)(band_row_sum[3] / bands_n[3] / fw),
                                 (int)(band_row_sum[4] / bands_n[4] / fw),
                                 (int)(band_row_sum[5] / bands_n[5] / fw),
                                 (int)(band_row_sum[6] / bands_n[6] / fw),
                                 (int)(band_row_sum[7] / bands_n[7] / fw),
                                 (int)(band_row_sum[8] / bands_n[8] / fw),
                                 (int)(band_row_sum[9] / bands_n[9] / fw),
                                 (unsigned)fr->data_len);
                    }
                }
                uvc_host_frame_return(st, fr);
                /* row window -> DSP: average the BRIGHTEST 21-row band.
                 * The spectral line is the brightest thing in the frame;
                 * its vertical position shifts with sensor crop between
                 * modes (800x600 vs 320x240 don't share row geometry), so
                 * a fixed middle window misses it. Sliding-window row sums
                 * find it every frame; averaging kills per-row noise. */
                if (s_midrow) {
                    const uint32_t half = 10;
                    uint32_t y0 = (dh / 2 > half) ? dh / 2 - half : 0;
                    if (dh > 2 * half + 4) {
                        /* row sums once, then O(1) sliding window */
                        uint32_t rsum_y0 = 0, wsum = 0;
                        for (uint32_t y = 0; y < 2 * half + 1; y++) {
                            uint32_t rs = 0;
                            const uint8_t *row = s_gray + (size_t)y * dw;
                            for (uint32_t x = 0; x < dw; x++) rs += row[x];
                            wsum += rs;
                        }
                        uint32_t best_sum = wsum;
                        for (uint32_t ys = 1; ys + 2 * half < dh; ys++) {
                            uint32_t rs_new = 0, rs_old = 0;
                            const uint8_t *row_in = s_gray + (size_t)(ys + 2 * half) * dw;
                            const uint8_t *row_out = s_gray + (size_t)(ys - 1) * dw;
                            for (uint32_t x = 0; x < dw; x++) {
                                rs_new += row_in[x];
                                rs_old += row_out[x];
                            }
                            wsum += rs_new;
                            wsum -= rs_old;
                            if (wsum > best_sum) {
                                best_sum = wsum;
                                rsum_y0 = ys;
                            }
                        }
                        y0 = rsum_y0;
                    }
                    uint32_t y1 = (y0 + 2 * half < dh) ? y0 + 2 * half : dh - 1;
                    for (uint32_t x = 0; x < dw; x++) {
                        uint32_t acc = 0;
                        for (uint32_t y = y0; y <= y1; y++)
                            acc += s_gray[(size_t)y * dw + x];
                        s_midrow[x] = (uint8_t)(acc / (y1 - y0 + 1));
                    }
                    spectra_process_frame(s_midrow, dw);
                    /* line tracker + sampled row-sum profile: the window
                     * search feeds spectra from ONE 21-row band — these are
                     * the facts about where that band landed and what the
                     * rows it skipped contain */
                    if (s_diag_win < 6) {
                        s_diag_win++;
                        uint8_t mmax = 0;
                        for (uint32_t x = 0; x < dw; x++)
                            if (s_midrow[x] > mmax) mmax = s_midrow[x];
                        ESP_LOGI(TAG, "row band: y0=%u..%u/%u max=%u (dw=%u)",
                                 (unsigned)y0, (unsigned)y1, (unsigned)dh, mmax,
                                 (unsigned)dw);
                    }
                    if (s_diag_rsum < 2) {
                        s_diag_rsum++;
                        uint32_t s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0;
                        uint32_t s6 = 0, s7 = 0, s8 = 0, s9 = 0, s10 = 0, s11 = 0;
                        for (uint32_t y = 0; y < dh; y++) {
                            uint32_t rs = 0;
                            for (uint32_t x = 0; x < dw; x++)
                                rs += s_gray[(size_t)y * dw + x];
                            if (y == 0) s0 = rs;
                            else if (y == 20) s1 = rs;
                            else if (y == 40) s2 = rs;
                            else if (y == 60) s3 = rs;
                            else if (y == 80) s4 = rs;
                            else if (y == 100) s5 = rs;
                            else if (y == 120) s6 = rs;
                            else if (y == 140) s7 = rs;
                            else if (y == 160) s8 = rs;
                            else if (y == 180) s9 = rs;
                            else if (y == 200) s10 = rs;
                            else if (y == 220) s11 = rs;
                        }
                        ESP_LOGI(TAG, "rsum r0/20/../220: %u %u %u %u %u %u %u %u %u %u %u %u",
                                 s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11);
                    }
                }
            } else if (got_first) {
                /* no frame for 5 s: treat as stall */
                if (xTaskGetTickCount() - last > pdMS_TO_TICKS(5000)) {
                    ESP_LOGW(TAG, "stream stalled (5s) -> restart");
                    break;
                }
                /* heartbeat so a hang inside the loop is visible in the log */
                static TickType_t idle_log = 0;
                if (xTaskGetTickCount() - idle_log > pdMS_TO_TICKS(2000)) {
                    idle_log = xTaskGetTickCount();
                    ESP_LOGI(TAG, "cam alive: waiting for frames (frames=%u drops=%u)",
                             (unsigned)s_frame_count, (unsigned)s_drop_count);
                }
            } else {
                /* still no first frame after 12 s -> restart cycle */
                if (xTaskGetTickCount() - last > pdMS_TO_TICKS(12000)) {
                    ESP_LOGW(TAG, "no frames after 12s -> retry");
                    break;
                }
            }
        }
        /* stream stopped: clean up and retry */
        if (s_stream) {
            /* frames queued by frame_cb transfer ownership to us; the driver
             * refuses uvc_host_stream_close while any are outstanding — and an
             * ignored close failure leaves the interface claimed forever
             * (every re-open then fails with ESP_ERR_INVALID_STATE) */
            uvc_host_frame_t *frq;
            while (xQueueReceive(s_frame_q, &frq, 0) == pdPASS)
                uvc_host_frame_return(s_stream, frq);
            uvc_host_stream_stop(s_stream);
            for (int i = 0; i < 5; i++) {
                if (uvc_host_stream_close(s_stream) == ESP_OK) break;
                while (xQueueReceive(s_frame_q, &frq, 0) == pdPASS)
                    uvc_host_frame_return(s_stream, frq);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            s_stream = NULL;
        }
        s_streaming = false;
        /* camera still connected: retry promptly with the next advertised
         * format; camera gone: back to waiting for a connect event.
         * A raw-toggle reopen rides through here too — it must reach the
         * picker, not the ladder (the ladder walked MJPEG garbage for ~8 s
         * after every toggle before it lucked into YUY2). */
        if (picker_wins) {
            start_idx = 0;
            picker_wins = false;
        } else {
            start_idx = (idx + 1) % (int)(s_frame_list_n ? s_frame_list_n : 1);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t camera_start(void)
{
    s_frame_lock = xSemaphoreCreateMutex();
    cam_prefs_load();
    s_fmt_signal = xSemaphoreCreateBinary();
    s_frame_q = xQueueCreate(5, sizeof(uvc_host_frame_t *));

    /* usb_lib event pump — REQUIRED for enumeration */
    BaseType_t ok = xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096,
                                            NULL, USB_LIB_TASK_PRIO, NULL,
                                            tskNO_AFFINITY);
    if (ok != pdTRUE) return ESP_FAIL;

    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));

    /* cam on core 1: JPEG decode + spectra may not starve IDLE0 — the USB
     * callbacks/wire handling also run there and IDLE0 is watchdog-watched
     * (wdt trips read "CPU 0: cam" when this task wanders onto core 0). */
    ok = xTaskCreatePinnedToCore(camera_task, "cam", 12288, NULL, CAM_TASK_PRIO,
                                 NULL, 1);
    if (ok != pdTRUE) return ESP_FAIL;
    return ESP_OK;
}

bool camera_get_frame(const uint8_t **data, size_t *len)
{
    xSemaphoreTake(s_frame_lock, portMAX_DELAY);
    bool ok = s_raw && s_raw_len > 0 && s_is_mjpeg;
    if (ok) {
        *data = s_raw;
        *len = s_raw_len;
    }
    xSemaphoreGive(s_frame_lock);
    return ok;
}

void camera_get_status(bool *mjpeg, uint32_t *w, uint32_t *h, int *fps,
                       int *format_idx, uint32_t *frames, bool *streaming)
{
    *mjpeg = s_is_mjpeg;
    *w = s_w;
    *h = s_h;
    *fps = (int)(s_fps_req + 0.5f);
    *format_idx = s_fmt_fmt;
    *frames = s_frame_count;
    *streaming = s_streaming;
}

size_t camera_row_len(void)
{
    /* width of the row actually fed to spectra (1/2-scale decode for big cams) */
    return s_gray_w ? s_gray_w : s_w;
}
