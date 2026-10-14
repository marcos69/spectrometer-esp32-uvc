#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start USB host + UVC stack and the frame-handling task.
 * Tries the format chain until the camera streams:
 *   MJPEG 640x480 -> MJPEG 320x240 -> YUY2 320x240
 * Safe to call once from app_main. */
esp_err_t camera_start(void);

/* Latest raw frame for the /preview endpoint.
 * Returns raw MJPEG bytes (served as image/jpeg), or NULL in YUY2 mode. */
bool camera_get_frame(const uint8_t **data, size_t *len);

/* Camera status for the UI */
void camera_get_status(bool *mjpeg, uint32_t *w, uint32_t *h, int *fps,
                       int *format_idx, uint32_t *frames, bool *streaming);

/* Raw YUY2 mode: prefer the camera's uncompressed format when it advertises
 * one (no JPEG quantization, much lower fps — accepted at boot). NVS-backed. */
void camera_set_raw(bool on);
bool camera_get_raw(void);

/* Manual exposure, microseconds (0 = camera auto). Applied via UVC control
 * requests after each stream open; camera keeps auto if it refuses. */
void camera_set_exposure_us(int us);
int camera_get_exposure_us(void);

/* Mid-row gray sample count (frame width in pixels) */
size_t camera_row_len(void);

#ifdef __cplusplus
}
#endif
