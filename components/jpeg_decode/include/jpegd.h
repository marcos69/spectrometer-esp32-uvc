#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Thin gray-decode wrapper over the vendored libjpeg (jpeg-9a port from
 * esp32-camera feat/add_libjpeg, Apache-2.0).
 *
 * For 3-component YCbCr JPEGs, outputting JCS_GRAYSCALE yields the Y
 * (luminance) plane exactly — no chroma conversion, which is what the
 * spectrometer needs (PySpectrometer also works on grayscale).
 */

/** Parse headers only (no decode). Fills w/h. 0 = ok, -1 = not a JPEG. */
int jpegd_scan_info(const uint8_t *src, size_t len, uint32_t *w, uint32_t *h);

/** Decode to 8-bit grayscale (Y plane for color images).
 * out must hold at least w*h bytes (see jpegd_scan_info). 0 = ok. */
int jpegd_decode_gray(const uint8_t *src, size_t len, uint8_t *out,
                      uint32_t w, uint32_t h);

/** Decode to 8-bit grayscale with automatic 1/2 DCT scaling for large
 * images (>= ~400k px): output = w/2 x h/2, much cheaper decode on the
 * S3 (mid-row spectrometer use case). Receives the actual output size
 * via out_w / out_h. 0 = ok. */
int jpegd_decode_gray_auto(const uint8_t *src, size_t len, uint8_t *out,
                           size_t out_cap, uint32_t *out_w, uint32_t *out_h);

#ifdef __cplusplus
}
#endif
