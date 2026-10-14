#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Render the spectrum graph to a PNG (RGBA8, 640x220).
 * out must be >= ~576 KB (snapshot PNG is mostly IDAT).
 * Returns void; *out_len is set on success (out_len==0 on failure). */
void render_png_spectrum(uint8_t *out, size_t out_cap, size_t *out_len,
                         const float *intensity, int n,
                         const int *peak_px, const float *peak_val, int n_peaks,
                         const int cal[4]);

#ifdef __cplusplus
}
#endif
