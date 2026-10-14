#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPECTRA_MAX_ROWS   1024   /* max supported frame width (px) */
#define SPECTRA_MAX_PEAKS  32     /* hard cap on labeled peaks */

typedef struct {
    int px1, w1;       /* calibration point 1: pixel -> nm */
    int px2, w2;       /* calibration point 2 */
} spectra_cal_t;

/* Default calibration = PySpectrometer v3.2 DEFAULT_CALIBRATION
 * (Pi-camera geometry; recalibrate with your camera via the web UI). */
void spectra_cal_default(spectra_cal_t *out);

void spectra_init(void);
/* Read latest state. Caller holds nothing; returns copies. */
/* Copy latest intensity array (smoothed, post-hold) into out_arr (>= SPECTRA_MAX_ROWS).
 * Returns sample count. */
size_t spectra_get_intensity(float *out_arr);
size_t spectra_row_len(void);
int    spectra_peaks(int *pos_arr, float *val_arr, int max_peaks);
void   spectra_cal_get(spectra_cal_t *out);
bool   spectra_cal_set(const spectra_cal_t *cal);  /* + persists to NVS */
void   spectra_set_min_dist(int v);   /* peak width / min distance, px */
void   spectra_set_thresh(int v);     /* threshold, percent 0..100 */
void   spectra_set_filter(int v);     /* savgol order 0..15 */
void   spectra_set_gain(int v);       /* display gain 1..50 (UI-only, not DSP) */
void   spectra_mark_settings_dirty(void);  /* persist md/th/order/gain after 2 s quiet */
void   spectra_get_settings(int *mindist, int *thresh, int *order, int *gain);
void   spectra_set_hold(bool on);
bool   spectra_hold(void);
void   spectra_reset_hold(void);

/* Per-frame pipeline entry point (camera task): smooth + peak detect.
 * gray_row is grayscale luminance of the mid-frame row, n samples. */
void   spectra_process_frame(const uint8_t *gray_row, size_t n);

/* Wavelength (nm) at pixel index px, using current calibration. */
float  spectra_px_to_nm(float px);

/* Port of PySpectrometer wavelength_to_rgb (380..780 nm false color). */
void   spectra_nm_to_rgb(float nm, uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus
}
#endif
