/* Spectra DSP core — C port of PySpectrometer v3.2 signal processing.
 *
 * Pipeline (per frame, done by the camera task via spectra_process_frame):
 *   mid-row grayscale -> savgol (window 17, order 0..15) -> intensity[]
 *   peak detection    -> peaks[] (peakutils port)
 *   calibration       -> px <-> nm linear (2-point)
 *
 * The web layer (render.c / web.c) turns this into canvas draw commands,
 * CSV and PNG snapshots. All shared state is guarded by s_lock.
 */
#include "spectra.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "spectra"
extern const float sg_coeff[16][17];   /* tools/gen_savgol.py, window 17 */
static void save_settings_cb(void *arg);
static esp_timer_handle_t s_save_timer;

static SemaphoreHandle_t s_lock;

static float s_int[SPECTRA_MAX_ROWS];
static size_t s_row_len = 0;
static float s_hold[SPECTRA_MAX_ROWS]; /* peak-hold memory */
static int   s_peak_pos[SPECTRA_MAX_PEAKS];
static float s_peak_val[SPECTRA_MAX_PEAKS];
static int   s_peak_n = 0;

static spectra_cal_t s_cal;
static int s_mindist = 50;
static int s_thresh = 20;
static int s_order = 7;
static int s_gain = 6;    /* UI display gain; 1.0 data ≈ x6 on the 280px graph */
static bool s_hold_on = false;

static const char *NVS_NS = "spectra";
#define NVS_KEY_CAL "cal"
#define NVS_KEY_SET "set"

void spectra_cal_default(spectra_cal_t *out)
{
    /* ((72, 405), (304, 532)) — from PySpectrometer DEFAULT_CALIBRATION */
    out->px1 = 72;  out->w1 = 405;
    out->px2 = 304; out->w2 = 532;
}

void spectra_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    spectra_cal_default(&s_cal);
    const esp_timer_create_args_t targs = {
        .callback = save_settings_cb,
        .name = "set_save",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_save_timer));
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t buf[16];
        size_t len = sizeof(buf);
        if (nvs_get_blob(h, NVS_KEY_CAL, buf, &len) == ESP_OK && len == sizeof(s_cal)) {
            memcpy(&s_cal, buf, sizeof(s_cal));
            ESP_LOGI(TAG, "loaded calibration px1=%d w1=%d px2=%d w2=%d",
                     s_cal.px1, s_cal.w1, s_cal.px2, s_cal.w2);
        }
        size_t len2 = sizeof(buf);
        esp_err_t r = nvs_get_blob(h, NVS_KEY_SET, buf, &len2);
        if (r == ESP_OK && len2 == 16) {
            memcpy(&s_mindist, buf, 4);
            memcpy(&s_thresh, buf + 4, 4);
            memcpy(&s_order, buf + 8, 4);
            memcpy(&s_gain, buf + 12, 4);
            ESP_LOGI(TAG, "loaded settings mindist=%d thresh=%d order=%d gain=%d",
                     s_mindist, s_thresh, s_order, s_gain);
        } else if (r == ESP_OK && len2 == 12) {
            /* legacy layout without gain */
            memcpy(&s_mindist, buf, 4);
            memcpy(&s_thresh, buf + 4, 4);
            memcpy(&s_order, buf + 8, 4);
            ESP_LOGI(TAG, "loaded settings (legacy) mindist=%d thresh=%d order=%d",
                     s_mindist, s_thresh, s_order);
        }
        nvs_close(h);
    }
}

/* ---- savgol: yhat[i] = sum_t m[t] * y_ext[i + 16 - t] ------------------ */
static void savgol(const uint8_t *y, size_t n, float *out, int order)
{
    const float *m = sg_coeff[order < 0 ? 0 : (order > 15 ? 15 : order)];
    for (size_t i = 0; i < n; i++) {
        float acc = 0.0f;
        for (int t = 0; t < 17; t++) {
            int idx = (int)i + 16 - t;       /* y_ext index */
            if (idx < 0) {
                /* left self-reflective pad: y[0]-|y[k]-y[0]| for k=1..8 */
                int k = 1 + (-idx);
                acc += m[t] * ((float)y[0] - fabsf((float)y[k] - (float)y[0]));
            } else if (idx >= (int)n) {
                int k = idx - (int)n;
                acc += m[t] * ((float)y[n - 1] + fabsf((float)y[n - 1 - k] - (float)y[n - 1]));
            } else {
                acc += m[t] * (float)y[idx];
            }
        }
        out[i] = acc;
    }
}

/* ---- peakutils.peakIndexes port ---------------------------------------- */
static void peak_detect(const float *y, size_t n, int *pos, float *val, int *n_peaks)
{
    if (n < 3) { *n_peaks = 0; return; }
    float mn = y[0], mx = y[0];
    for (size_t i = 1; i < n; i++) { if (y[i] < mn) mn = y[i]; if (y[i] > mx) mx = y[i]; }
    float thres = (s_thresh / 100.0f) * (mx - mn) + mn;
    int md = s_mindist; if (md < 1) md = 1;

    float *dy = malloc(sizeof(float) * (n - 1));
    if (!dy) { *n_peaks = 0; return; }
    for (size_t i = 0; i < n - 1; i++) dy[i] = y[i + 1] - y[i];

    /* plateau handling: propagate non-zero diffs across zero runs */
    for (size_t i = 0; i + 1 < n; i++) {
        if (dy[i] == 0.0f) {
            size_t start = i;
            while (i + 1 < n && dy[i + 1] == 0.0f) i++;
            size_t end = i;
            float left_val = (start > 0) ? dy[start - 1] : 1.0f;
            float right_val = (end + 1 < n - 1) ? dy[end + 1] : -1.0f;
            size_t split = end - (end - start) / 2;
            for (size_t j = start; j <= end; j++)
                dy[j] = (j < split) ? left_val : right_val;
        }
    }

    int peaks[SPECTRA_MAX_PEAKS];
    int pk = 0;
    for (size_t i = 0; i < n && pk < SPECTRA_MAX_PEAKS; i++) {
        float left = (i == 0) ? 0.0f : dy[i - 1];
        float right = (i == n - 1) ? 0.0f : dy[i];
        if (left > 0.0f && right < 0.0f && y[i] > thres) {
            peaks[pk++] = (int)i;
        }
    }
    /* min-distance pruning, highest peak first (peakutils behaviour) */
    int order_idx[SPECTRA_MAX_PEAKS];
    for (int i = 0; i < pk; i++) order_idx[i] = i;
    for (int a = 0; a < pk; a++)
        for (int b = a + 1; b < pk; b++)
            if (y[peaks[order_idx[b]]] > y[peaks[order_idx[a]]]) {
                int t = order_idx[a]; order_idx[a] = order_idx[b]; order_idx[b] = t;
            }
    bool keep[SPECTRA_MAX_PEAKS];
    memset(keep, 0, sizeof(keep));
    for (int a = 0; a < pk; a++) {
        int p = peaks[order_idx[a]];
        bool clash = false;
        for (int b = 0; b < pk; b++) {
            if (keep[b] && p != peaks[order_idx[b]] &&
                fabsf((float)(p - peaks[order_idx[b]])) <= md) { clash = true; break; }
        }
        if (!clash) keep[a] = true;
    }
    int out_n = 0;
    for (int a = 0; a < pk; a++) {
        if (keep[a]) {
            pos[out_n] = peaks[order_idx[a]];
            val[out_n] = y[peaks[order_idx[a]]];
            out_n++;
        }
    }
    /* sort by position for stable display */
    for (int a = 0; a < out_n; a++)
        for (int b = a + 1; b < out_n; b++)
            if (pos[b] < pos[a]) {
                int tp = pos[a]; pos[a] = pos[b]; pos[b] = tp;
                float tv = val[a]; val[a] = val[b]; val[b] = tv;
            }
    *n_peaks = out_n;
    free(dy);
}

/* ---- public state accessors --------------------------------------------- */

/* Copy latest intensity array (smoothed, post-hold). Returns sample count. */
size_t spectra_get_intensity(float *out_arr)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    size_t n = s_row_len > SPECTRA_MAX_ROWS ? SPECTRA_MAX_ROWS : s_row_len;
    if (n > 0) memcpy(out_arr, s_int, n * sizeof(float));
    xSemaphoreGive(s_lock);
    return n;
}

size_t spectra_row_len(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    size_t n = s_row_len;
    xSemaphoreGive(s_lock);
    return n;
}

int spectra_peaks(int *pos_arr, float *val_arr, int max_peaks)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = s_peak_n < max_peaks ? s_peak_n : max_peaks;
    if (n > 0) {
        memcpy(pos_arr, s_peak_pos, n * sizeof(int));
        memcpy(val_arr, s_peak_val, n * sizeof(float));
    }
    xSemaphoreGive(s_lock);
    return n;
}

void spectra_cal_get(spectra_cal_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_cal;
    xSemaphoreGive(s_lock);
}

bool spectra_cal_set(const spectra_cal_t *cal)
{
    /* all-zero cal = "no calibration" (UI reset button); keep it storable */
    bool zero = !cal->px1 && !cal->w1 && !cal->px2 && !cal->w2;
    if (!zero && (cal->px1 == cal->px2 || cal->w1 == cal->w2)) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cal = *cal;
    xSemaphoreGive(s_lock);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, NVS_KEY_CAL, &s_cal, sizeof(s_cal));
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "calibration set px1=%d w1=%d px2=%d w2=%d", cal->px1, cal->w1, cal->px2, cal->w2);
    return true;
}

void spectra_set_min_dist(int v) { if (v >= 1 && v <= 500) s_mindist = v; }
void spectra_set_thresh(int v)   { if (v >= 0 && v <= 100) s_thresh = v; }
void spectra_set_filter(int v)   { if (v >= 0 && v <= 15)  s_order = v; }
void spectra_set_gain(int v)     { if (v >= 1 && v <= 50)  s_gain = v; }

/* Debounced settings persistence: sliding a range fires many POSTs per
 * second; write NVS once 2 s after the last change instead of per change. */
static void save_settings_cb(void *arg)
{
    (void)arg;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    uint8_t buf[16];
    memcpy(buf, &s_mindist, 4);
    memcpy(buf + 4, &s_thresh, 4);
    memcpy(buf + 8, &s_order, 4);
    memcpy(buf + 12, &s_gain, 4);
    nvs_set_blob(h, NVS_KEY_SET, buf, sizeof(buf));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "settings saved md=%d th=%d or=%d g=%d", s_mindist, s_thresh, s_order, s_gain);
}

void spectra_mark_settings_dirty(void)
{
    if (!s_save_timer) return;
    esp_timer_stop(s_save_timer);
    esp_timer_start_once(s_save_timer, 2 * 1000000ULL);
}

void spectra_get_settings(int *mindist, int *thresh, int *order, int *gain)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *mindist = s_mindist; *thresh = s_thresh; *order = s_order; *gain = s_gain;
    xSemaphoreGive(s_lock);
}

void spectra_set_hold(bool on) { s_hold_on = on; }
bool  spectra_hold(void)       { return s_hold_on; }

void spectra_reset_hold(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memset(s_hold, 0, sizeof(s_hold));
    xSemaphoreGive(s_lock);
}

float spectra_px_to_nm(float px)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    float pxrange = (float)(s_cal.px2 - s_cal.px1);
    if (pxrange == 0.0f) pxrange = 1.0f;
    float nmrange = (float)(s_cal.w2 - s_cal.w1);
    float pxpernm = pxrange / nmrange;
    float zero = (float)s_cal.w1 - (float)s_cal.px1 / pxpernm;
    float nm = zero + px / pxpernm;
    xSemaphoreGive(s_lock);
    return nm;
}

/* PySpectrometer wavelength_to_rgb, gamma 0.8 */
void spectra_nm_to_rgb(float nm, uint8_t *r, uint8_t *g, uint8_t *b)
{
    const float gamma = 0.8f, max_i = 255.0f;
    float R = 0, G = 0, B = 0, factor = 0;
    if (nm < 380 || nm > 780) { *r = *g = *b = 0; return; }
    if (nm <= 439) { R = -(nm - 440) / 60.0f; G = 0; B = 1; }
    else if (nm <= 489) { R = 0; G = (nm - 440) / 50.0f; B = 1; }
    else if (nm <= 509) { R = 0; G = 1; B = -(nm - 510) / 20.0f; }
    else if (nm <= 579) { R = (nm - 510) / 70.0f; G = 1; B = 0; }
    else if (nm <= 644) { R = 1; G = -(nm - 645) / 65.0f; B = 0; }
    else { R = 1; G = 0; B = 0; }
    if (nm <= 419) factor = 0.3f + 0.7f * (nm - 380) / 40.0f;
    else if (nm <= 700) factor = 1.0f;
    else factor = 0.3f + 0.7f * (780 - nm) / 80.0f;
    *r = (uint8_t)(R > 0 ? max_i * powf(R * factor, gamma) : 0);
    *g = (uint8_t)(G > 0 ? max_i * powf(G * factor, gamma) : 0);
    *b = (uint8_t)(B > 0 ? max_i * powf(B * factor, gamma) : 0);
}

/* ---- per-frame pipeline (called by camera task) -------------------------- */
void spectra_process_frame(const uint8_t *gray_row, size_t n)
{
    if (n == 0) return;
    if (n > SPECTRA_MAX_ROWS) n = SPECTRA_MAX_ROWS;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_hold_on) {
        for (size_t i = 0; i < n; i++) {
            if (gray_row[i] > s_hold[i]) s_hold[i] = gray_row[i];
        }
        memcpy(s_int, s_hold, n * sizeof(float));
    } else if (s_order > 0) {
        savgol(gray_row, n, s_int, s_order);
    } else {
        for (size_t i = 0; i < n; i++) s_int[i] = gray_row[i];
    }
    s_row_len = n;

    /* peak detection on a stable copy */
    float copy[SPECTRA_MAX_ROWS];
    memcpy(copy, s_int, n * sizeof(float));
    int pos[SPECTRA_MAX_PEAKS];
    float val[SPECTRA_MAX_PEAKS];
    int pk = 0;
    peak_detect(copy, n, pos, val, &pk);
    s_peak_n = pk;
    memcpy(s_peak_pos, pos, pk * sizeof(int));
    memcpy(s_peak_val, val, pk * sizeof(float));
    xSemaphoreGive(s_lock);
}
