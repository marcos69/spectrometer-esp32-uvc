/* Spectrum graph PNG renderer (snapshot feature).
 *
 * - RGBA8 PNG with STORED deflate blocks (no zlib dependency; snapshot is
 *   ~570 KB, fine for an on-demand image over Wi-Fi)
 * - FIXED 1250nm..22nm ruler left->right — mirrors the web page (NM0/NM1):
 *   the graticule never moves with calibration; peaks get calibrated labels
 * - 5x7 bitmap font for graticule/peak labels
 * - Black background, false-color spectrum (palette via spectra_nm_to_rgb)
 */
#include <stdint.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "render.h"
#include "spectra.h"

#define TAG "render"

/* keep in sync with web_index.html NM0/NM1 (fixed nm ruler) */
#define RENDER_NM0 1250.0f
#define RENDER_NM1 22.0f

/* ---- PNG: zlib wrapper + stored deflate blocks -------------------------- */
static uint32_t crc_table[256];
static bool s_crc_init = false;

static void crc_init(void)
{
    if (s_crc_init) return;
    for (uint32_t n = 0; n < 256; n++) {
        uint32_t c = n;
        for (int k = 0; k < 8; k++) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
        crc_table[n] = c;
    }
    s_crc_init = true;
}

static uint32_t crc32_buf(const uint8_t *d, size_t n)
{
    crc_init();
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) c = crc_table[(c ^ d[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

static uint32_t adler32(const uint8_t *d, size_t n)
{
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < n; i++) {
        a = (a + d[i]) % 65521;
        b = (b + a) % 65521;
    }
    return (b << 16) | a;
}

static void be16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v & 0xFF; }
static void be32(uint8_t *p, uint32_t v)
{
    p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v & 0xFF;
}

/* Stored deflate: per RFC 1951 a stored block is BFINAL|BTYPE(00) padded to
 * a byte boundary, LEN (little-endian), NLEN (one's complement of LEN) —
 * omitting NLEN makes every decoder reject the stream with
 * "invalid stored block lengths". */
static size_t deflate_store(const uint8_t *in, size_t inlen, uint8_t *out)
{
    size_t o = 0;
    out[o++] = 0x78; /* CMF: deflate, 32K window */
    out[o++] = 0x01; /* FLG: (CMF*256+FLG) % 31 == 0 */
    size_t done = 0;
    while (done < inlen) {
        bool last = (done + 65535 >= inlen);
        uint16_t chunk = last ? (uint16_t)(inlen - done) : 65535;
        uint16_t nlen = (uint16_t)~chunk;
        out[o++] = last ? 0x01 : 0x00; /* BFINAL + BTYPE(00) + padding bits */
        out[o++] = chunk & 0xFF;       /* LEN, little-endian */
        out[o++] = (chunk >> 8) & 0xFF;
        out[o++] = nlen & 0xFF;        /* NLEN, little-endian */
        out[o++] = (nlen >> 8) & 0xFF;
        memcpy(out + o, in + done, chunk);
        o += chunk;
        done += chunk;
    }
    be32(out + o, adler32(in, inlen));
    o += 4;
    return o;
}

static size_t png_chunk(uint8_t *out, const char *type, const void *data, size_t dlen)
{
    size_t o = 0;
    be32(out + o, (uint32_t)dlen);
    o += 4;
    memcpy(out + o, type, 4);
    o += 4;
    if (dlen) { memcpy(out + o, data, dlen); o += dlen; }
    be32(out + o, crc32_buf(out + 4, 4 + dlen));
    o += 4;
    return o;
}

/* ---- tiny 5x7 font ------------------------------------------------------ */
static const uint8_t FONT[][5] = {
    {0x7C,0x86,0x9A,0xA5,0x7C}, /* 0 */
    {0x18,0x38,0x18,0x18,0x7C}, /* 1 */
    {0x7C,0x86,0x60,0x30,0xFC}, /* 2 */
    {0xFC,0x30,0x30,0x30,0xFC}, /* 3 */
    {0x36,0x4A,0xFE,0x0C,0x0C}, /* 4 */
    {0xFE,0x90,0xF8,0x12,0xEE}, /* 5 */
    {0x3C,0x60,0xF8,0x92,0x7C}, /* 6 */
    {0xFC,0x86,0x0A,0x12,0x12}, /* 7 */
    {0x7C,0x92,0x7C,0x92,0x7C}, /* 8 */
    {0x7C,0x92,0x7C,0x0E,0x78}, /* 9 */
    {0x00,0x66,0x99,0x99,0x99}, /* n */
    {0x00,0x66,0xFF,0xFF,0x66}, /* m */
    {0x00,0x00,0x00,0x00,0x18}, /* . */
    {0x00,0x00,0x78,0x00,0x00}, /* - */
};
#define FONT_N 14
static const char *font_chars = "0123456789nm.-";

/* unsigned decimal, most-significant first; returns digit count */
static int utoa_dec(int v, char *out)
{
    char tmp[8];
    int t = 0;
    if (v == 0) tmp[t++] = '0';
    while (v > 0) { tmp[t++] = (char)('0' + v % 10); v /= 10; }
    for (int k = 0; k < t; k++) out[k] = tmp[t - 1 - k];
    out[t] = 0;
    return t;
}

static void draw_char(uint32_t *px, int W, int H, int x, int y, const char *s,
                      int scale, uint32_t fg)
{
    for (; *s; s++) {
        int idx = (int)(s[0] - *font_chars);
        if (idx < 0 || idx >= FONT_N) continue;
        for (int row = 0; row < 5; row++) {
            for (int col = 0; col < 5; col++) {
                if (!(FONT[idx][row] & (0x10 >> col))) continue;
                for (int sy = 0; sy < scale; sy++) {
                    for (int sx = 0; sx < scale; sx++) {
                        int xx = x + col * scale + sx;
                        int yy = y + row * scale + sy;
                        if (xx >= 0 && xx < W && yy >= 0 && yy < H)
                            px[yy * W + xx] = fg;
                    }
                }
            }
        }
        x += 6 * scale;
    }
}

static void draw_hline(uint32_t *px, int W, int H, int y, uint32_t c)
{
    if (y < 0 || y >= H) return;
    for (int x = 0; x < W; x++) px[y * W + x] = c;
}

static void draw_vline(uint32_t *px, int W, int H, int x, uint32_t c)
{
    if (x < 0 || x >= W) return;
    for (int y = 0; y < H; y++) px[y * W + x] = c;
}

void render_png_spectrum(uint8_t *out, size_t out_cap, size_t *out_len,
                         const float *intensity, int n,
                         const int *peak_px, const float *peak_val, int n_peaks,
                         const int cal[4])
{
    *out_len = 0;
    const int W = 640, H = 220;
    const int bottom = H - 24; /* x-axis area */
    const int top = 8;
    const float nm_span = RENDER_NM1 - RENDER_NM0; /* negative: 1250 -> 22 */

    /* PSRAM scratch (cached): framebuffer + raw scanlines + IDAT */
    static uint32_t *px = NULL;
    static uint8_t *raw = NULL;
    static uint8_t *idat = NULL;
    if (!px || !raw || !idat) {
        px   = heap_caps_malloc((size_t)W * H * 4, MALLOC_CAP_SPIRAM);
        raw  = heap_caps_malloc((size_t)H * (W * 4 + 1), MALLOC_CAP_SPIRAM);
        idat = heap_caps_malloc((size_t)H * (W * 4 + 1) + 128, MALLOC_CAP_SPIRAM);
        if (!px || !raw || !idat) {
            ESP_LOGE(TAG, "PNG scratch alloc failed");
            return;
        }
        ESP_LOGI(TAG, "PNG scratch in PSRAM (3x ~564 KB)");
    }

    for (int i = 0; i < W * H; i++) px[i] = 0; /* black bg */

    /* fixed-ruler x mapping (snapshot = the zoom-1 full-scale view) */
    float px_per_data = (n > 1) ? (float)W / n : 1.0f;

    /* graticule: line every 50nm, bold + label every 100nm */
    for (int nm = 250; nm < 1250; nm += 50) {
        int x = (int)((nm - RENDER_NM0) / nm_span * W + 0.5f);
        if (x < 0 || x >= W) continue;
        bool major = (nm % 100 == 0);
        draw_vline(px, W, H, x, major ? 0x00606060u : 0x00282828u);
        if (major) {
            char label[8];
            int l = utoa_dec(nm, label);
            draw_char(px, W, H, x - 6 * l + 3, H - 18, label, 2, 0x00B0B0B0u);
        }
    }
    draw_char(px, W, H, W - 36, H - 18, "nm", 2, 0x00808080u);
    draw_hline(px, W, H, bottom, 0x00808080u);

    /* spectrum: false-color strips, colors clamp to the visible band
     * exactly like the web page (out-of-band data still drawn) */
    if (n > 0) {
        float maxv = 1.0f;
        for (int i = 0; i < n; i++) if (intensity[i] > maxv) maxv = intensity[i];
        int bw = (int)(px_per_data + 0.5f);
        if (bw < 1) bw = 1;
        for (int i = 0; i < n; i++) {
            int x = (int)(px_per_data * i);
            if (x >= W) break;
            float nm = RENDER_NM0 + nm_span * i / (float)n;
            float cnm = nm < 380.0f ? 380.0f : (nm > 780.0f ? 780.0f : nm);
            uint8_t r, g, b;
            spectra_nm_to_rgb(cnm, &r, &g, &b);
            int v = (int)(intensity[i] * 255.0f / maxv);
            if (v < 0) v = 0;
            if (v > 255) v = 255;
            int y0 = bottom - v * (bottom - top) / 255;
            uint32_t c = (0xFFu << 24) | ((uint32_t)b << 16) |
                         ((uint32_t)g << 8) | r;
            for (int y = y0; y < bottom; y++)
                for (int xx = x; xx < x + bw && xx < W; xx++)
                    px[y * W + xx] = c;
        }
    }

    /* peaks: marker line + label — calibrated nm when a cal exists, the
     * fixed-ruler value otherwise (mirrors the web page) */
    bool has_cal = cal[0] && cal[2] && cal[2] != cal[0];
    float cal_slope = has_cal ? ((float)cal[3] - cal[1]) / (cal[2] - cal[0]) : 0.0f;
    for (int i = 0; i < n_peaks && i < 24; i++) {
        int ipx = peak_px[i];
        if (ipx < 0) continue;
        int x = (int)(px_per_data * ipx);
        if (x < 0 || x >= W) continue;
        draw_vline(px, W, H, x, 0x00C8FF40u);
        float nm = has_cal
            ? (float)cal[1] + (ipx - cal[0]) * cal_slope
            : RENDER_NM0 + nm_span * ipx / (float)n;
        char label[12];
        int l = utoa_dec((int)(nm + 0.5f), label);
        label[l++] = 'n'; label[l++] = 'm'; label[l] = 0;
        draw_char(px, W, H, x + 4, top, label, 1, 0x00C8FF40u);
    }

    /* scanlines (filter type 0); px packs A|B|G|R, PNG wants R,G,B,A */
    size_t rowbytes = (size_t)W * 4 + 1;
    for (int y = 0; y < H; y++) {
        raw[y * rowbytes] = 0;
        const uint32_t *row = &px[y * W];
        uint8_t *o = &raw[y * rowbytes + 1];
        for (int x = 0; x < W; x++) {
            o[0] = row[x] & 0xFF;         /* R */
            o[1] = (row[x] >> 8) & 0xFF;  /* G */
            o[2] = (row[x] >> 16) & 0xFF; /* B */
            o[3] = 0xFF;
            o += 4;
        }
    }
    size_t rawlen = (size_t)H * rowbytes;
    size_t idatlen = deflate_store(raw, rawlen, idat);

    uint8_t ihdr[13];
    be32(ihdr + 0, (uint32_t)W);
    be32(ihdr + 4, (uint32_t)H);
    ihdr[8] = 8;  /* bit depth */
    ihdr[9] = 6;  /* color type RGBA */
    ihdr[10] = ihdr[11] = ihdr[12] = 0;

    size_t need = 8 + 25 + (8 + idatlen) + 12;
    if (out_cap < need) {
        ESP_LOGE(TAG, "PNG out buffer too small: %u < %u",
                 (unsigned)out_cap, (unsigned)need);
        return;
    }
    uint8_t *p = out;
    const uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    memcpy(p, sig, 8); p += 8;
    p += png_chunk(p, "IHDR", ihdr, 13);
    p += png_chunk(p, "IDAT", idat, idatlen);
    p += png_chunk(p, "IEND", NULL, 0);
    *out_len = (size_t)(p - out);
}
