/* HTTP web server: UI page, data API, calibration, settings, live preview,
 * net credentials, and snapshots (CSV / PNG).
 */
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "web.h"
#include "web_index.h"
#include "camera.h"
#include "render.h"
#include "spectra.h"
#include "net.h"

#define TAG "web"

#define MAX_ROWS 1024
#define MAX_PEAKS 32

/* ------------------------------------------------------------------ */
static esp_err_t handle_index(httpd_req_t *req)
{
    /* the page changes with every flash — never let the browser reuse a
     * cached copy (stale-page symptoms look exactly like dead controls) */
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

/* GET /api/data — full state for the JS poller */
static esp_err_t handle_data(httpd_req_t *req)
{
    static char buf[7 * 1024];
    float intensity[MAX_ROWS];
    int n = (int)spectra_get_intensity(intensity);
    int peak_px[MAX_PEAKS], n_peaks;
    float peak_val[MAX_PEAKS];
    n_peaks = spectra_peaks(peak_px, peak_val, MAX_PEAKS);
    spectra_cal_t cal;
    spectra_cal_get(&cal);
    int md, th, or_, gn;
    spectra_get_settings(&md, &th, &or_, &gn);
    /* pipeline diagnostic — proves what the browser is served (1 Hz max):
     * n==0 -> spectra never fed; real values but empty graph -> JS side */
    static TickType_t t_last_log = 0;
    if (xTaskGetTickCount() - t_last_log > pdMS_TO_TICKS(1000)) {
        t_last_log = xTaskGetTickCount();
        int imn = 255, imx = 0;
        for (int i = 0; i < n; i++) {
            int v = (int)(intensity[i] + 0.5f);
            if (v < imn) imn = v;
            if (v > imx) imx = v;
        }
        ESP_LOGI(TAG, "api/data: n=%d int=%d..%d peaks=%d cal=[%d,%d,%d,%d] or=%d g=%d",
                 n, imn, imx, n_peaks,
                 cal.px1, cal.w1, cal.px2, cal.w2, or_, gn);
    }
    bool hold = spectra_hold();
    bool mjpeg, streaming;
    uint32_t w, h, frames;
    int fps, fmt_idx;
    camera_get_status(&mjpeg, &w, &h, &fps, &fmt_idx, &frames, &streaming);

    int off = snprintf(buf, sizeof(buf), "{\"n\":%d,\"int\":[", n);
    for (int i = 0; i < n && off < (int)sizeof(buf) - 8; i++)
        off += snprintf(buf + off, sizeof(buf) - off, "%s%d", i ? "," : "",
                        (int)(intensity[i] + 0.5f));
    off += snprintf(buf + off, sizeof(buf) - off, "],\"peaks\":[");
    for (int i = 0; i < n_peaks && off < (int)sizeof(buf) - 64; i++) {
        float nm = spectra_px_to_nm(peak_px[i]);
        off += snprintf(buf + off, sizeof(buf) - off,
                        "%s{\"px\":%d,\"nm\":%.1f,\"val\":%.1f}",
                        i ? "," : "", peak_px[i], nm, peak_val[i]);
    }
    off += snprintf(buf + off, sizeof(buf) - off,
                    "],\"cal\":[%d,%d,%d,%d],\"md\":%d,\"th\":%d,\"or\":%d,\"g\":%d,"
                    "\"hold\":%s,\"fmt\":\"%s\",\"w\":%u,\"h\":%u,\"fps\":%d,"
                    "\"frames\":%u,\"stream\":%s,\"net\":\"%s\",\"ip\":\"%s\"",
                    cal.px1, cal.w1, cal.px2, cal.w2,
                    md, th, or_, gn, hold ? "true" : "false",
                    mjpeg ? "mjpeg" : "yuy2", (unsigned)w, (unsigned)h,
                    fps, (unsigned)frames, streaming ? "true" : "false",
                    net_mode_str(), net_ip_str());
    /* camera prefs trail the state; payload closes here */
    off += snprintf(buf + off, sizeof(buf) - off,
                    ",\"raw\":%s,\"exp\":%d}",
                    camera_get_raw() ? "true" : "false", camera_get_exposure_us());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, off);
}

/* POST /api/cal — body: px1=N&w1=N&px2=N&w2=N (int; nm 250..1200) or reset=1 */
static esp_err_t handle_cal(httpd_req_t *req)
{
    char buf[128];
    int n = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (n <= 0)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv");
    buf[n] = 0;
    if (strncmp(buf, "reset=1", 7) == 0) {
        const spectra_cal_t zero = {0};
        if (!spectra_cal_set(&zero))
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "nvs fail");
        char out[40];
        int l = snprintf(out, sizeof(out), "{\"ok\":true,\"cal\":[0,0,0,0]}");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, out, l);
    }
    int px1, w1, px2, w2;
    if (sscanf(buf, "px1=%d&w1=%d&px2=%d&w2=%d", &px1, &w1, &px2, &w2) != 4)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
    if (px1 <= 0 || px2 <= 0 || px1 == px2 ||
        w1 < 250 || w1 > 1200 || w2 < 250 || w2 > 1200)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "range");
    spectra_cal_t cal = {px1, w1, px2, w2};
    if (!spectra_cal_set(&cal))
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "nvs fail");
    char out[96];
    int l = snprintf(out, sizeof(out), "{\"ok\":true,\"cal\":[%d,%d,%d,%d]}",
                     px1, w1, px2, w2);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, l);
}

/* POST /api/settings — body: md=N / th=N / or=N / hold=0|1 / rhold=1 */
static esp_err_t handle_settings(httpd_req_t *req)
{
    char buf[96];
    int n = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (n <= 0)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv");
    buf[n] = 0;
    int v;
    if (sscanf(buf, "md=%d", &v) == 1 && v >= 1 && v <= 200)
        spectra_set_min_dist(v);
    else if (sscanf(buf, "th=%d", &v) == 1 && v >= 0 && v <= 100)
        spectra_set_thresh(v);
    else if (sscanf(buf, "or=%d", &v) == 1 && v >= 0 && v <= 15)
        spectra_set_filter(v);
    else if (sscanf(buf, "g=%d", &v) == 1 && v >= 1 && v <= 50)
        spectra_set_gain(v);
    else if (sscanf(buf, "hold=%d", &v) == 1)
        spectra_set_hold(v != 0);
    else if (strncmp(buf, "rhold=", 6) == 0)
        spectra_reset_hold();
    else
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "param");
    spectra_mark_settings_dirty();
    int md, th, or_, g;
    spectra_get_settings(&md, &th, &or_, &g);
    char out[80];
    int l = snprintf(out, sizeof(out),
                     "{\"ok\":true,\"md\":%d,\"th\":%d,\"or\":%d,\"g\":%d,\"hold\":%s}",
                     md, th, or_, g, spectra_hold() ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, l);
}

/* POST /api/cam — body: raw=0|1 / exp=N ; camera prefs (NVS in camera.c) */
static esp_err_t handle_cam(httpd_req_t *req)
{
    char buf[64];
    int n = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (n <= 0)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv");
    buf[n] = 0;
    int v;
    if (sscanf(buf, "raw=%d", &v) == 1) {
        camera_set_raw(v != 0);
    } else if (sscanf(buf, "exp=%d", &v) == 1 && v >= 0 && v <= 3000000) {
        camera_set_exposure_us(v);
    } else {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "param");
    }
    char out[64];
    int l = snprintf(out, sizeof(out),
                     "{\"ok\":true,\"raw\":%s,\"exp\":%d}",
                     camera_get_raw() ? "true" : "false", camera_get_exposure_us());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, l);
}

/* POST /api/net — body: ssid=NAME&pass=SECRET ; persist + restart STA */
static esp_err_t handle_net(httpd_req_t *req)
{
    char buf[160];
    int n = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (n <= 0)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv");
    buf[n] = 0;
    char ssid[33] = {0}, pass[65] = {0};
    if (sscanf(buf, "ssid=%32[^&]&pass=%64s", ssid, pass) < 1)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
    if (ssid[0] == '\0')
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty ssid");
    net_set_credentials(ssid, pass);
    char out[32] = "{\"ok\":true}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
    vTaskDelay(pdMS_TO_TICKS(100));
    net_reconnect_sta();
    return ESP_OK;
}

/* GET /preview — raw MJPEG frame (live preview) */
static esp_err_t handle_preview(httpd_req_t *req)
{
    const uint8_t *data;
    size_t len;
    if (!camera_get_frame(&data, &len) || len == 0)
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no frame");
    httpd_resp_set_type(req, "image/jpeg");
    return httpd_resp_send(req, (const char *)data, len);
}

/* GET /snapshot.csv */
static esp_err_t handle_csv(httpd_req_t *req)
{
    static char buf[8 * 1024];
    float intensity[MAX_ROWS];
    int n = (int)spectra_get_intensity(intensity);
    int off = snprintf(buf, sizeof(buf), "px,intensity,nm\n");
    for (int i = 0; i < n && off < (int)sizeof(buf) - 32; i++)
        off += snprintf(buf + off, sizeof(buf) - off, "%d,%d,%.1f\n", i,
                        (int)(intensity[i] + 0.5f), spectra_px_to_nm(i));
    httpd_resp_set_type(req, "text/csv");
    return httpd_resp_send(req, buf, off);
}

/* GET /snapshot.png — one-time PSRAM render buffer */
static esp_err_t handle_png(httpd_req_t *req)
{
    static uint8_t *png = NULL;
    size_t png_len = 0;
    if (!png) {
        png = heap_caps_malloc(600 * 1024, MALLOC_CAP_SPIRAM);
        if (!png)
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "no PSRAM");
    }
    spectra_cal_t cal;
    spectra_cal_get(&cal);
    int calarr[4] = {cal.px1, cal.w1, cal.px2, cal.w2};
    float intensity[MAX_ROWS];
    int n = (int)spectra_get_intensity(intensity);
    int peak_px[MAX_PEAKS], n_peaks;
    float peak_val[MAX_PEAKS];
    n_peaks = spectra_peaks(peak_px, peak_val, MAX_PEAKS);
    render_png_spectrum(png, 600 * 1024, &png_len, intensity, n,
                        peak_px, peak_val, n_peaks, calarr);
    if (png_len == 0)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "render");
    httpd_resp_set_type(req, "image/png");
    return httpd_resp_send(req, (const char *)png, png_len);
}

esp_err_t web_start(void)
{
    static httpd_handle_t srv;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 10;
    cfg.stack_size = 12288; /* handle_data/handle_csv place 4 KB float arrays on the worker stack */
    cfg.server_port = 80;
    cfg.ctrl_port = 32768;
    cfg.lru_purge_enable = true;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));
    httpd_uri_t uris[] = {
        { .uri = "/",             .method = HTTP_GET,  .handler = handle_index    },
        { .uri = "/index.html",   .method = HTTP_GET,  .handler = handle_index    },
        { .uri = "/api/data",     .method = HTTP_GET,  .handler = handle_data     },
        { .uri = "/api/cal",      .method = HTTP_POST, .handler = handle_cal      },
        { .uri = "/api/settings", .method = HTTP_POST, .handler = handle_settings },
        { .uri = "/api/net",      .method = HTTP_POST, .handler = handle_net      },
        { .uri = "/api/cam",      .method = HTTP_POST, .handler = handle_cam      },
        { .uri = "/preview",      .method = HTTP_GET,  .handler = handle_preview  },
        { .uri = "/snapshot.csv", .method = HTTP_GET,  .handler = handle_csv      },
        { .uri = "/snapshot.png", .method = HTTP_GET,  .handler = handle_png      },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
        ESP_ERROR_CHECK(httpd_register_uri_handler(srv, &uris[i]));
    ESP_LOGI(TAG, "web server started");
    return ESP_OK;
}
