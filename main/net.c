/* Wi-Fi: STA with automatic AP fallback.
 *
 * - Credentials (ssid/pass) persist in NVS (ns "spectra"); set once via the
 *   web UI (WiFi setup form) or sdkconfig.defaults.
 * - Boot: if creds exist, STA connect for up to 20 s. On failure (or no
 *   creds): open AP "ESP-Spectrometer".
 * - STA loss later: retry STA; AP only when STA is unavailable.
 */
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "net.h"

#define TAG "net"
#define AP_SSID "ESP-Spectrometer"
#define STA_TIMEOUT_MS 20000

#define BIT_CONNECTED  (1 << 0)
#define BIT_DISCONNECT (1 << 2)

static EventGroupHandle_t s_evt;
static char s_ssid[33] = {0};
static char s_pass[65] = {0};
static bool s_have_creds = false;
static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif = NULL;
static char s_ip_str[16] = "-";
static const char *s_mode = "none";
static uint8_t s_disconn_reason = 0; /* last STA_DISCONNECT reason */

/* NVS */
#define NS "spectra"
#define K_SSID "ssid"
#define K_PASS "pass"

static void load_creds(void)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len = sizeof(s_ssid);
    if (nvs_get_str(h, K_SSID, s_ssid, &len) == ESP_OK && s_ssid[0]) {
        len = sizeof(s_pass);
        if (nvs_get_str(h, K_PASS, s_pass, &len) == ESP_OK) {
            s_have_creds = true;
            ESP_LOGI(TAG, "STA creds from NVS: ssid=%s", s_ssid);
        }
    }
    nvs_close(h);
}

void net_set_credentials(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs open RW failed");
        return;
    }
    nvs_set_str(h, K_SSID, ssid);
    nvs_set_str(h, K_PASS, pass ? pass : "");
    nvs_commit(h);
    nvs_close(h);
    snprintf(s_ssid, sizeof(s_ssid), "%s", ssid);
    snprintf(s_pass, sizeof(s_pass), "%s", pass ? pass : "");
    s_have_creds = true;
    ESP_LOGI(TAG, "creds saved (ssid=%s)", ssid);
}

static void on_system_evt(void *base, const char *event, int32_t id, void *ctx)
{
    (void)base; (void)event;
    if (id == WIFI_EVENT_STA_CONNECTED) {
        xEventGroupSetBits(s_evt, BIT_CONNECTED);
        s_mode = "sta";
        ESP_LOGI(TAG, "STA connected");
    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d =
            (const wifi_event_sta_disconnected_t *)ctx;
        s_disconn_reason = d ? d->reason : 0;
        xEventGroupSetBits(s_evt, BIT_DISCONNECT);
        /* 2=AUTH_EXPIRE 15=4WAY_HANDSHAKE_TIMEOUT (typ. wrong password)
         * 201=NO_AP_FOUND 202=AUTH_FAIL (typ. wrong password) */
        ESP_LOGW(TAG, "STA disconnected reason=%d %s", s_disconn_reason,
                 s_disconn_reason == 2 ? "(AUTH_EXPIRE)" :
                 s_disconn_reason == 15 ? "(4WAY_HANDSHAKE_TIMEOUT — wrong password?)" :
                 s_disconn_reason == 201 ? "(NO_AP_FOUND — router not on 2.4 GHz or out of range?)" :
                 s_disconn_reason == 202 ? "(AUTH_FAIL — wrong password?)" :
                 "");
    }
}

static void wifi_copy(char *dst, size_t n, const char *src)
{
    size_t l = strlen(src);
    if (l >= n) l = n - 1;
    memcpy(dst, src, l);
    dst[l] = 0;
}

static void wifi_set_config(void)
{
    wifi_config_t wc = {0};
    wifi_copy((char *)wc.sta.ssid, sizeof(wc.sta.ssid), s_ssid);
    wifi_copy((char *)wc.sta.password, sizeof(wc.sta.password), s_pass);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
}

static bool netif_has_ip(esp_netif_t *n, char *out_ip)
{
    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(n, &info) != ESP_OK) return false;
    if (info.ip.addr == 0) return false;
    esp_ip4addr_ntoa(&info.ip, out_ip, sizeof(out_ip) > 0 ? 15 : 15);
    return true;
}

/* Wait up to STA_TIMEOUT_MS for the STA to gain an IP.
 * Early-out with the disconnect reason on rejection. Radio must be running
 * in STA mode and already connecting. */
static bool sta_wait_for_ip(void)
{
    TickType_t t0 = xTaskGetTickCount();
    while (xTaskGetTickCount() - t0 < pdMS_TO_TICKS(STA_TIMEOUT_MS)) {
        if (xEventGroupGetBits(s_evt) & BIT_DISCONNECT) {
            ESP_LOGW(TAG, "STA rejected (reason=%d)", s_disconn_reason);
            return false;
        }
        if (netif_has_ip(s_sta_netif, s_ip_str)) {
            ESP_LOGI(TAG, "STA IP: %s", s_ip_str);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    return false;
}

/* Fallback: (re)activate AP. Radio already running (started flag decides). */
static void apsta_fallback(bool already_started)
{
    if (s_have_creds)
        ESP_LOGW(TAG, "STA failed/timeout -> starting AP \"%s\"", AP_SSID);
    else
        ESP_LOGW(TAG, "no STA creds -> starting AP \"%s\"", AP_SSID);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    wifi_config_t ap = {0};
    snprintf((char *)ap.ap.ssid, sizeof(ap.ap.ssid), "%s", AP_SSID);
    ap.ap.ssid_len = strlen(AP_SSID);
    ap.ap.channel = 1;
    ap.ap.authmode = WIFI_AUTH_OPEN;
    ap.ap.max_connection = 4;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    if (!already_started) ESP_ERROR_CHECK(esp_wifi_start());
    s_mode = "ap";
    vTaskDelay(pdMS_TO_TICKS(500));
    if (!netif_has_ip(s_ap_netif, s_ip_str))
        snprintf(s_ip_str, sizeof(s_ip_str), "192.168.4.1");
    ESP_LOGI(TAG, "AP up: %s  ->  http://%s/", AP_SSID, s_ip_str);
}

static void wifi_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_evt = xEventGroupCreate();
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        on_system_evt, NULL, NULL);

    bool sta_ok = false;
    if (s_have_creds) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        wifi_set_config();
        s_mode = "sta";
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "connecting to \"%s\"...", s_ssid);
        /* esp_wifi_start() does NOT join the AP — an explicit connect is
         * required. Its absence made every boot fail the 20 s window
         * silently (no events, no reason) and revert to AP. */
        ESP_ERROR_CHECK(esp_wifi_connect());
        sta_ok = sta_wait_for_ip();
    }

    if (!sta_ok) apsta_fallback(false);
    vTaskDelete(NULL);
}

/* Runs after a credential save: try STA for 20 s; on failure ALWAYS fall
 * back to AP — the radio stays reachable instead of stranding the user
 * in an unreachable STA state until a power cycle. */
static void reconnect_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    if (s_have_creds) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        wifi_set_config();
        s_mode = "sta";
        if (s_evt) xEventGroupClearBits(s_evt, BIT_DISCONNECT | BIT_CONNECTED);
        ESP_ERROR_CHECK(esp_wifi_connect());
        ESP_LOGI(TAG, "reconnecting STA \"%s\"...", s_ssid);
        if (!sta_wait_for_ip()) apsta_fallback(true);
    } else {
        apsta_fallback(true);
    }
    vTaskDelete(NULL);
}

esp_err_t net_start(void)
{
    load_creds();
    xTaskCreate(wifi_task, "net", 8192, NULL, 5, NULL);
    return ESP_OK;
}

const char *net_mode_str(void) { return s_mode; }
const char *net_ip_str(void)  { return s_ip_str; }

void net_reconnect_sta(void)
{
    /* AP fallback now lives in reconnect_task: a failed join leaves the
     * box serving its AP instead of stranding the user in STA-only. */
    if (xTaskCreate(reconnect_task, "netrc", 8192, NULL, 5, NULL) != pdTRUE) {
        ESP_LOGE(TAG, "reconnect task spawn failed");
    }
}
