/* ESP32-S3 USB-UVC spectrometer — entry point.
 *
 * Pipeline: UVC camera (USB OTG) -> MJPEG decode (vendored libjpeg)
 *           -> middle-row grayscale -> Savitzky-Golay + peak detect
 *           -> web server (canvas UI, calibration, snapshots)
 * Network: Wi-Fi STA (NVS creds) with automatic open-AP fallback.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"
#include "net.h"
#include "web.h"
#include "camera.h"
#include "spectra.h"

#define TAG "main"

static void status_task(void *arg)
{
    int last_frames = -1;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        bool mjpeg, streaming;
        uint32_t w, h, frames;
        int fps, idx;
        camera_get_status(&mjpeg, &w, &h, &fps, &idx, &frames, &streaming);
        if ((int)frames != last_frames) {
            last_frames = (int)frames;
            ESP_LOGI(TAG, "cam: %s %ux%u@%d frames=%u streaming=%d | net=%s ip=%s",
                     mjpeg ? "mjpeg" : "yuy2", (unsigned)w, (unsigned)h, fps,
                     (unsigned)frames, (int)streaming,
                     net_mode_str(), net_ip_str());
        }
    }
}

void app_main(void)
{
    /* NVS (calibration, settings, Wi-Fi creds) */
    esp_err_t rc = nvs_flash_init();
    if (rc == ESP_ERR_NVS_NO_FREE_PAGES || rc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        rc = nvs_flash_init();
    }
    ESP_ERROR_CHECK(rc);

    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "=== ESP32-S3 spectrometer v%s (IDF %s) ===", app->version, IDF_VER);
    size_t psram = esp_psram_get_size();
    ESP_LOGI(TAG, "PSRAM: %u KB total, %u KB free | DRAM free: %u KB",
             (unsigned)(psram / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    if (psram < 4 * 1024 * 1024)
        ESP_LOGW(TAG, "PSRAM smaller than expected — frame buffers may fail");

    spectra_init();
    ESP_ERROR_CHECK(camera_start());
    /* IDLE1 watchdog is disabled at build time (sdkconfig:
     * CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=n) — the cam task
     * legitimately saturates core 1 with JPEG decode. CPU0 stays watched. */
    ESP_ERROR_CHECK(net_start());
    ESP_ERROR_CHECK(web_start());

    xTaskCreate(status_task, "status", 4096, NULL, 3, NULL);
    ESP_LOGI(TAG, "ready");
}
