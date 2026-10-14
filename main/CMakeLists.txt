idf_component_register(
    SRCS "app_main.c" "camera.c" "web.c" "render.c" "net.c"
    REQUIRES usb_host_uvc
    PRIV_REQUIRES
        usb
        esp_psram
        esp_http_server
        esp_app_format
        esp_wifi
        esp_netif
        esp_event
        json
        nvs_flash
        freertos
        spectra
        jpeg_decode
)
