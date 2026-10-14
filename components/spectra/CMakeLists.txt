idf_component_register(
    SRCS "spectra.c" "sg_coeff.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES freertos nvs_flash esp_timer
)
