idf_component_register(
    SRCS "main.c" "elog.c" "uart.c" "bt.c" "can.c" "elm.c" "wifi.c" "ota.c"
    INCLUDE_DIRS "."
    REQUIRES vfs bt lwip esp_wifi mdns esp_http_client app_update
)
