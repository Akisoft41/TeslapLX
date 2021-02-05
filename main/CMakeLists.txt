idf_component_register(
    SRCS "httpd.c" "main.c" "elog.c" "uart.c" "bt.c" "can.c" "elm.c" "wifi.c" "ota.c" "httpd.c"
    INCLUDE_DIRS "."
    REQUIRES vfs bt lwip esp_netif esp_wifi mdns esp_http_client app_update esp_http_server json
)
