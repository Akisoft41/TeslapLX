TeslapLX
========

Module to make link between a CAN Tesla bus and a program like 'Scan My Tesla'.

Tesla car  --[CAN bus]-->  TeslapLX  --[Bluetooth]-->  'Scan My Tesla'

Program run on a ESP32 WROOM module with a CAN tranceiver:
```
-----------+                       +----------
ESP32      |     +-----------+     | CAN bus
module     |     | SN65HVD   |     |
           |     |           |     |
CAN_TX_PIN +-----+ CTX  CANH +-----+ CANH
           |     |           |     |
CAN_RX_PIN +-----+ CRX  CANL +-----+ CANL
           |     |           |     |
      3.3v +-----+ 3.3v      |     |
           |     |           |     |
       GND +-----+ GND       |     |
           |     +-----------+     |
           |                       |
           |     +-----------+     |
           |     | BUCK 5v   |     |
           |     |           |     |
        5v +-----+ OUT    IN +-----+ 12v
           |     |           |     |
       GND |-----+ GND   GND +-----| GND
           |     +-----------+     |
-----------+                       +----------
```

Connexion:
- Serial UART0 (USB on module)
- Bluetooth SPP
- WIFI TCP port 35000

The program emulate a ELM327 and ST1110 commands.
Emulate partial commands set, sufficient for 'Scan My Tesla'.

The can messages are filtered for know Tesla ID, 
and are limited to one same ID on 100ms (max 10 messages per second and per ID.)

Additionnals commands for configuration:
- REBOOT or RESTART: restart ESP32
- PS: list running tasks
- FREE: list memory utilisation
- ELOG[level] [tag]: set esp log level for tag. Default 0 *
- SIMU START: start a simple simulation of the car
- SIMU STOP: stop simulation
- WIFI: show WIFI fonfiguration
- WIFI STA [ssid] [password]: connect to the ssid with password
- WIFI AP [ssid] [password]: make a WIFI AP
- WIFI STOP: stop the wifi interface
- WIFI SCAN: scan WIFI
- OTA: show current running firmware
- OTA [url]: update firmware by dowloding binary file from url

Default configuration for WIFI:
- AP mode, ssid TeslapLX without password.

The programm is compiled with EDP-IDF 4.1
