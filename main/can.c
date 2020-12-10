// Copyright 2020 Pascal Akermann
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/can.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"
#include "hal/can_types.h"
#include "sdkconfig.h"

// #include "dbc_vehicle.h"
#include "can.h"

static const char* TAG = "can";

// Data logging shield + CAN transiver
#define CAN_TX_PIN 17
#define CAN_RX_PIN 16

// // ESP32-CAM + CAN transiver
// #define CAN_TX_PIN 2
// #define CAN_RX_PIN 16

#define CAN_TASK_PRIO 9
#define CAN_TASK_CORE 1 // tskNO_AFFINITY
#define CAN_MAX_CB 10

static RingbufHandle_t can_rx_ringbuf[CAN_MAX_CB];
static bool can_simu_task_run;

static can_filter_config_t can_filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
static can_timing_config_t can_timing_config = CAN_TIMING_CONFIG_500KBITS();
static can_general_config_t can_general_config = {
    .mode = CAN_MODE_LISTEN_ONLY,
    .tx_io = (gpio_num_t)CAN_TX_PIN,
    .rx_io = (gpio_num_t)CAN_RX_PIN,
    .clkout_io = CAN_IO_UNUSED,
    .bus_off_io = CAN_IO_UNUSED,
    .tx_queue_len = 5,
    .rx_queue_len = 50,
    .alerts_enabled = CAN_ALERT_ERR_ACTIVE +
                      CAN_ALERT_RECOVERY_IN_PROGRESS +
                      CAN_ALERT_BUS_RECOVERED +
                      CAN_ALERT_ARB_LOST +
                      CAN_ALERT_ABOVE_ERR_WARN +
                      CAN_ALERT_BUS_ERROR +
                      CAN_ALERT_TX_FAILED +
                      // CAN_ALERT_RX_QUEUE_FULL +
                      CAN_ALERT_ERR_PASS +
                      CAN_ALERT_BUS_OFF, // +
                                         // CAN_ALERT_AND_LOG, // CAN_ALERT_NONE,
    .clkout_divider = 0,
};

#ifndef VEHICLEBUS_ID
#define VEHICLEBUS_ID                                        \
    {                                                        \
        0x00C,     /* 12 UI Status */                        \
            0x04F, /* 79 GPS Lat Long */                     \
            0x082, /* 130 UI Trip Planning */                \
            0x102, /* 258 VCLEFT Door Status */              \
            0x103, /* 259 VCRIGHT Door Status */             \
            0x108, /* 264 DIR Torque */                      \
            0x118, /* 280 Drive System Status */             \
            0x123, /* 291 UI Alert Matrix1 */                \
            0x126, /* 294 Rear HV Status */                  \
            0x129, /* 297 Steering Angle */                  \
            0x132, /* 306 HV Battery */                      \
            0x13D, /* 317 CP Charge Status */                \
            0x142, /* 322 VCLEFT Liftgate Status */          \
            0x154, /* 340 Rear Torque Old */                 \
            0x186, /* 390 DIF Torque */                      \
            0x1A5, /* 421 Front HV Status */                 \
            0x1D4, /* 468 Front Torque Old */                \
            0x1D5, /* 469 Front Torque */                    \
            0x1D8, /* 472 Rear Torque */                     \
            0x201, /* 513 VCFRONT Logging And Vitals 10Hz */ \
            0x20A, /* 522 HVP Contactor State */             \
            0x20C, /* 524 VCRIGHT Hvac Request */            \
            0x212, /* 530 BMS Status */                      \
            0x214, /* 532 Fast Charge VA */                  \
            0x215, /* 533 FC Isolation */                    \
            0x217, /* 535 FC Info */                         \
            0x21D, /* 541 CP Evse Status */                  \
            0x221, /* 545 VCFRONT LV Power State */          \
            0x224, /* 548 PCS DCDC Status */                 \
            0x228, /* 552 EPB Right Status */                \
            0x229, /* 553 Gear Lever */                      \
            0x22E, /* 558 PARK Sdi Rear */                   \
            0x23D, /* 573 DCP charge Status */               \
            0x241, /* 577 VCFRONT Coolant */                 \
            0x243, /* 579 VCRIGHT Hvac Status */             \
            0x244, /* 580 Fast Charge Limits */              \
            0x247, /* 583 DAS Autopilot Debug */             \
            0x249, /* 585 SCCM Left Stalk */                 \
            0x252, /* 594 Power Available */                 \
            0x257, /* 599 UI Speed */                        \
            0x25D, /* 605 DCP Status */                      \
            0x261, /* 609 12v Batt Status */                 \
            0x263, /* 611 VCRIGHT Logging 10Hz */            \
            0x264, /* 612 Charge Line Status */              \
            0x266, /* 614 Rear Inverter Power */             \
            0x267, /* 615 DI Vehicle Estimates */            \
            0x268, /* 616 System Power */                    \
            0x281, /* 641 VCFRONT CMP Request */             \
            0x282, /* 642 VCLEFT Hvac Blower Feedback */     \
            0x284, /* 644 UI Vehicle Modes */                \
            0x287, /* 647 PTC Cabin Heat Sensor Status */    \
            0x288, /* 648 EPB Left Status */                 \
            0x292, /* 658 BMS SOC */                         \
            0x293, /* 659 UI Chassis Control */              \
            0x29D, /* 669 CP DC Charge Status */             \
            0x2A8, /* 680 CMPD State */                      \
            0x2B3, /* 691 VCRIGHT Logging 1Hz */             \
            0x2B4, /* 692 PCS DCDC Rail Status */            \
            0x2B6, /* 694 DI Chassis Control Status */       \
            0x2C1, /* 705 VCFront 10hz */                    \
            0x2C4, /* 708 PCS Logging */                     \
            0x2D2, /* 722 BMS VA Limits */                   \
            0x2E1, /* 737 VCFRONT Status */                  \
            0x2E5, /* 741 Front Inverter Power */            \
            0x2F1, /* 753 VCFRONT EFuse Debug Status */      \
            0x2F3, /* 755 UI Hvac Request */                 \
            0x300, /* 768 BMS Info */                        \
            0x301, /* 769 VCFRONT Info */                    \
            0x309, /* 777 DAS Object */                      \
            0x312, /* 786 BMS Thermal */                     \
            0x313, /* 787 UI Track Mode Settings */          \
            0x315, /* 789 Rear Inverter Temps */             \
            0x318, /* 792 System Time UTC */                 \
            0x31C, /* 796 CC Chg Status */                   \
            0x31D, /* 797 CC Chg Status 2 */                 \
            0x320, /* 800 BMS Alert Matrix */                \
            0x321, /* 801 VCFRONT Sensors */                 \
            0x32C, /* 812 CC Log Data */                     \
            0x332, /* 818 Batt Cell Min Max */               \
            0x333, /* 819 UI Charge Request */               \
            0x334, /* 820 UI Powertrain Control */           \
            0x335, /* 821 Rear DI Info */                    \
            0x336, /* 822 Max Power Rating */                \
            0x33A, /* 826 UI Range SOC */                    \
            0x352, /* 850 BMS Energy Status */               \
            0x376, /* 886 Front Inverter Temps */            \
            0x381, /* 897 VCFRONT Logging 1Hz */             \
            0x383, /* 899 VCRIGHT Ths Status */              \
            0x393, /* 915 VCRIGHT Epbm Debug */              \
            0x395, /* 917 DIR Oil Pump */                    \
            0x396, /* 918 Front Oil Pump */                  \
            0x399, /* 921 DAS Status */                      \
            0x3A1, /* 929 VCFRONT Vehicle Status */          \
            0x3B2, /* 946 BMS Log2 */                        \
            0x3B3, /* 947 UI Vehicle Control2 */             \
            0x3B6, /* 950 Odometer */                        \
            0x3BB, /* 955 UI Power */                        \
            0x3C2, /* 962 VCLEFT_switch Status */            \
            0x3C3, /* 963 VCRIGHT Switch Status */           \
            0x3D2, /* 978 Total Charge Discharge */          \
            0x3D8, /* 984 Elevation */                       \
            0x3D9, /* 985 UI GPS Vehicle Speed */            \
            0x3E2, /* 994 VCLEFT Light Status */             \
            0x3E3, /* 995 VCRIGHT Light Status */            \
            0x3E9, /* 1001 DAS Body Controls */              \
            0x3F2, /* 1010 BMS Counters */                   \
            0x3F5, /* 1013 VCFRONT Lighting */               \
            0x3FE, /* 1022 Brake Temps Estimated */          \
            0x401, /* 1025 Cell Voltages */                  \
            0x405, /* 1029 VIN */                            \
            0x42A, /* 1066 VCSEC TPMS Connection Data */     \
            0x43D, /* 1085 CP Charge Status Log */           \
            0x51E, /* 1310 FC Info */                        \
            0x528, /* 1320 Unix Time */                      \
            0x541, /* 1345 Fast Charge Max Limits */         \
            0x556, /* 1366 Front DI Temps */                 \
            0x557, /* 1367 Front Thermal Control */          \
            0x5D5, /* 1493 Rear DI Temps */                  \
            0x5D7, /* 1495 Rear Thermal Control */           \
            0x628, /* 1576 UDS MCU to PCS */                 \
            0x629, /* 1577 UDS PCS to MCU */                 \
            0x656, /* 1622 Front DI Info */                  \
            0x743, /* 1859 VCRIGHT Recall Status */          \
            0x757, /* 1879 DIF Debugs */                     \
            0x75D, /* 1885 CP Sensor Data */                 \
            0x7AA, /* 1962 HVP Debug Message */              \
            0x7D5, /* 2005 DIR Debug */                      \
            0x7FF, /* 2047 Car Config */                     \
    }
#endif

// STFAP 132,7FF
// STM

static const uint32_t can_id[] = VEHICLEBUS_ID;
static const int can_id_count = sizeof(can_id) / sizeof(*can_id);
static const uint32_t can_id_delay_us = 1000000 / 11; // max msg per second
static uint32_t can_id_last_us[sizeof(can_id) / sizeof(*can_id)] = {0};

void _can_raise(can_message_timestamp_t* msg)
{
    for (int i = 0; i < CAN_MAX_CB; i++) {
        if (can_rx_ringbuf[i]) {
            BaseType_t done = xRingbufferSend(can_rx_ringbuf[i], msg, sizeof(can_message_timestamp_t), 0);
            if (done == 0)
                ESP_LOGW(TAG, "rx buffer full");
        }
    }
}

RingbufHandle_t can_ringbuf_new(size_t itemNum)
{
    for (int i = 0; i < CAN_MAX_CB; i++) {
        if (can_rx_ringbuf[i] == NULL) {
            can_rx_ringbuf[i] = xRingbufferCreateNoSplit(sizeof(can_message_timestamp_t), itemNum);
            return can_rx_ringbuf[i];
        }
    }
    return NULL;
}

bool can_ringbuf_del(RingbufHandle_t ringbuf)
{
    if (ringbuf == NULL) return true;
    for (int i = 0; i < CAN_MAX_CB; i++) {
        if (can_rx_ringbuf[i] == ringbuf) {
            vRingbufferDelete(can_rx_ringbuf[i]);
            can_rx_ringbuf[i] = NULL;
            return true;
        }
    }
    return false;
}

bool _can_filter_id(uint32_t id, uint32_t ts)
{
    for (int i = 0; i < can_id_count; i++) {
        if (id != can_id[i]) continue;

        if ((ts - can_id_last_us[i]) >= can_id_delay_us) {
            can_id_last_us[i] = ts;
            return true;
        }
        else
            return false;
    }
    return false;
}

void can_simu_task(void* param)
{
    can_message_timestamp_t msg;
    int can_count = 0;
    uint32_t stat_us = esp_timer_get_time();
    TickType_t last_tick = xTaskGetTickCount();

    ESP_LOGI(TAG, "simu task started");


//static const sg_t SG_132_SmoothBattCurrent132 = {"SmoothBattCurrent132", 0, 16, 16, true, true, -0.1, 0, -3276.7, 3276.7, "A", "HV Battery Current", {{0, NULL}}};
//static const sg_t SG_257_UIspeed_signed257 = {"UIspeed_signed257", 0, 12, 12, true, false, 0.08, -40, -40, 287.6, "KPH", "Vehicle Speed", {{0, NULL}}};

    int i = 0;
    while (can_simu_task_run) {

        vTaskDelayUntil(&last_tick, 10);

        i++;
        int16_t current = 3;
        uint16_t speed = 0;
        switch (i / 100)
        {
        case 0:
            current = 3;
            speed = 0;
            break;
        case 1:
            current = 30;
            speed = i % 100;
            break; 
        case 2:
            current = 30;
            speed = 100;
            break;
        case 3:
            current = -20;
            speed = 100 - (i % 100);
            break;
        
        default:
            i = 0;
            break;
        }

        current = -current * 10;
        speed = (uint16_t)((float)(speed + 40) / 0.08f + 0.5f);

        // 132 04 89 E4 FF D2 26 FF 0F
        msg.timestamp = esp_timer_get_time();
        msg.msg.identifier = 0x132;
        msg.msg.data_length_code = 8;
        msg.msg.data[0] = 0x40; // Battery Voltage 400V
        msg.msg.data[1] = 0x9C;
        msg.msg.data[2] = current; // Battery Current
        msg.msg.data[3] = current >> 8;
        msg.msg.data[4] = 0; // Raw Current
        msg.msg.data[5] = 0;
        msg.msg.data[6] = 0xFF; // Charge Time Remaining
        msg.msg.data[7] = 0x0F;
        _can_raise(&msg);
        can_count++;

        // 257 C3 49 1F 00 02 00 00 00 
        msg.timestamp = esp_timer_get_time();
        msg.msg.identifier = 0x257;
        msg.msg.data_length_code = 8;
        msg.msg.data[0] = 0x00;
        msg.msg.data[1] = speed << 4; // Vehicle Speed
        msg.msg.data[2] = speed >> 4; // 0x1F;
        msg.msg.data[3] = 0x00;
        msg.msg.data[4] = 0x02;
        msg.msg.data[5] = 0x00;
        msg.msg.data[6] = 0x00;
        msg.msg.data[7] = 0x00;
        _can_raise(&msg);
        can_count++;

        // stat
        uint32_t us = esp_timer_get_time();
        uint32_t time_us = (us - stat_us);
        if (time_us >= 10 * 1000000) {
            ESP_LOGI(TAG, "simu stat: count=%i/s", (int)((float)can_count / ((float)time_us / 1000000)));
            can_count = 0;
            stat_us = us;
        }
    }

    ESP_LOGI(TAG, "simu task stopped");
    vTaskDelete(NULL);
}

void can_task(void* param)
{
    can_message_t rx_msg;
    can_message_timestamp_t rx_msg_ts;
    uint32_t alerts;
    int can_count = 0;
    int can_count_all = 0;
    can_status_info_t stat;
    uint32_t stat_missed = 0;
    uint32_t stat_error = 0;
    esp_err_t err;
    uint32_t stat_us = esp_timer_get_time();

    ESP_LOGI(TAG, "rx task started");

    while (true) {

        // can receive
        err = can_receive(&rx_msg, portMAX_DELAY); // pdMS_TO_TICKS(100)); // min timeout: 100 ms
        uint64_t ts = esp_timer_get_time();
        // if (err == ESP_ERR_TIMEOUT) {
        //     // ESP_LOGI(TAG, "timeout");
        //     rx_msg.identifier = 0;
        //     rx_msg.data_length_code = 0;
        //     _can_raise(&rx_msg, ts); // timeout msg
        // } else
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "receive error 0x%x %s", err, esp_err_to_name(err));
            break;
        }
        else if (_can_filter_id(rx_msg.identifier, (uint32_t)ts)) {
            can_count_all++;
            can_count++;
            rx_msg_ts.timestamp = ts;
            memcpy(&rx_msg_ts.msg, &rx_msg, sizeof(rx_msg));
            _can_raise(&rx_msg_ts); // can msg
        }
        else {
            // filtered msg
            can_count_all++;
        }

        // stat
        uint32_t us = ts;
        uint32_t time_us = (us - stat_us);
        if (time_us >= 10 * 1000000) {

            err = can_read_alerts(&alerts, 0);
            if (err == ESP_OK && alerts > 0) {
                ESP_LOGW(TAG, "alerts: %u", alerts);
            }

            err = can_get_status_info(&stat);
            if (err == ESP_OK) {
                if (stat.state != CAN_STATE_RUNNING) {
                    ESP_LOGE(TAG, "status error state=%i", stat.state);
                    break;
                }
                if (can_count_all > 0) {
                    ESP_LOGI(TAG, "stat: tot=%i/s count=%i/s rx=%i missed=%u error=%u",
                             (int)((float)can_count_all / ((float)time_us / 1000000)),
                             (int)((float)can_count / ((float)time_us / 1000000)),
                             stat.msgs_to_rx,
                             stat.rx_missed_count - stat_missed,
                             stat.rx_error_counter + stat.bus_error_count - stat_error);
                    stat_missed = stat.rx_missed_count;
                    stat_error = stat.rx_error_counter + stat.bus_error_count;
                }
            }
            else {
                ESP_LOGI(TAG, "stat: count=%u", can_count);
            }
            can_count = 0;
            can_count_all = 0;
            stat_us = us;
        }
    }

    ESP_LOGI(TAG, "rx task stopped");
    vTaskDelete(NULL);
}

bool can_init()
{
    // can_filter_config.acceptance_code = 0x00000000;
    // can_filter_config.acceptance_mask = 0x7fffffff; // mask=0x3ff

    esp_err_t err = can_driver_install(&can_general_config, &can_timing_config, &can_filter_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "driver install error 0x%x %s", err, esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Driver installed");
    err = can_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start error 0x%x %s", err, esp_err_to_name(err));
        return false;
    }
    memset(can_id_last_us, 0, sizeof(can_id_last_us));
    ESP_LOGI(TAG, "Driver started");

    memset(can_rx_ringbuf, 0, sizeof(can_rx_ringbuf));

    xTaskCreatePinnedToCore(can_task, "can", 8 * 1024, NULL, CAN_TASK_PRIO, NULL, CAN_TASK_CORE);
    return true;
}

void can_simu_start()
{
    if (can_simu_task_run) return;
    can_simu_task_run = true;
    xTaskCreatePinnedToCore(can_simu_task, "can-simu", 2 * 1024, NULL, CAN_TASK_PRIO, NULL, CAN_TASK_CORE);
}

void can_simu_stop()
{
    can_simu_task_run = false;
}
