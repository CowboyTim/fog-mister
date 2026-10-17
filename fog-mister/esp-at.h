/*
 * esp-at.h: Common header file for ESP32/ESP8266 AT command firmware
 *
 * Author: CowboyTim
 *
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <https://unlicense.org>
 */

#ifndef _ESP_AT_H
#define _ESP_AT_H

#include "sdkconfig.h"

// Logging setup for esp32c3

#define SUPPORT_PLUGINS

#ifndef LOGUART
#define LOGUART
#endif // LOGUART

#ifndef BLUETOOTH_UART_AT
#define BLUETOOTH_UART_AT
#endif // BLUETOOTH_UART_AT

#ifndef SUPPORT_ESP_LOG_INFO
#define SUPPORT_ESP_LOG_INFO
#endif // SUPPORT_ESP_LOG_INFO

#ifndef TIMELOG
#define TIMELOG
#endif // TIMELOG

#ifndef LOOP_DELAY
#define LOOP_DELAY
#endif // LOOP_DELAY

#ifndef DEFAULT_HOSTNAME
#define DEFAULT_HOSTNAME "uart"
#endif // DEFAULT_HOSTNAME

#ifndef UART_AT
#define UART_AT
#endif // UART_AT

#ifndef SUPPORT_UART1
#define SUPPORT_UART1
#endif // SUPPORT_UART1

#ifndef SUPPORT_BLE_UART1
#define SUPPORT_BLE_UART1
#endif // SUPPORT_BLE_UART1

#ifndef SUPPORT_GPIO
#define SUPPORT_GPIO
#endif // SUPPORT_GPIO

#if defined(CONFIG_ESP32_WIFI_SUPPORT) || defined(CONFIG_ESP8266_WIFI_SUPPORT)
#if not defined(SUPPORT_WIFI) && SUPPORT_WIFI != 0
#define SUPPORT_WIFI
#endif // SUPPORT_WIFI
#endif // CONFIG_ESP32_WIFI_SUPPORT || CONFIG_ESP8266_WIFI_SUPPORT

#ifdef SUPPORT_WIFI

// WiFi support enabled, enable related features if not explicitly disabled
#ifndef WIFI_WPS
#define WIFI_WPS
#endif // WIFI_WPS

#ifndef SUPPORT_TCP_SERVER
#define SUPPORT_TCP_SERVER
#endif // SUPPORT_TCP_SERVER

#ifndef SUPPORT_TCP
#define SUPPORT_TCP
#endif // SUPPORT_TCP

#undef SUPPORT_TLS

#ifndef SUPPORT_UDP
#define SUPPORT_UDP
#endif // SUPPORT_UDP

#ifndef SUPPORT_NTP
#define SUPPORT_NTP
#endif // SUPPORT_NTP

#ifndef SUPPORT_MDNS
#define SUPPORT_MDNS
#endif // SUPPORT_MDNS

#endif // SUPPORT_WIFI

#endif // _ESP_AT_H
