/*
 * esp-at.ino - ESP32/ESP8266 AT command firmware with WiFi, BLE UART, and networking support
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

#include "esp-at.h"
#include "common.h"
#include "plugins.h"

#include <Arduino.h>
#ifdef DEBUG
#define USE_ESP_IDF_LOG
#define CORE_DEBUG_LEVEL 5
#define LOG_LOCAL_LEVEL 5
#endif // DEBUG
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_partition.h>
#include <esp_spiffs.h>
#include <esp_heap_caps.h>
#include <SPIFFS.h>
#include <esp_bt.h>
#include <esp_system.h>
#include <esp_efuse.h>
#include <esp_mac.h>
#ifdef SUPPORT_WIFI
#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_wps.h>
#include <ESPmDNS.h>
#endif // SUPPORT_WIFI

#ifdef SUPPORT_NTP
#include <esp_sntp.h>
#endif // SUPPORT_NTP

#include <errno.h>
#include <sys/time.h>
#include "time.h"

#ifndef LED
#define LED    GPIO_NUM_8
#endif
#ifndef SUPPORT_LED_BRIGHTNESS
#define SUPPORT_LED_BRIGHTNESS
#endif

#ifndef BUTTON_BUILTIN
#define BUTTON_BUILTIN  GPIO_NUM_9
#endif
uint8_t current_button = BUTTON_BUILTIN;

#ifndef DEFAULT_HOSTNAME
#define DEFAULT_HOSTNAME "uart"
#endif // DEFAULT_HOSTNAME

#ifndef SUPPORT_WIFI
// no WiFi support, disable related features
#undef WIFI_WPS
#undef SUPPORT_TCP_SERVER
#undef SUPPORT_TCP
#undef SUPPORT_TLS
#undef SUPPORT_UDP
#undef SUPPORT_NTP
#undef SUPPORT_MDNS
#endif // !SUPPORT_WIFI

#ifdef SUPPORT_TLS
#include <WiFiClientSecure.h>
#endif // SUPPORT_TLS

#ifdef SUPPORT_NTP
#include <esp_sntp.h>
#endif // SUPPORT_NTP

#ifdef SUPPORT_BLE_UART1
#define SUPPORT_UART1
#define BLUETOOTH_UART_AT
#endif // SUPPORT_BLE_UART1

#ifndef VERBOSE
#undef SUPPORT_ESP_LOG_INFO
#undef DEBUG
#endif // VERBOSE

#ifdef SUPPORT_UART1
#define UART1_RX_PIN 0
#define UART1_TX_PIN 1
#endif // SUPPORT_UART1

#if defined(SUPPORT_TCP) || defined(SUPPORT_UDP) || defined(SUPPORT_TCP_SERVER)
#include <fcntl.h>
#include <sys/select.h>
#endif // SUPPORT_TCP || SUPPORT_UDP

#undef BT_CLASSIC

#if defined(UART_AT) || defined(BLUETOOTH_UART_AT) || defined(BT_CLASSIC)
#include "SerialCommands.h"
void sc_cmd_handler(SerialCommands* s, const char* atcmdline);
#endif

// Function declarations
#ifdef SUPPORT_WIFI
void WiFiEvent(WiFiEvent_t event);
uint8_t restart_networking = 0;
#endif

#define UART1_READ_SIZE       64 // read bytes at a time from UART1
#define UART1_WRITE_SIZE      64 // write bytes at a time to UART1
#define TCP_READ_SIZE         16 // read bytes at a time from TCP
#define REMOTE_BUFFER_SIZE   512 // max size of REMOTE buffer
#define LOCAL_BUFFER_SIZE    512 // max size of LOCAL buffer

// from "LOCAL", e.g. "UART1", add 1 byte for \0 during buffer prints in debugging/logging
ALIGN(4) uint8_t inbuf[LOCAL_BUFFER_SIZE+1] = {0};
// max size of inbuf, notice the -1, as we will never fill up that byte
const uint8_t *inbuf_max = (uint8_t *)&inbuf + LOCAL_BUFFER_SIZE;

// from "REMOTE", e.g. TCP, UDP
ALIGN(4) uint8_t outbuf[REMOTE_BUFFER_SIZE] = {0};
size_t outlen = 0;

uint8_t sent_ok = 0;

#ifdef LOOP_DEBUG
#define LOOP_D D
#define LOOP_R R
#define LOOP_T T
#define LOOP_E E
#else
#define LOOP_D(...)
#define LOOP_R(...)
#define LOOP_T(...)
#define LOOP_E(...)
#endif

/* Bluetooth support */
#if defined(BLUETOOTH_UART_AT) || defined(SUPPORT_BLE_UART1)

#define BT_BLE

#ifndef BLUETOOTH_UART_DEVICE_NAME
#define BLUETOOTH_UART_DEVICE_NAME DEFAULT_HOSTNAME
#endif // BLUETOOTH_UART_DEVICE_NAME

#ifndef BLUETOOTH_UART_DEFAULT_PIN
#define BLUETOOTH_UART_DEFAULT_PIN 123456
#endif

#ifdef BT_CLASSIC
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#warning Bluetooth is not enabled or possible.
#undef BT_CLASSIC
#endif // CONFIG_BT_ENABLED
#if !defined(CONFIG_BT_SPP_ENABLED)
#warning Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#undef BT_CLASSIC
#endif // CONFIG_BT_SPP_ENABLED
#endif // BT_CLASSIC

#ifdef BT_BLE
#include <BLEUUID.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEService.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>
#include "BLESecurity.h"
#include "esp_blufi.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"

#define BLE_ADVERTISING_TIMEOUT 10000   // 10 seconds in milliseconds
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
unsigned long ble_advertising_start = 0;
uint8_t deviceConnected = 0;
uint8_t securityRequestPending = 0;
uint32_t passkeyForDisplay = 0;
#endif // BT_BLE
#endif // BLUETOOTH_UART_AT

#ifdef BT_CLASSIC
/* AT commands over Classic Serial Bluetooth */
BluetoothSerial SerialBT;
ALIGN(4) char atscbt[128] = {""};
SerialCommands ATScBT(&SerialBT, atscbt, sizeof(atscbt), "\r\n", "\r\n");
#endif

/* NTP server to use, can be configured later on via AT commands */
#ifndef DEFAULT_NTP_SERVER
#define DEFAULT_NTP_SERVER "at.pool.ntp.org"
#endif

/* Default DNS server for static IPv4 configuration */
#ifndef DEFAULT_DNS_IPV4
#define DEFAULT_DNS_IPV4 "1.1.1.1"
#endif

#ifdef UART_AT
/* our AT commands over UART */
ALIGN(4) char atscbu[128] = {""};
SerialCommands ATSc(&USBSERIAL0, atscbu, sizeof(atscbu), "\r\n", "\r\n");
#endif // UART_AT

#define CFGVERSION 0x02 // switch between 0x01/0x02/0x03 to reinit the config struct change
#define CFGINIT    0x72 // at boot init check flag

#define IPV4_DHCP    1
#define IPV4_STATIC  2
#define IPV6_SLAAC   4

#define MAX_WIFI_SSID 32
#define MAX_WIFI_PASS 64
#define MAX_HOSTNAME  64

/* main config */
typedef struct cfg_t {
  uint8_t initialized  = 0;
  uint8_t version      = 0;
  #ifdef VERBOSE
  uint8_t do_verbose   = 1;
  #endif
  #ifdef LOGUART
  uint8_t do_log       = 0;
  #endif
  #ifdef TIMELOG
  uint8_t do_timelog   = 0;
  #endif
  #ifdef LOOP_DELAY
  unsigned long main_loop_delay = 0; // 0.1 seconds
  #endif
  #ifdef SUPPORT_ESP_LOG_INFO
  unsigned long esp_log_interval = 60000; // 60 seconds
  #endif

  #ifdef SUPPORT_WIFI

  uint8_t wifi_enabled = 1;   // WiFi enabled by default
  char wifi_ssid[MAX_WIFI_SSID] = {0}; // max 31 + 1
  char wifi_pass[MAX_WIFI_PASS] = {0}; // nax 63 + 1

  #ifdef SUPPORT_NTP
  uint8_t ntp_enabled         = 1;    // NTP enabled by default
  char ntp_host[MAX_HOSTNAME] = {0}; // max hostname + 1
  #endif // SUPPORT_NTP

  #ifdef SUPPORT_MDNS
  uint8_t mdns_enabled             = 1;   // mDNS enabled by default
  char mdns_hostname[MAX_HOSTNAME] = {0}; // mDNS hostname, defaults to hostname if empty
  #endif // SUPPORT_MDNS

  uint8_t ip_mode      = IPV4_DHCP | IPV6_SLAAC;
  char hostname[MAX_HOSTNAME] = {0}; // max hostname + 1
  uint8_t ipv4_addr[4] = {0}; // static IP address
  uint8_t ipv4_gw[4]   = {0}; // static gateway
  uint8_t ipv4_mask[4] = {0}; // static netmask
  uint8_t ipv4_dns[4]  = {0}; // static DNS server

  #ifdef SUPPORT_UDP
  // UDP support
  uint16_t udp_port         = 0;
  uint16_t udp_listen_port  = 0; // local UDP port to listen on, 0=disabled (IPv4/IPv6 auto-detect)
  uint16_t udp6_listen_port = 0; // local UDP IPv6 port to listen on, 0=disabled
  uint16_t udp_send_port = 0;    // remote UDP port to send to, 0=disabled
  char udp_send_ip[40] = {0};    // remote UDP host to send to, IPv4 or IPv6 string
  char udp_host_ip[40] = {0};    // remote UDP IPv4 or IPv6 string
  #endif // SUPPORT_UDP

  #ifdef SUPPORT_TCP
  // TCP client support
  uint16_t tcp_port    = 0;
  char tcp_host_ip[40] = {0};
  #endif // SUPPORT_TCP

  #ifdef SUPPORT_TLS
  // TLS/SSL configuration
  uint8_t tls_enabled = 0;          // 0=disabled, 1=enabled for TCP connections
  uint8_t tls_verify_mode = 1;      // 0=none, 1=optional, 2=required
  uint8_t tls_use_sni = 1;          // 0=disabled, 1=enabled (Server Name Indication)
  uint16_t tls_port = 0;            // TLS port (if different from tcp_port)
  #endif // SUPPORT_TLS

  #ifdef SUPPORT_TCP_SERVER
  // TCP server support
  uint16_t tcp_server_port = 0; // TCP server port (IPv4/IPv6 dual-stack)
  uint16_t tcp6_server_port = 0; // TCP server IPv6-only port, 0=disabled
  uint8_t tcp_server_max_clients = 8; // maximum concurrent client connections
  #endif // SUPPORT_TCP_SERVER
  #endif // SUPPORT_WIFI

  #ifdef SUPPORT_UART1
  // UART1 configuration
  uint32_t uart1_baud  = 115200;       // baud rate
  uint8_t uart1_data   = 8;            // data bits (5-8)
  uint8_t uart1_parity = 0;            // parity: 0=None, 1=Even, 2=Odd
  uint8_t uart1_stop   = 1;            // stop bits (1-2)
  uint8_t uart1_rx_pin = UART1_RX_PIN; // RX pin
  uint8_t uart1_tx_pin = UART1_TX_PIN; // TX pin
  uint8_t uart1_inv    = 0;            // 0=normal, 1=inverted
  #endif // SUPPORT_UART1

  #ifdef BLUETOOTH_UART_AT
  // BLE security configuration
  uint32_t ble_pin = BLUETOOTH_UART_DEFAULT_PIN; // PIN code for pairing, 0=none
  uint8_t ble_security_mode = 0;   // Security mode: 0=None, 1=PIN, 2=Bonding
  uint8_t ble_io_cap   = 0;        // IO capability: 0=DisplayOnly, 1=DisplayYesNo, 2=KeyboardOnly, 3=NoInputNoOutput, 4=KeyboardDisplay
  uint8_t ble_auth_req = 0;        // Authentication requirements: 0=None, 1=Bonding, 2=MITM, 3=Bonding+MITM

  // BLE MAC address configuration
  uint8_t ble_addr_type = 0;       // Address type: 0=Public, 1=Random Static, 2=Private Resolvable, 3=Private Non-resolvable
  uint8_t ble_custom_addr[6] = {0}; // Custom MAC address (6 bytes), all zeros = use default
  uint8_t ble_addr_auto_random = 1; // Auto-generate random static address if needed
  #endif // BLUETOOTH_UART_AT

  #ifdef SUPPORT_BLE_UART1
  uint8_t ble_uart1_bridge = 0; // 0=disabled, 1=enabled
  #endif // SUPPORT_BLE_UART1

  #ifdef SUPPORT_GPIO
  // Persistent GPIO config: up to 10 pins
  struct {
    int8_t pin   = -1; // GPIO pin number
    int8_t mode  = -1; // -1=none, 0=input, 1=input-pullup, 2=output
    int8_t value = -1; // 0=low, 1=high (for output mode)
  } gpio_cfg[10];
  uint8_t gpio_cfg_count = 0;
  #endif // SUPPORT_GPIO
};
cfg_t cfg;

#if defined(SUPPORT_WIFI) && defined(SUPPORT_NTP)
RTC_DATA_ATTR long last_ntp_log = 0;
RTC_DATA_ATTR int8_t last_hour = -1;
RTC_DATA_ATTR int8_t ntp_is_synced = 0;


void cb_ntp_synced(struct timeval *tv) {
  LOG("[NTP] NTP time synced, system time updated: %s", ctime(&tv->tv_sec));
  ntp_is_synced = 1;
}

NOINLINE
void setup_ntp() {
  // Check if NTP is enabled
  if(cfg.ntp_enabled == 0) {
    LOG("[NTP] NTP is disabled");
    if(esp_sntp_enabled())
      esp_sntp_stop();
    return;
  }
  // Default to DEFAULT_NTP_SERVER if ntp_host isn't set
  if(strlen(cfg.ntp_host) == 0) {
    strcpy((char *)&cfg.ntp_host, (char *)DEFAULT_NTP_SERVER);
    LOG("[NTP] no NTP host configured, defaulting to: %s", DEFAULT_NTP_SERVER);
  }
  // if we have a NTP host configured, sync
  if(strlen(cfg.ntp_host)) {
    LOG("[NTP] setting up NTP with host: %s, interval: %d, timezone: UTC", cfg.ntp_host, 4 * 3600);
    if(esp_sntp_enabled()) {
      LOG("[NTP] already enabled, skipping setup");
      sntp_set_sync_interval(4 * 3600 * 1000UL);
      sntp_setservername(0, (char*)&cfg.ntp_host);
    } else {
      LOG("[NTP] setting up NTP sync");
      esp_sntp_stop();
      sntp_set_sync_interval(4 * 3600 * 1000UL);
      sntp_setservername(0, (char*)&cfg.ntp_host);
      sntp_set_time_sync_notification_cb(cb_ntp_synced);
      sntp_setoperatingmode(SNTP_OPMODE_POLL);
      sntp_init();
    }
    setenv("TZ", "UTC", 1);
    tzset();
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    last_hour = timeinfo.tm_hour;
  } else {
    LOG("[NTP] no NTP host configured, skipping NTP setup");
    if(esp_sntp_enabled())
      esp_sntp_stop();
  }
}
#endif // SUPPORT_WIFI && SUPPORT_NTP

#ifdef SUPPORT_MDNS
NOINLINE
void setup_mdns() {
  // Check if mDNS is enabled
  if(!cfg.mdns_enabled) {
    LOG("[mDNS] mDNS is disabled, skipping mDNS setup");
    return;
  }

  // Determine hostname to use for mDNS
  const char* hostname_to_use = NULL;
  if(strlen(cfg.mdns_hostname) > 0) {
    hostname_to_use = cfg.mdns_hostname;
  } else if(strlen(cfg.hostname) > 0) {
    hostname_to_use = cfg.hostname;
  } else {
    hostname_to_use = DEFAULT_HOSTNAME;
  }

  LOG("[mDNS] Starting mDNS responder with hostname: %s.local", hostname_to_use);

  // Start mDNS responder
  if(MDNS.begin(hostname_to_use)) {
    LOG("[mDNS] mDNS responder started successfully");

    uint16_t port = 0;

    #if defined(SUPPORT_TCP) || defined(SUPPORT_TCP_SERVER)
    // TCP Server: Add service to MDNS-SD: _uart._tcp._local
    port = cfg.tcp_server_port ? cfg.tcp_server_port : (cfg.tcp6_server_port ? cfg.tcp6_server_port : 0);
    if(port != 0) {
      MDNS.addService("uart", "tcp", port);
      LOG("[mDNS] Added UART service on port %d", port);

      // Add additional service information
      MDNS.addServiceTxt("uart", "tcp", "device", "ESP-AT-UART");
      MDNS.addServiceTxt("uart", "tcp", "version", "1.0");
    }
    #endif // SUPPORT_TCP || SUPPORT_TCP_SERVER

    #ifdef SUPPORT_UDP
    // UDP Listener: Add service to MDNS-SD: _uart._udp._local
    port = cfg.udp_listen_port ? cfg.udp_listen_port : (cfg.udp6_listen_port ? cfg.udp6_listen_port : 0);
    if(port != 0) {
      MDNS.addService("uart", "udp", port);
      LOG("[mDNS] Added UART UDP service on port %d", port);

      // Add additional service information
      MDNS.addServiceTxt("uart", "udp", "device", "ESP-AT-UART");
      MDNS.addServiceTxt("uart", "udp", "version", "1.0");
    }
    #endif // SUPPORT_UDP
  } else {
    LOGE("[mDNS] Error setting up mDNS responder");
  }
}

NOINLINE
void stop_mdns() {
  LOG("[mDNS] Stopping mDNS responder");
  MDNS.end();
  LOG("[mDNS] mDNS responder stopped");
}
#endif // SUPPORT_MDNS

/* state flags */
#ifdef SUPPORT_WIFI
RTC_DATA_ATTR long last_wifi_check = 0;
RTC_DATA_ATTR long last_wifi_info_log = 0;
RTC_DATA_ATTR long last_wifi_reconnect = 0;
#endif // SUPPORT_WIFI

#ifdef TIMELOG
#define TIMELOG_INTERVAL 60000 // 60 seconds
RTC_DATA_ATTR long last_time_log = 0;
#endif // TIMELOG

#ifdef SUPPORT_ESP_LOG_INFO
RTC_DATA_ATTR long last_esp_info_log = 0;
#endif // SUPPORT_ESP_LOG_INFO

//typedef int8_t FD; // file descriptor type
#define FD int8_t

#ifdef SUPPORT_UDP
FD udp_sock = -1;
FD udp_listen_sock = -1;
FD udp6_listen_sock = -1;
FD udp_out_sock = -1;
#endif // SUPPORT_UDP

#ifdef LED

// PWM settings for LED brightness control
// Uses hardware PWM on ESP32 for smooth brightness control
#define LED_PWM_CHANNEL 2
#define LED_PWM_FREQUENCY 5000  // 5 kHz (above human hearing range)
#define LED_PWM_RESOLUTION 8    // 8-bit resolution (0-255 brightness levels)

// Brightness levels for different states (255=max, 0=min on ESP32)
#define LED_BRIGHTNESS_OFF     255  // LED completely off
#define LED_BRIGHTNESS_ON        0  // LED at full brightness
#define LED_BRIGHTNESS_LOW     200  // LED at low brightness (dimmed)
#define LED_BRIGHTNESS_DIM     150  // LED at dim brightness
#define LED_BRIGHTNESS_MEDIUM  100  // LED at medium brightness
#define LED_BRIGHTNESS_HIGH     50  // LED at high brightness (bright)
#define LED_BRIGHTNESS_FLICKER  20  // LED at very high brightness for flicker

// Blink intervals for different states
#define LED_BLINK_INTERVAL_SLOW     2000  // slow blink (not connected, BLE connected, WPS)
#define LED_BLINK_INTERVAL_NORMAL   1000  // normal blink, startup
#define LED_BLINK_INTERVAL_HALF      500  // half blink (WiFi connecting)
#define LED_BLINK_INTERVAL_QUICK     250  // quick blink (WPS Waiting)
#define LED_BLINK_INTERVAL_FAST      100  // fast blink (BLE advertising)
#define LED_BLINK_INTERVAL_FLICKER    50  // quick flicker for data activity
#define LED_BLINK_OFF                  0  // no blinking, solid on

// LED PWM mode tracking for ESP32
bool led_pwm_enabled = false; // Track if PWM is working on ESP32

// LED state tracking
unsigned long last_activity = 0;
#define COMM_ACTIVITY_LED_DURATION 200  // Show communication activity for 200ms
#endif // LED

// WPS support
#if defined(SUPPORT_WIFI) && defined(WIFI_WPS)
#define WPS_TIMEOUT_MS 30000 // 30 seconds
bool wps_running = false;
unsigned long wps_start_time = 0;
#endif // SUPPORT_WIFI && WIFI_WPS

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
FD tcp_server_sock = -1;
FD tcp6_server_sock = -1;
#endif // SUPPORT_WIFI && SUPPORT_TCP_SERVER

#ifdef SUPPORT_WIFI
NOINLINE
void setup_wifi() {
  LOG("[WiFi] setup started");

  // Check if WiFi is enabled
  if(!cfg.wifi_enabled) {
    LOG("[WiFi] WiFi is disabled, skipping WiFi setup");
    WiFi.mode(WIFI_MODE_NULL);
    return;
  }

  // set persistence off
  WiFi.persistent(false);

  // first disconnect if already connected
  if(WiFi.STA.connected()){
    WiFi.disconnect(true);
    while(WiFi.status() == WL_CONNECTED) {
      doYIELD;
      LOG("[WiFi] waiting for disconnect, status: %d", WiFi.status());
      delay(100);
    }
  }
  WiFi.mode(WIFI_MODE_NULL);

  // start WiFi in STA mode
  LOG("[WiFi] setting WiFi mode to STA");
  WiFi.mode(WIFI_MODE_STA);
  LOG("[WiFi] adding event handler");
  WiFi.removeEvent(WiFiEvent);
  WiFi.onEvent(WiFiEvent);
  if(WiFi.STA.connected()) {
    LOG("[WiFi] Already connected");
    WiFi.STA.disconnect(true); // disconnect and erase old config
  }
  WiFi.STA.begin();
  LOG("[WiFi] MAC: %s", WiFi.macAddress().c_str());
  LOG("[WiFi] IP Mode configured: %s%s%s",
      (cfg.ip_mode & IPV4_DHCP) ? "IPv4 DHCP " : "",
      (cfg.ip_mode & IPV4_STATIC) ? "IPv4 STATIC " : "",
      (cfg.ip_mode & IPV6_SLAAC) ? "IPv6 DHCP " : "");
  LOG("[WiFi] SSID: %s", cfg.wifi_ssid);
  if(strlen(cfg.wifi_pass) == 0) {
    LOG("[WiFi] Pass: none");
  } else {
    // print password as stars, even fake the length
    D("[WiFi] Pass: %s", cfg.wifi_pass);
    LOG("[WiFi] Pass: ******");
  }
  // are we connecting to WiFi?
  if(strlen(cfg.wifi_ssid) == 0) {
    LOG("[WiFi] No SSID configured, skipping WiFi setup");
    WiFi.mode(WIFI_MODE_NULL);
    return;
  }
  if(WiFi.status() == WL_CONNECTED) {
    LOG("[WiFi] Already connected, skipping WiFi setup");
    return;
  }

  LOG("[WiFi] setting up WiFi");

  // IPv6 configuration, before WiFi.begin() and WiFi.config()
  if(cfg.ip_mode & IPV6_SLAAC) {
    LOG("[WiFi] Using DHCP for IPv6");
    if(WiFi.enableIPv6(true)) {
      LOG("[WiFi] IPv6 enabled");
    } else {
      LOGE("[WiFi] Failed to enable IPv6");
    }
  } else {
    LOG("[WiFi] Not using IPv6");
    WiFi.enableIPv6(false);
  }

  // IPv4 configuration
  if(cfg.ip_mode & IPV4_DHCP) {
    LOG("[WiFi] Using DHCP for IPv4");
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  } else if(cfg.ip_mode & IPV4_STATIC) {
    LOG("[WiFi] Using static IPv4 configuration");
    WiFi.config(IPAddress(cfg.ipv4_addr[0], cfg.ipv4_addr[1], cfg.ipv4_addr[2], cfg.ipv4_addr[3]),
                IPAddress(cfg.ipv4_gw[0], cfg.ipv4_gw[1], cfg.ipv4_gw[2], cfg.ipv4_gw[3]),
                IPAddress(cfg.ipv4_mask[0], cfg.ipv4_mask[1], cfg.ipv4_mask[2], cfg.ipv4_mask[3]),
                IPAddress(cfg.ipv4_dns[0], cfg.ipv4_dns[1], cfg.ipv4_dns[2], cfg.ipv4_dns[3]));
  } else {
    LOG("[WiFi] Using no IPv4 configuration, assume loopback address");
    WiFi.config(
      IPAddress(127,0,0,1),
      IPAddress(255,255,255,0),
      IPAddress(127,0,0,1),
      IPAddress(127,0,0,1));
  }

  WiFi.mode(WIFI_MODE_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  if(cfg.hostname) {
    WiFi.setHostname(cfg.hostname);
  } else {
    WiFi.setHostname(DEFAULT_HOSTNAME);
  }

  // These need to be called before WiFi.begin()!
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK); // require WPA2
  WiFi.setScanMethod(WIFI_FAST_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

  // set country code if needed
  wifi_country_t country = {0};
  country.schan  = 1;
  country.nchan  = 13;
  country.policy = WIFI_COUNTRY_POLICY_AUTO;
  country.cc[0] = 'B';
  country.cc[1] = 'E';
  esp_err_t e;
  e = esp_wifi_set_country((const wifi_country_t *)&country);
  if(e == ESP_OK) {
    LOG("[WiFi] Country code set to BE");
  } else {
    LOGE("[WiFi] Failed to set country code");
  }
  char cc[4] = {0};
  e = esp_wifi_get_country_code((char *)&cc);
  if(e == ESP_OK) {
    LOG("[WiFi] Country code: %s", cc);
  } else {
    LOGE("[WiFi] Failed to get country code");
  }

  // Lower power to save battery and reduce interference, mostly reflections
  // due to bad antenna design?
  // See https://forum.arduino.cc/t/no-wifi-connect-with-esp32-c3-super-mini/1324046/12
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  // Get Tx power, map the enum properly
  uint8_t txp = WiFi.getTxPower();
  switch(txp) {
    case WIFI_POWER_19_5dBm: txp=19; break; // 78,// 19.5dBm
    case WIFI_POWER_19dBm: txp=19; break; // 76,// 19dBm
    case WIFI_POWER_18_5dBm: txp=18; break; // 72,// 18.5dBm
    case WIFI_POWER_17dBm: txp=17; break; // 68,// 17dBm
    case WIFI_POWER_15dBm: txp=15; break; // 60,// 15dBm
    case WIFI_POWER_13dBm: txp=13; break; // 52,// 13dBm
    case WIFI_POWER_11dBm: txp=11; break; // 44,// 11dBm
    case WIFI_POWER_8_5dBm: txp=8; break; // 34,// 8.5dBm
    case WIFI_POWER_7dBm: txp=7; break; // 30,// 7dBm
    case WIFI_POWER_5dBm: txp=5; break; // 22,// 5dBm
    case WIFI_POWER_2dBm: txp=2; break; // 14,// 2dBm
    case WIFI_POWER_MINUS_1dBm: txp=-1; break; // 6,// -1dBm
    default: txp = 0; break;
  }
  LOG("[WiFi] Tx Power set to %d dBm", txp);

  // after WiFi.config()!
  LOG("[WiFi] Connecting to %s", cfg.wifi_ssid);
  uint8_t ok = 0;
  if(strlen(cfg.wifi_pass) == 0) {
    LOG("[WiFi] No password, connecting to open network");
    ok = WiFi.begin(cfg.wifi_ssid, NULL, 0, NULL, true);
  } else {
    LOG("[WiFi] Connecting with password");
    ok = WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass, 0, NULL, true);
  }
  if(ok != WL_CONNECTED && ok != WL_IDLE_STATUS) {
    LOG("[WiFi] waiting for connection");
  } else {
    LOG("[WiFi] connected");
  }

  // lower the WiFi power save mode if enabled
  if(cfg.wifi_enabled == 1) {
    esp_err_t err = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    if(err != ESP_OK) {
      LOG("[WiFi] WiFi power save mode set failed: %s", esp_err_to_name(err));
    } else {
      LOG("[WiFi] WiFi power save mode set to MIN_MODEM");
    }
  }

  // setup NTP sync if needed
  #if defined(SUPPORT_WIFI) && defined(SUPPORT_NTP)
  setup_ntp();
  #endif
}
#endif // SUPPORT_WIFI

#ifdef SUPPORT_WIFI
NOINLINE
void stop_networking() {
  LOG("[WiFi] Stop networking");
  // first stop WiFi
  if(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS)
    WiFi.disconnect(true);
  while(WiFi.status() == WL_CONNECTED) {
    doYIELD;
    LOG("[WiFi] waiting for disconnect, status: %d", WiFi.status());
    delay(100);
  }
  WiFi.mode(WIFI_MODE_NULL);
  // Ensure WiFi is truly off on ESP32
  wifi_mode_t current_mode;
  esp_err_t err = esp_wifi_get_mode(&current_mode);
  if(err == ESP_OK) {
    // WiFi is initialized, stop it properly
    err = esp_wifi_stop();
    if(err == ESP_OK) {
      LOG("[WiFi] WiFi stopped successfully");
    } else {
      LOG("[WiFi] WiFi stop failed: 0x%x", err);
    }
    err = esp_wifi_deinit();
    if(err == ESP_OK) {
      LOG("[WiFi] WiFi deinitialized successfully");
    } else {
      LOG("[WiFi] WiFi deinit failed: 0x%x", err);
    }
  } else {
    LOG("[WiFi] WiFi was not initialized, skipping stop/deinit");
  }
  LOG("[WiFi] Stop networking done");
}

NOINLINE
void reset_networking() {
  if(!cfg.wifi_enabled) {
    LOG("[WiFi] WiFi is disabled, skipping networking reset");
    return;
  }
  if(strlen(cfg.wifi_ssid) != 0) {
    LOG("[WiFi] resetting networking, SSID: %s", cfg.wifi_ssid);
  } else {
    LOG("[WiFi] resetting networking, no SSID configured");
    stop_networking();
    return;
  }
  #if defined(SUPPORT_WIFI) && defined(WIFI_WPS)
  if(wps_running) {
      LOG("[WiFi] WPS is running, cannot reset networking now");
      return;
  }
  #endif // SUPPORT_WIFI && WIFI_WPS
  LOG("[WiFi] reset networking");
  // first stop WiFi
  stop_networking();
  // start networking
  setup_wifi();
  LOG("[WiFi] reset networking done");
}
#endif // SUPPORT_WIFI

#ifdef SUPPORT_WIFI
NOINLINE
void reconfigure_network_connections() {
  // Check if WiFi is enabled
  if(!cfg.wifi_enabled) {
    LOG("[WiFi] WiFi is disabled, skipping network connections");
    return;
  }

  LOG("[WiFi] network connections, wifi status: %s", (WiFi.status() == WL_CONNECTED) ? "connected" : "not connected");
  if(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS) {
    // tcp - attempt both IPv4 and IPv6 connections based on target and available addresses
    #ifdef SUPPORT_TCP
    connect_tcp();
    #endif // SUPPORT_TCP

    #ifdef SUPPORT_TLS
    connect_tls();
    #endif // SUPPORT_TLS

    #if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
    stop_tcp_servers();
    start_tcp_servers();
    #endif
  } else {
    #if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
    // WiFi not connected, stop TCP servers
    stop_tcp_servers();
    #endif
  }

  if(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS) {
    #ifdef SUPPORT_UDP
    LOG("[UDP] setting up sockets");
    // udp - attempt both IPv4 and IPv6 connections based on target and available addresses

    // in/out udp receive/send socket
    if(strlen(cfg.udp_host_ip) >= 0 || cfg.udp_port != 0) {
        in_out_socket_udp(udp_sock);
        if(udp_sock >= 0){
            LOG("[UDP] IPv4/IPv6 UDP socket set up on port %hu to host %s", cfg.udp_port, cfg.udp_host_ip);
        }
    } else {
        udp_sock = -1;
        LOG("[UDP] IPv4/IPv6 UDP socket disabled");
    }

    // receive-only udp socket (IPv4 only)
    if(cfg.udp_listen_port != 0){
        in_socket_udp(udp_listen_sock, cfg.udp_listen_port?cfg.udp_listen_port:cfg.udp6_listen_port);
        if(udp_listen_sock >= 0){
            LOG("[UDP_LISTEN] IPv4 UDP listening socket set up on port %hu", cfg.udp_listen_port);
        }
    } else {
        udp_listen_sock = -1;
        LOG("[UDP_LISTEN] IPv4 UDP listening socket disabled");
    }

    // receive-only udp socket (IPv6 only)
    if(cfg.udp6_listen_port != 0){
        in_socket_udp6(udp6_listen_sock, cfg.udp6_listen_port?cfg.udp6_listen_port:cfg.udp_listen_port);
        if(udp6_listen_sock >= 0){
            LOG("[UDP6_LISTEN] IPv6 UDP listening socket set up on port %hu", cfg.udp6_listen_port);
        }
    } else {
        udp6_listen_sock = -1;
        LOG("[UDP6_LISTEN] IPv6 UDP listening socket disabled");
    }

    // send-only udp socket
    if(strlen(cfg.udp_send_ip) >= 0 || cfg.udp_send_port != 0){
        out_socket_udp(udp_out_sock, cfg.udp_send_port, cfg.udp_send_ip);
        if(udp_out_sock >= 0){
            LOG("[UDP_SEND] IPv4/IPv6 UDP send socket set up to %s:%hu", cfg.udp_send_ip, cfg.udp_send_port);
        }
    } else {
        udp_out_sock = -1;
        LOG("[UDP_SEND] IPv4/IPv6 UDP send socket disabled");
    }
    #endif // SUPPORT_UDP
  }
  return;
}

void stop_network_connections() {
  LOG("[WiFi] stop network connections");

  errno = 0;

  #ifdef SUPPORT_TCP
  close_tcp_socket();
  #endif // SUPPORT_TCP

  #ifdef SUPPORT_TLS
  close_tls_connection();
  #endif // SUPPORT_TLS

  #ifdef SUPPORT_UDP
  close_udp_socket(udp_sock, "[UDP]");
  close_udp_socket(udp_listen_sock, "[UDP_LISTEN]");
  close_udp_socket(udp6_listen_sock, "[UDP6_LISTEN]");
  close_udp_socket(udp_out_sock, "[UDP_SEND]");
  #endif // SUPPORT_UDP

  #if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
  stop_tcp_servers();
  #endif // SUPPORT_WIFI && SUPPORT_TCP_SERVER

  LOG("[WiFi] stop network connections done");
}
#else // !SUPPORT_WIFI
void reconfigure_network_connections() {
  // WiFi not supported, no network connections to configure
}

void stop_network_connections() {
  // WiFi not supported, no network connections to stop
}
#endif // SUPPORT_WIFI


#if defined(SUPPORT_WIFI) && (defined(SUPPORT_TCP) || defined(SUPPORT_UDP))
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <lwip/netdb.h>

// Helper: check if string is IPv6
NOINLINE
bool is_ipv6_addr(const char* ip) {
  if(ip == NULL)
    return false;
  return strchr(ip, ':') != NULL;
}

#endif // SUPPORT_WIFI && (SUPPORT_UDP || SUPPORT_TCP)

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP)
FD tcp_sock = -1;
long last_tcp_check = 0;
uint8_t tcp_connection_writable = 0;
#endif // SUPPORT_WIFI && SUPPORT_TCP

#ifdef SUPPORT_TLS
WiFiClientSecure tls_client = NULL;
uint8_t tls_connected = 0;
uint8_t tls_handshake_complete = 0;
long last_tls_check = 0;
#endif // SUPPORT_TLS

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP)

void connect_tcp() {
  if(strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0) {
    D("[TCP] Invalid TCP host IP or port, not setting up TCP");
    return;
  }
  if(tcp_sock >= 0)
    close_tcp_socket();

  // just as a test/debug, leave at 0
  uint8_t blocking_connect = 0;

  int r = 0;
  if(is_ipv6_addr(cfg.tcp_host_ip)) {
    // IPv6
    LOG("[TCP] setting up TCP/ipv6 to:%s, port:%hu", cfg.tcp_host_ip, cfg.tcp_port);
    struct sockaddr_in6 sa6;
    memset(&sa6, 0, sizeof(sa6));
    sa6.sin6_family = AF_INET6;
    sa6.sin6_port = htons(cfg.tcp_port);
    if (inet_pton(AF_INET6, cfg.tcp_host_ip, &sa6.sin6_addr) != 1) {
      LOG("[TCP] Invalid IPv6 address for TCP: %s", cfg.tcp_host_ip);
      return;
    }

    // socket
    tcp_sock = socket(AF_INET6, SOCK_STREAM, 0);
    if (tcp_sock == -1) {
      LOGE("[TCP] Failed to create IPv6 TCP socket");
      return;
    }

    // Set socket options
    int optval = 1;
    if (setsockopt(tcp_sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
      LOGE("[TCP] Failed to set SO_REUSEADDR");
      close(tcp_sock);
      tcp_sock = -1;
      return;
    }

    // Configure for IPv6-only (no IPv4 mapping)
    if (setsockopt(tcp_sock, IPPROTO_IPV6, IPV6_V6ONLY, &optval, sizeof(optval)) < 0) {
      LOGE("[TCP] Failed to set IPV6_V6ONLY");
      close(tcp_sock);
      tcp_sock = -1;
      return;
    }

    // set socket to non-blocking mode and read/write
    int flags = fcntl(tcp_sock, F_GETFL, 0);
    if (blocking_connect == 0)
      flags |= O_NONBLOCK;
    if (flags >= 0)
      fcntl(tcp_sock, F_SETFL, flags | O_RDWR);

    // connect
    r = connect(tcp_sock, (struct sockaddr*)&sa6, sizeof(sa6));

  } else {
    // IPv4
    LOG("[TCP] setting up TCP/ipv4 to: %s, port:%hu", cfg.tcp_host_ip, cfg.tcp_port);
    struct sockaddr_in sa4 = {0};
    sa4.sin_family = AF_INET;
    sa4.sin_port = htons(cfg.tcp_port);
    if (inet_pton(AF_INET, cfg.tcp_host_ip, &sa4.sin_addr) != 1) {
      LOG("[TCP] Invalid IPv4 address for TCP: %s", cfg.tcp_host_ip);
      return;
    }

    // socket
    tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_sock < 0) {
      LOGE("[TCP] Failed to create IPv4 TCP socket");
      return;
    }

    // set socket to non-blocking mode and read/write
    int flags = fcntl(tcp_sock, F_GETFL, 0);
    if (blocking_connect == 0)
      flags |= O_NONBLOCK;
    if (flags >= 0)
      fcntl(tcp_sock, F_SETFL, flags | O_RDWR);

    // connect
    r = connect(tcp_sock, (struct sockaddr*)&sa4, sizeof(sa4));
  }

  // connect, this will be non-blocking, so we get a EINPROGRESS
  if (r == -1) {
    if(errno && errno != EINPROGRESS) {
      // If not EINPROGRESS, connection failed
      LOGE("[TCP] Failed to connect IPv6 TCP socket");
      close_tcp_socket();
      return;
    }
    uint8_t old_errno = errno;
    errno = 0; // clear errno after EINPROGRESS
    int optval, r_o, s_bufsize, r_bufsize;
    socklen_t optlen = sizeof(optval);
    optval = 1;
    r_o = setsockopt(tcp_sock, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen);
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP KEEPALIVE");
    optval = 1;
    r_o = setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPIDLE, &optval, optlen);
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP KEEPIDLE");
    optval = 1;
    r_o = setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPINTVL, &optval, optlen);
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP KEEPINTVL");
    optval = 1;
    r_o = setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPCNT, &optval, optlen);
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP KEEPCNT");
    // set receive buffer size
    r_bufsize = 512;
    r_o = setsockopt(tcp_sock, SOL_SOCKET, SO_RCVBUF, &r_bufsize, sizeof(r_bufsize));
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP SO_RCVBUF");
    r_o = getsockopt(tcp_sock, SOL_SOCKET, SO_RCVBUF, &r_bufsize, &optlen);
    if (r_o < 0) {
      LOGE("[TCP] Failed to get TCP SO_RCVBUF");
    } else {
      D("[TCP] TCP NEW SO_RCVBUF: %d", r_bufsize);
    }
    // disable Nagle's algorithm, send data immediately
    optval = 1;
    r_o = setsockopt(tcp_sock, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval));
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP NODELAY");
    // set recv/send timeout to 1 second
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    r_o = setsockopt(tcp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP RCVTIMEO");
    optval = 1000; // milliseconds
    r_o = setsockopt(tcp_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP SNDTIMEO");
    errno = 0; // clear errno
    errno = old_errno; // restore old errno
    LOGE("[TCP] connection on fd:%d initiated to: %s, port:%hu", tcp_sock, cfg.tcp_host_ip, cfg.tcp_port);
    int flags = fcntl(tcp_sock, F_GETFL, 0);
    flags |= O_RDWR;
    flags |= O_NONBLOCK;
    if (flags >= 0)
      fcntl(tcp_sock, F_SETFL, flags);
    LOG("[TCP] connection in progress on fd:%d to %s, port:%hu", tcp_sock, cfg.tcp_host_ip, cfg.tcp_port);

    if(is_ipv6_addr(cfg.tcp_host_ip)) {
      // for debug, get local and peer address
      #ifdef DEBUG
      struct sockaddr_in6 l_sa6 = {0};
      socklen_t optlen = sizeof(l_sa6);
      if(getsockname(tcp_sock, (struct sockaddr*)&l_sa6, &optlen) == 0) {
        char local_addr_str[40] = {0};
        if(inet_ntop(AF_INET6, &l_sa6.sin6_addr, local_addr_str, sizeof(local_addr_str))) {
          D("[TCP] TCP local address: %s, port:%hu", local_addr_str, ntohs(l_sa6.sin6_port));
        }
      } else {
        E("[TCP] Failed to get local IPv6 TCP address");
      }
      struct sockaddr_in6 r_sa6;
      memset(&r_sa6, 0, sizeof(r_sa6));
      if(getpeername(tcp_sock, (struct sockaddr*)&r_sa6, &optlen) == 0) {
        char peer_addr_str[40] = {0};
        if(inet_ntop(AF_INET6, &r_sa6.sin6_addr, peer_addr_str, sizeof(peer_addr_str))) {
          D("[TCP] TCP peer address: %s, port:%hu", peer_addr_str, ntohs(r_sa6.sin6_port));
        }
      } else {
        E("[TCP] Failed to get peer IPv6 TCP address");
      }
      #endif // DEBUG
    } else {

      // for debug, get local and peer address
      #ifdef DEBUG
      struct sockaddr_in l_sa4;
      memset(&l_sa4, 0, sizeof(l_sa4));
      socklen_t optlen = sizeof(l_sa4);
      if(getsockname(tcp_sock, (struct sockaddr*)&l_sa4, &optlen) == 0) {
        char local_addr_str[16] = {0};
        if(inet_ntop(AF_INET, &l_sa4.sin_addr, local_addr_str, sizeof(local_addr_str))) {
          D("[TCP] TCP local address: %s, port:%hu", local_addr_str, ntohs(l_sa4.sin_port));
        }
      } else {
        E("[TCP] Failed to get local IPv4 TCP address");
      }
      struct sockaddr_in r_sa4;
      memset(&r_sa4, 0, sizeof(r_sa4));
      optlen = sizeof(r_sa4);
      if(getpeername(tcp_sock, (struct sockaddr*)&r_sa4, &optlen) == 0) {
        char peer_addr_str[16] = {0};
        if(inet_ntop(AF_INET, &r_sa4.sin_addr, peer_addr_str, sizeof(peer_addr_str))) {
          D("[TCP] TCP peer address: %s, port:%hu", peer_addr_str, ntohs(r_sa4.sin_port));
        }
      } else {
        E("[TCP] Failed to get peer IPv4 TCP address");
      }
      #endif // DEBUG

    }
    return;
  }
  LOG("[TCP] connected fd:%d to %s, port:%hu", tcp_sock, cfg.tcp_host_ip, cfg.tcp_port);
}


NOINLINE
void close_tcp_socket() {
  FD fd_orig = tcp_sock;
  if (tcp_sock >= 0) {
    D("[TCP] closing socket %d", fd_orig);
    errno = 0;
    if (shutdown(tcp_sock, SHUT_RDWR) == -1) {
        if (errno && errno != ENOTCONN && errno != EBADF && errno != EINVAL)
            LOGE("[TCP] Failed to shutdown %d socket", fd_orig);
    }
    D("[TCP] socket %d shutdown", fd_orig);
    errno = 0;
    // now close the socket
    if (close(tcp_sock) == -1)
        if (errno && errno != EBADF && errno != ENOTCONN)
            LOGE("[TCP] Failed to close %d socket", fd_orig);
    tcp_sock = -1;
    LOG("[TCP] socket %d closed", fd_orig);
  }
}

// Helper: send TCP data (IPv4/IPv6)
int send_tcp_data(const uint8_t* data, size_t len) {
  D("[TCP] send_tcp_data len: %d", len);
  int result = send(tcp_sock, data, len, 0);
  if (result == -1 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINPROGRESS)) {
    // Would block, try again later
    return -1;
  }
  if (result == -1) {
    // Error occurred, close the socket and mark as invalid
    LOGE("[TCP] send error on socket %d: %d (%s)", tcp_sock, errno, COMMON::get_errno_string(errno));
    close_tcp_socket();
    return -1;
  }
  if (result == 0) {
    // Connection closed by the remote host
    LOG("[TCP] connection closed by remote host on socket %d", tcp_sock);
    close_tcp_socket();
    return 0;
  }
  doYIELD;
  return result;
}

// Helper: receive TCP data (IPv4/IPv6)
int recv_tcp_data(uint8_t* buf, size_t maxlen) {
  // IPv6 socket (non-blocking)
  int result = recv(tcp_sock, buf, maxlen, 0);
  if (result == -1 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINPROGRESS)) {
    // No data available right now
    return -1;
  }
  if (result == -1) {
    LOG("[TCP] recv error on socket %d: %d (%s)", tcp_sock, errno, COMMON::get_errno_string(errno));
    close_tcp_socket(); // Error occurred, close the socket and mark as invalid
    return -1;
  }
  if (result == 0) {
    LOG("[TCP] connection closed by remote host on socket %d", tcp_sock);
    close_tcp_socket(); // Connection closed by the remote host
    return 0;
  }
  return result;
}

// TCP Connection Check: Verify if TCP connection is still alive
int check_tcp_connection(unsigned int tm = 0) {
  if (strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0) {
    // No TCP host configured
    return 0;
  }

  if (tcp_sock == -1) {
    // No valid TCP host or connection needs to be established
    D("[TCP] No valid TCP host, attempting to establish connection");
    return 0;
  }

  // IPv6 socket: use select() to check if socket is ready for read/write
  fd_set readfds, writefds, errorfds;

  FD_ZERO(&readfds);
  FD_ZERO(&writefds);
  FD_ZERO(&errorfds);
  FD_SET(tcp_sock, &readfds);
  FD_SET(tcp_sock, &writefds);
  FD_SET(tcp_sock, &errorfds);

  // non-blocking select with 0 timeout
  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = tm;

  int ready = select(tcp_sock + 1, &readfds, &writefds, &errorfds, &timeout);
  if (ready < 0) {
    LOGE("[TCP] select error");
    close_tcp_socket();
    return 0;
  }

  if (FD_ISSET(tcp_sock, &errorfds)) {
    LOG("[TCP] socket %d has error, checking error", tcp_sock);

    // Check if socket is connected by trying to get socket error
    int socket_error = 0;
    socklen_t len = sizeof(socket_error);
    if (getsockopt(tcp_sock, SOL_SOCKET, SO_ERROR, &socket_error, &len) == 0) {
      if (socket_error != 0) {
        LOG("[TCP] socket %d error detected: %d (%s), reconnecting", tcp_sock, socket_error, COMMON::get_errno_string(socket_error));
        close_tcp_socket();
        return 0;
      }
    } else {
      LOGE("[TCP] getsockopt failed on socket %d", tcp_sock);
    }
    LOG("[TCP] socket %d error but no error detected, assuming connection OK", tcp_sock);
    close_tcp_socket();
    return 0;
  }

  #ifdef DEBUG
  if (FD_ISSET(tcp_sock, &writefds)) {
    tcp_connection_writable = 1;
    D("[TCP] socket fd:%d writable, connection OK", tcp_sock);
  } else {
    tcp_connection_writable = 0;
    D("[TCP] socket fd:%d not yet writable", tcp_sock);
  }
  #endif
  return 1;
}
#endif // SUPPORT_WIFI && SUPPORT_TCP

#ifdef SUPPORT_TLS

// TLS/SSL Connection Management Functions

// Load TLS certificate/key from SPIFFS file
bool load_tls_file_from_spiffs(const char* filename, uint8_t* buffer, size_t buffer_size) {
  if (!SPIFFS.begin(true)) {
    LOG("[SPIFFS] Failed to initialize SPIFFS");
    return false;
  }

  if (!SPIFFS.exists(filename)) {
    LOG("[SPIFFS] File %s does not exist", filename);
    return false;
  }

  File file = SPIFFS.open(filename, "r");
  if (!file) {
    LOG("[SPIFFS] Failed to open file %s", filename);
    return false;
  }

  size_t file_size = file.size();
  if (file_size >= buffer_size) {
    LOG("[SPIFFS] File %s too large (%d bytes, max %d)", filename, file_size, buffer_size - 1);
    file.close();
    return false;
  }

  memset(buffer, 0, buffer_size);
  size_t bytes_read = file.readBytes((char *)buffer, file_size);
  file.close();

  if (bytes_read != file_size) {
    LOG("[SPIFFS] Failed to read complete file %s (%d/%d bytes)", filename, bytes_read, file_size);
    return false;
  }

  buffer[bytes_read] = '\0'; // Ensure null termination
  LOG("[SPIFFS] Loaded %s (%d bytes)", filename, bytes_read);
  return true;
}

// Save TLS certificate/key to SPIFFS file
bool save_tls_file_to_spiffs(const char* filename, const char* content) {
  if (!SPIFFS.begin(true)) {
    LOG("[SPIFFS] Failed to initialize SPIFFS");
    return false;
  }

  File file = SPIFFS.open(filename, "w");
  if (!file) {
    LOG("[SPIFFS] Failed to create file %s", filename);
    return false;
  }

  size_t content_len = strlen(content);
  size_t bytes_written = file.write((const uint8_t*)content, content_len);
  file.close();

  if (bytes_written != content_len) {
    LOG("[SPIFFS] Failed to write complete file %s (%d/%d bytes)", filename, bytes_written, content_len);
    return false;
  }
  LOG("[SPIFFS] Saved %s (%d bytes)", filename, bytes_written);
  return true;
}

void connect_tls() {
  if(!cfg.tls_enabled) {
    D("[TLS] TLS is disabled");
    return;
  }
  if(strlen(cfg.tcp_host_ip) == 0 || (cfg.tcp_port == 0 && cfg.tls_port == 0)) {
    D("[TLS] Invalid TLS host IP or port, not setting up TLS");
    return;
  }

  uint16_t port_to_use = cfg.tls_port ? cfg.tls_port : cfg.tcp_port;
  LOG("[TLS] Setting up TLS connection to: %s, port: %d", cfg.tcp_host_ip, port_to_use);

  // Close existing connection if any
  if(tls_connected) {
    close_tls_connection();
  }

  // Configure TLS client
  tls_client = WiFiClientSecure();
  uint8_t tls_buf[2048] = {0};
  load_tls_file_from_spiffs("/ca_cert.pem", tls_buf, sizeof(tls_buf));
  if(strlen((const char *)tls_buf) > 0) {
    tls_client.setCACert((const char *)tls_buf);
    LOG("[TLS] CA certificate configured");
  } else {
    tls_client.setInsecure(); // Skip certificate verification
    LOG("[TLS] Using insecure mode (no certificate verification)");
  }

  load_tls_file_from_spiffs("/client_cert.pem", tls_buf, sizeof(tls_buf));
  if(strlen((const char *)tls_buf) > 0) {
    tls_client.setCertificate((const char *)tls_buf);
    LOG("[TLS] Client cert configured");
    load_tls_file_from_spiffs("/client_cert.key", tls_buf, sizeof(tls_buf));
    if(strlen((const char *)tls_buf) > 0) {
      tls_client.setPrivateKey((const char *)tls_buf);
      LOG("[TLS] Client key configured");
    }
  }

  // Connect
  if(tls_client.connect(cfg.tcp_host_ip, port_to_use)) {
    tls_connected = 1;
    tls_handshake_complete = 1;
    LOG("[TLS] TLS Connected successfully to %s:%d", cfg.tcp_host_ip, port_to_use);
  } else {
    tls_connected = 0;
    tls_handshake_complete = 0;
    LOG("[TLS] Failed to connect to %s:%d", cfg.tcp_host_ip, port_to_use);
  }
}

void close_tls_connection() {
  if(!cfg.tls_enabled || strlen(cfg.tcp_host_ip) == 0 || (cfg.tcp_port == 0 && cfg.tls_port == 0) || tls_client == NULL)
    return;
  if(tls_connected || tls_handshake_complete) {
    tls_client.stop();
    tls_connected = 0;
    tls_handshake_complete = 0;
    tls_client = NULL;
    LOG("[TLS] Connection closed");
  }
}

// Helper: send TLS data
int send_tls_data(const uint8_t* data, size_t len) {
  if(!tls_connected || !tls_handshake_complete || tls_client == NULL) {
    return -1;
  }

  size_t written = tls_client.write(data, len);
  if(written != len) {
    LOG("[TLS] Send incomplete: %d/%d bytes", written, len);
    if(written == 0) {
      // Connection might be closed
      if(!tls_client.connected()) {
        LOG("[TLS] Connection lost during send");
        close_tls_connection();
        return -1;
      }
    }
  }
  return written;
}

// Helper: receive TLS data
int recv_tls_data(uint8_t* buf, size_t maxlen) {
  if(!tls_connected || !tls_handshake_complete || tls_client == NULL) {
    return -1;
  }

  if(!tls_client.available()) {
    return -1; // No data available
  }

  int result = tls_client.read(buf, maxlen);
  if(result <= 0) {
    // Check if connection is still alive
    if(!tls_client.connected()) {
      LOG("[TLS] Connection lost during receive");
      close_tls_connection();
      return -1;
    }
  }
  return result;
}

// TLS Connection Check: Verify if TLS connection is still alive
int check_tls_connection() {
  if(!cfg.tls_enabled || strlen(cfg.tcp_host_ip) == 0 || (cfg.tcp_port == 0 && cfg.tls_port == 0) || tls_client == NULL) {
    return 0;
  }

  if(!tls_connected) {
    D("[TLS] No TLS connection, attempting to establish");
    return 0;
  }

  if(!tls_client.connected()) {
    LOG("[TLS] Connection lost, closing");
    close_tls_connection();
    return 0;
  }

  return 1;
}

#endif // SUPPORT_TLS

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)

// support up to 8 clients
#define TCP_CLIENTS_MAX 8
FD tcp_server_clients[TCP_CLIENTS_MAX] = {-1, -1, -1, -1, -1, -1, -1, -1};

// TCP Server functions
void start_tcp_server() {
  if(cfg.tcp_server_port == 0 && cfg.tcp6_server_port == 0) {
    D("[TCP_SERVER] TCP server port not configured");
    return;
  }

  uint16_t port_to_use = cfg.tcp_server_port ? cfg.tcp_server_port : cfg.tcp6_server_port;
  if(tcp_server_sock != -1) {
    LOG("[TCP_SERVER] TCP server already running on port %hu", port_to_use);
    return;
  }
  LOG("[TCP_SERVER] Starting TCP server on port %hu", port_to_use);

  // Create socket (supports both IPv4 and IPv6)
  tcp_server_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_server_sock == -1) {
    LOGE("[TCP_SERVER] Failed to create server socket");
    return;
  }

  // Set socket options
  int optval = 1;
  if (setsockopt(tcp_server_sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
    LOGE("[TCP_SERVER] Failed to set SO_REUSEADDR");
    close(tcp_server_sock);
    tcp_server_sock = -1;
    return;
  }

  // Set non-blocking
  int flags = fcntl(tcp_server_sock, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(tcp_server_sock, F_SETFL, flags | O_NONBLOCK);
  }

  // Bind to port
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr = in_addr{.s_addr = INADDR_ANY};
  server_addr.sin_port = htons(port_to_use);

  if (bind(tcp_server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    LOGE("[TCP_SERVER] Failed to bind to port %hu", port_to_use);
    close(tcp_server_sock);
    tcp_server_sock = -1;
    return;
  }

  // Start listening
  if (listen(tcp_server_sock, cfg.tcp_server_max_clients) < 0) {
    LOGE("[TCP_SERVER] Failed to listen on port %hu", port_to_use);
    close(tcp_server_sock);
    tcp_server_sock = -1;
    return;
  }

  LOG("[TCP_SERVER] TCP server started successfully on port %hu", port_to_use);
}

void stop_tcp_server() {
  if(tcp_server_sock == -1)
    return;
  LOG("[TCP_SERVER] Stopping TCP server on port %hu", cfg.tcp_server_port);
  close(tcp_server_sock);
  tcp_server_sock = -1;
}

void handle_tcp_server() {
  if(tcp_server_sock == -1)
    return;

  // Accept new connections
  struct sockaddr_in client_addr;
  socklen_t client_len = sizeof(client_addr);
  int new_client = accept(tcp_server_sock, (struct sockaddr*)&client_addr, &client_len);
  if(new_client >= 0) {
    // Find empty slot for new client
    int8_t slot = -1;
    for(uint8_t i = 0; i < cfg.tcp_server_max_clients && i < TCP_CLIENTS_MAX; i++) {
      if(tcp_server_clients[i] == -1) {
        slot = i;
        break;
      }
    }

    if(slot >= 0) {
      // Set client socket to non-blocking
      int flags = fcntl(new_client, F_GETFL, 0);
      if (flags >= 0) {
        fcntl(new_client, F_SETFL, flags | O_NONBLOCK);
      }

      // Log client connection
      tcp_server_clients[slot] = new_client;
      char client_ip[INET_ADDRSTRLEN] = {0};
      inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
      LOG("[TCP_SERVER] New client connected from [%s]:%hu in slot %d, fd:%d",
        client_ip, ntohs(client_addr.sin_port), slot, new_client);

    } else {
      // No available slots, reject connection
      LOG("[TCP_SERVER] Connection rejected - server full");
      close(new_client);
    }
  }
}

// Send data to all connected TCP server clients
int send_tcp_server_data(const uint8_t* data, size_t len) {
  uint8_t clients_sent = 0;
  for(uint8_t i = 0; i < cfg.tcp_server_max_clients && i < TCP_CLIENTS_MAX; i++) {
    if(tcp_server_clients[i] == -1)
      continue;
    errno = 0;
    int result = send(tcp_server_clients[i], data, len, 0);
    if(result > 0) {
      clients_sent++;
    } else if (result == -1) {
      if(errno != EAGAIN && errno != EWOULDBLOCK) {
        // Client connection error
        LOG("[TCP_SERVER] Error sending to client %d fd:%d, disconnecting", i, tcp_server_clients[i]);
        close(tcp_server_clients[i]);
        tcp_server_clients[i] = -1;
      } else {
        // Would block, try again later
        LOOP_E("[TCP_SERVER] Would block sending to client %d fd:%d, try again later", i, tcp_server_clients[i]);
        continue;
      }
    } else if(result == 0) {
      // Client disconnected
      LOGE("[TCP_SERVER] Client %d fd:%d disconnected [SEND]", i, tcp_server_clients[i]);
      close(tcp_server_clients[i]);
      tcp_server_clients[i] = -1;
    }
  }
  return clients_sent;
}

// Receive data from all connected TCP server clients
int recv_tcp_server_data(uint8_t* buf, size_t maxlen) {
  // TODO: randomize client order to avoid starvation
  for(uint8_t i = 0; i < cfg.tcp_server_max_clients && i < TCP_CLIENTS_MAX; i++) {
    if(tcp_server_clients[i] == -1)
      continue;
    errno = 0;
    int bytes_received = recv(tcp_server_clients[i], buf, maxlen, MSG_DONTWAIT);
    if(bytes_received > 0) {
      return bytes_received; // Return data from the first client that has data
    } else if(bytes_received == -1) {
      if(errno != EAGAIN && errno != EWOULDBLOCK) {
        // Client connection error
        LOG("[TCP_SERVER] Error receiving from client %d fd:%d, disconnecting", i, tcp_server_clients[i]);
        close(tcp_server_clients[i]);
        tcp_server_clients[i] = -1;
      } else {
        // No data available right now
        LOOP_E("[TCP_SERVER] No data available from client %d fd:%d right now", i, tcp_server_clients[i]);
        continue;
      }
    } else if(bytes_received == 0) {
      // Client disconnected
      LOGE("[TCP_SERVER] Client %d fd:%d disconnected [RECV]", i, tcp_server_clients[i]);
      close(tcp_server_clients[i]);
      tcp_server_clients[i] = -1;
    }
  }
  return -1; // No data received
}

// Get number of connected TCP server clients
uint8_t get_tcp_server_client_count() {
  uint8_t count = 0;
  for(uint8_t i = 0; i < cfg.tcp_server_max_clients && i < TCP_CLIENTS_MAX; i++) {
    if(tcp_server_clients[i] != -1)
      count++;
  }
  return count;
}

// TCP6 Server functions (IPv6-only)
void start_tcp6_server() {
  if(cfg.tcp6_server_port == 0 && cfg.tcp_server_port == 0) {
    D("[TCP6_SERVER] TCP6 server port not configured");
    return;
  }

  uint16_t port_to_use = cfg.tcp6_server_port ? cfg.tcp6_server_port : cfg.tcp_server_port;
  if(tcp6_server_sock != -1) {
    LOG("[TCP6_SERVER] TCP6 server already running on port %hu", port_to_use);
    return;
  }
  LOG("[TCP6_SERVER] Starting TCP6 server on port %hu", port_to_use);

  // Create IPv6-only socket
  tcp6_server_sock = socket(AF_INET6, SOCK_STREAM, 0);
  if (tcp6_server_sock == -1) {
    LOGE("[TCP6_SERVER] Failed to create server socket");
    return;
  }

  // Set socket options
  int optval = 1;
  if (setsockopt(tcp6_server_sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
    LOGE("[TCP6_SERVER] Failed to set SO_REUSEADDR");
    close(tcp6_server_sock);
    tcp6_server_sock = -1;
    return;
  }

  // Configure for IPv6-only (no IPv4 mapping)
  if (setsockopt(tcp6_server_sock, IPPROTO_IPV6, IPV6_V6ONLY, &optval, sizeof(optval)) < 0) {
    LOGE("[TCP6_SERVER] Failed to set IPV6_V6ONLY");
    close(tcp6_server_sock);
    tcp6_server_sock = -1;
    return;
  }

  // Set non-blocking
  int flags = fcntl(tcp6_server_sock, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(tcp6_server_sock, F_SETFL, flags | O_NONBLOCK);
  }

  // Bind to port
  struct sockaddr_in6 server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin6_family = AF_INET6;
  server_addr.sin6_addr = in6addr_any;  // Listen on all IPv6 interfaces
  server_addr.sin6_port = htons(port_to_use);

  if (bind(tcp6_server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    LOGE("[TCP6_SERVER] Failed to bind to port %hu", port_to_use);
    close(tcp6_server_sock);
    tcp6_server_sock = -1;
    return;
  }

  // Start listening
  if (listen(tcp6_server_sock, cfg.tcp_server_max_clients) < 0) {
    LOGE("[TCP6_SERVER] Failed to listen on port %hu", port_to_use);
    close(tcp6_server_sock);
    tcp6_server_sock = -1;
    return;
  }

  LOG("[TCP6_SERVER] TCP6 server started successfully on port %hu", port_to_use);
}

void stop_tcp6_server() {
  if(tcp6_server_sock != -1)
    return;
  // Close server socket
  LOG("[TCP6_SERVER] Stopping TCP6 server on port %hu", cfg.tcp6_server_port);
  close(tcp6_server_sock);
  tcp6_server_sock = -1;
}

void handle_tcp6_server() {
  if(tcp6_server_sock == -1)
    return;

  // Accept new connections
  struct sockaddr_in6 client_addr;
  socklen_t client_len = sizeof(client_addr);
  int new_client = accept(tcp6_server_sock, (struct sockaddr*)&client_addr, &client_len);
  if(new_client >= 0) {
    // Find available slot for new client
    int8_t slot = -1;
    for(uint8_t i = 0; i < cfg.tcp_server_max_clients && i < TCP_CLIENTS_MAX; i++) {
      if(tcp_server_clients[i] == -1) {
        slot = i;
        break;
      }
    }

    if(slot >= 0) {
      // Set non-blocking for client socket
      int flags = fcntl(new_client, F_GETFL, 0);
      if (flags >= 0) {
        fcntl(new_client, F_SETFL, flags | O_NONBLOCK);
      }

      // Log client connection
      tcp_server_clients[slot] = new_client;
      char client_ip[INET6_ADDRSTRLEN] = {0};
      inet_ntop(AF_INET6, &client_addr.sin6_addr, client_ip, INET6_ADDRSTRLEN);
      LOG("[TCP6_SERVER] New client connected from [%s]:%hu in slot %d, fd:%d",
        client_ip, ntohs(client_addr.sin6_port), slot, new_client);

      #ifdef LED
      last_activity = millis();
      #endif
    } else {
      LOG("[TCP6_SERVER] No available slots for new client, rejecting");
      close(new_client);
    }
  }
}

void handle_tcp_server_disconnects() {
  // handle disconnects
  for(uint8_t i = 0; i < cfg.tcp_server_max_clients && i < TCP_CLIENTS_MAX; i++) {
    if(tcp_server_clients[i] == -1)
      continue;
    int socket_error = 0;
    socklen_t len = sizeof(socket_error);
    if (getsockopt(tcp_server_clients[i], SOL_SOCKET, SO_ERROR, &socket_error, &len) == 0) {
      if (socket_error != 0) {
        LOG("[TCP_SERVER] socket %d, fd:%d error detected: %d (%s), reconnecting", i, tcp_server_clients[i], socket_error, COMMON::get_errno_string(socket_error));
        close(tcp_server_clients[i]);
        tcp_server_clients[i] = -1;
        continue;
      }
    } else {
      LOGE("[TCP_SERVER] getsockopt failed on socket %d", tcp_server_clients[i]);
    }
  }
}

void start_tcp_servers() {
  // Initialize client slots
  for(uint8_t i = 0; i < TCP_CLIENTS_MAX; i++) {
    if(tcp_server_clients[i] != -1)
      close(tcp_server_clients[i]);
    tcp_server_clients[i] = -1;
  }

  start_tcp_server();
  start_tcp6_server();
}

void stop_tcp_servers() {
  // Close all client connections
  for(uint8_t i = 0; i < TCP_CLIENTS_MAX; i++) {
    if(tcp_server_clients[i] == -1)
      continue;
    close(tcp_server_clients[i]);
    tcp_server_clients[i] = -1;
  }

  stop_tcp_server();
  stop_tcp6_server();
}
#endif // SUPPORT_WIFI && SUPPORT_TCP_SERVER

#if defined(SUPPORT_WIFI) && defined(SUPPORT_UDP)

#define UDP_READ_MSG_SIZE 512

void in_out_socket_udp(FD &fd) {
  if(strlen(cfg.udp_host_ip) == 0 || cfg.udp_port == 0) {
    LOOP_D("[UDP] No valid UDP host IP or port, not setting up UDP");
    close_udp_socket(fd, "[UDP]");
    return;
  }
  // already setup?
  if(fd != -1)
    return;
  char *d_ip = cfg.udp_host_ip;
  int16_t port = cfg.udp_port;
  LOG("[UDP] setting up UDP to:%s, port:%hu", d_ip, port);
  if(is_ipv6_addr(d_ip)) {
    if(!udp_socket(fd, 1, "[UDP]"))
      return;

    // local IPv6
    struct sockaddr_in6 l_sa6;
    memset(&l_sa6, 0, sizeof(l_sa6));
    l_sa6.sin6_family = AF_INET6;
    l_sa6.sin6_port = htons(port);
    l_sa6.sin6_addr = in6addr_any;
    if (bind(fd, (const sockaddr *)&l_sa6, sizeof(l_sa6)) == -1) {
      LOGE("[UDP] Failed to bind UDP socket fd:%d to %s:%hu", fd, d_ip, port);
      close_udp_socket(fd, "[UDP]");
      return;
    }
  } else {
    if(!udp_socket(fd, 0, "[UDP]"))
      return;

    // local IPv4 listen IP
    struct sockaddr_in l_sa4;
    memset(&l_sa4, 0, sizeof(l_sa4));
    l_sa4.sin_family = AF_INET;
    l_sa4.sin_port = htons(port);
    l_sa4.sin_addr = in_addr{.s_addr = INADDR_ANY};
    if (bind(fd, (const sockaddr *)&l_sa4, sizeof(l_sa4)) == -1) {
      LOGE("[UDP] Failed to bind UDP socket fd:%d to %s:%hu", fd, d_ip, port);
      close_udp_socket(fd, "[UDP]");
      return;
    }
  }
  LOG("[UDP] UDP socket fd:%d setup to %s:%hu", fd, d_ip, port);
}

void in_socket_udp(FD &fd, int16_t port) {
  if(port == 0) {
    LOG("[UDP_LISTEN] No UDP listening port configured, disable");
    close_udp_socket(fd, "[UDP_LISTEN]");
    return;
  }
  // already setup?
  if(fd != -1)
    return;

  // Setup listening socket
  LOG("[UDP_LISTEN] setting up UDP listening on port:%hu", port);
  if(!udp_socket(fd, 0, "[UDP_LISTEN]")) {
    LOGE("[UDP_LISTEN] Failed to create UDP listening socket, IPv4");
    return;
  }

  struct sockaddr_in t_sa4;
  memset(&t_sa4, 0, sizeof(t_sa4));
  t_sa4.sin_family = AF_INET;
  t_sa4.sin_port = htons(port);
  t_sa4.sin_addr = in_addr{.s_addr = INADDR_ANY};

  if(bind(fd, (struct sockaddr*)&t_sa4, sizeof(t_sa4)) < 0) {
    LOGE("[UDP_LISTEN] Failed to bind ipv4 UDP listening on port:%hu", port);
    close_udp_socket(fd, "[UDP_LISTEN]");
    return;
  }
  LOG("[UDP_LISTEN] UDP listening socket fd:%d setup on port:%hu", fd, port);
}

void in_socket_udp6(FD &fd, int16_t port) {
  if(port == 0) {
    LOG("[UDP6_LISTEN] No UDP6 listening port configured, disable");
    close_udp_socket(fd, "[UDP6_LISTEN]");
    return;
  }
  // already setup?
  if(fd != -1)
    return;

  // Setup IPv6-only listening socket
  LOG("[UDP6_LISTEN] setting up UDP6 listening on port:%hu", port);
  if(!udp_socket(fd, 1, "[UDP6_LISTEN]")) {
    LOGE("[UDP6_LISTEN] Failed to create UDP6 listening socket");
    return;
  }

  // Set IPv6-only mode
  int optval = 1;
  if (setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &optval, sizeof(optval)) < 0) {
    LOGE("[UDP6_LISTEN] Failed to set IPV6_V6ONLY");
    close_udp_socket(fd, "[UDP6_LISTEN]");
    return;
  }

  struct sockaddr_in6 t_sa6;
  memset(&t_sa6, 0, sizeof(t_sa6));
  t_sa6.sin6_family = AF_INET6;
  t_sa6.sin6_port = htons(port);
  t_sa6.sin6_addr = in6addr_any;

  if(bind(fd, (struct sockaddr*)&t_sa6, sizeof(t_sa6)) < 0) {
    LOGE("[UDP6_LISTEN] Failed to bind ipv6 UDP listening on port:%hu", port);
    close_udp_socket(fd, "[UDP6_LISTEN]");
    return;
  }
  LOG("[UDP6_LISTEN] UDP6 listening socket fd:%d setup on port:%hu", fd, port);
}

void out_socket_udp(FD &fd, int16_t port, const char* ip) {
  if(port == 0 || ip == NULL || strlen(ip) == 0) {
    // No sending port or IP configured
    close_udp_socket(fd, "[UDP_SEND]");
    LOG("[UDP_SEND] No UDP send IP or port configured, disable");
    return;
  }
  // already setup?
  if(fd != -1)
    return;

  // Setup sending socket, no need to bind, we just prepare sa6/sa4/sa for sendto()
  LOG("[UDP_SEND] setting up UDP sending to: %s:%hu", ip, port);
  if(is_ipv6_addr(ip)) {
    // IPv6
    if(!udp_socket(fd, 1, "[UDP_SEND]")) {
      LOGE("[UDP_SEND] Failed to create UDP sending socket, IPv6");
      return;
    }
  } else {
    // IPv4
    if(!udp_socket(fd, 0, "[UDP_SEND]")) {
      LOGE("[UDP_SEND] Failed to create UDP sending socket, IPv4");
      return;
    }
  }
  LOG("[UDP_SEND] UDP sending socket fd:%d setup to %s:%hu", fd, ip, port);
}

uint8_t udp_socket(FD &fd, uint8_t ipv6, const char* tag) {
  D("%s Creating UDP socket, type: %s, current fd: %d", tag, ipv6 ? "IPv6" : "IPv4", fd);

  // Close any existing socket
  close_udp_socket(fd, tag);

  // Socket
  if(ipv6) {
    D("%s Creating IPv6 UDP socket", tag);
    // IPv6
    fd = socket(AF_INET6, SOCK_DGRAM, 0);
    if (fd < 0) {
      LOGE("%s Failed to create IPv6 socket", tag);
      return 0;
    }
    LOG("%s socket ipv6 on fd:%d", tag, fd);
  } else {
    D("%s Creating IPv4 UDP socket", tag);
    // IPv4
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
      LOGE("%s Failed to create IPv4 socket", tag);
      return 0;
    }
    LOG("%s socket ipv4 on fd:%d", tag, fd);
  }
  if(fcntl(fd, F_SETFL, O_NONBLOCK | O_RDWR) < 0) {
    LOGE("%s Failed to set UDP socket to non-blocking", tag);
    close_udp_socket(fd, tag);
    return 0;
  }
  if(setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char[]) {1}, sizeof(int)) < 0) {
    LOGE("%s Failed to set SO_REUSEADDR on UDP socket", tag);
    close_udp_socket(fd, tag);
    return 0;
  }
  errno = 0;

  // all is well
  return 1;
}


void close_udp_socket(FD &fd, const char* tag) {
  if(fd < 0)
    return;

  // close()
  FD fd_orig = fd;
  if(errno != 0) {
    LOGE("%s closing UDP socket fd:%d, as we got an error", tag, fd);
  } else {
    LOG("%s closing UDP socket fd:%d", tag, fd);
  }
  errno = 0;
  if (close(fd) == -1)
    if (errno && errno != EBADF)
      LOGE("%s Failed to close socket fd:%d", tag, fd_orig);
  fd = -1;
}

// Helper: send UDP data (IPv4/IPv6)
int send_udp_data(FD &fd, const uint8_t* data, size_t len, char *d_ip, uint16_t port, char *tag) {
  D("%s Sending %d bytes to %s, fd:%d, port:%hu", tag, len, d_ip, fd, port);
  if(fd < 0)
    return -1;

  doYIELD;

  struct sockaddr_in s_sa4 = {0};
  struct sockaddr_in6 s_sa6 = {0};
  struct sockaddr *s_sa = NULL;
  size_t s_sa_sz = 0;
  if(is_ipv6_addr(d_ip)) {
    s_sa_sz = sizeof(s_sa6);
    s_sa6.sin6_family = AF_INET6;
    s_sa6.sin6_port = htons(port);
    s_sa = (struct sockaddr*)&s_sa6;
    if (inet_pton(AF_INET6, d_ip, &s_sa6.sin6_addr) != 1) {
      LOG("[UDP] Invalid IPv6 address:%s", d_ip);
      close_udp_socket(fd, tag);
      return -1;
    }
  } else {
    s_sa_sz = sizeof(s_sa4);
    s_sa4.sin_family = AF_INET;
    s_sa4.sin_port = htons(port);
    s_sa = (struct sockaddr*)&s_sa4;
    if (inet_pton(AF_INET, d_ip, &s_sa4.sin_addr) != 1) {
      LOG("[UDP] Invalid IPv4 address:%s", d_ip);
      close_udp_socket(fd, tag);
      return -1;
    }
  }
  size_t n = sendto(fd, data, len, 0, s_sa, s_sa_sz);
  doYIELD;
  if (n == -1) {
    LOGE("%s sendto failed to %s, len:%d, port:%hu on fd:%d", tag, d_ip, len, port, fd);
    close_udp_socket(fd, "[UDP]");
    doYIELD;
    return -1;
  } else if (n == 0) {
    D("%s send returned 0 bytes, no data sent", tag);
    doYIELD;
    return 0;
  } else {
    D("%s send_udp_data len: %d, sent: %d", tag, len, n);
  }
  doYIELD;
  return n;
}

// Helper: receive UDP data (IPv4/IPv6)
int recv_udp_data(FD &fd, uint8_t* buf, size_t maxlen) {
  size_t n = recv(fd, buf, maxlen, 0);
  if (n == -1) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      LOOP_E("[UDP] No data available on fd:%d, port:%hu", fd, cfg.udp_port);
      return 0;
    } else {
      LOGE("[UDP] recv failed from fd:%d, port:%hu", fd, cfg.udp_port);
      close_udp_socket(fd, "[UDP]");
      return -1;
    }
  } else if (n == 0) {
    D("[UDP] receive returned 0 bytes, no data received on fd:%d,port:%hu", fd, cfg.udp_port);
  }
  return n;
}

void udp_read(FD fd, uint8_t *buf, size_t &len, size_t read_size, size_t maxlen, const char *tag) {
  // ok file descriptor?
  if(fd < 0)
    return;

  // space in outbuf?
  if (len + read_size > maxlen) {
    D("%s outbuf full, cannot read more data, len: %d, read_size: %d, bufsize: %d", tag, len, read_size, maxlen);
    // no space in outbuf, cannot read more data
    // just yield and wait for outbuf to be cleared
    return;
  }

  // read data
  LOOP_D("%s Receiving up to %d bytes on fd:%d, port:%hu", tag, read_size, fd, cfg.udp_port);
  int os = recv_udp_data(fd, buf + len, read_size);
  if (os > 0) {
    #ifdef LED
    last_activity = millis(); // Trigger LED activity for UDP receive
    #endif // LED
    D("%s Received %d bytes, total: %d, data: >>%s<<", tag, os, len + os, buf);
    len += os;
    return;
  } else if (os < 0) {
    if(errno && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINPROGRESS) {
      // Error occurred, log it
      LOGE("%s receive error, closing connection", tag);
    }
  } else {
    // No data available, just yield
    LOOP_D("%s no data available, yielding...", tag);
  }
  return;
}
#endif // SUPPORT_WIFI && SUPPORT_UDP

#define AT_MODE     1
#define BRIDGE_MODE 0
#ifdef SUPPORT_BLE_UART1
NOINLINE
uint8_t at_mode = AT_MODE; // 1=AT command mode, 0=AT bridge mode
void ble_uart1_at_mode(uint8_t enable) {
  if(enable == AT_MODE) {
    LOG("[BLE_UART1] Switching to AT command mode");
    at_mode = AT_MODE;
  } else {
    LOG("[BLE_UART1] Switching to BLE UART1 bridge mode");
    at_mode = BRIDGE_MODE;
  }
}
#endif

#define CFG_PARTITION "esp-at"
#define CFG_NAMESPACE "esp-at"
#define CFG_STORAGE   "config"

NOINLINE
void CFG_SAVE() {
  CFG::SAVE(CFG_PARTITION, CFG_NAMESPACE, CFG_STORAGE, (void*)&cfg, sizeof(cfg));
}

NOINLINE
void CFG_CLEAR() {
  CFG::CLEAR(CFG_PARTITION, CFG_NAMESPACE, CFG_STORAGE);
}

NOINLINE
void CFG_LOAD() {
  CFG::LOAD(CFG_PARTITION, CFG_NAMESPACE, CFG_STORAGE, (void*)&cfg, sizeof(cfg));
}

NOINLINE
void setup_nvs(){
  if(!CFG::INIT(CFG_PARTITION, CFG_NAMESPACE)){
    LOGE("[CFG] Failed to initialize NVS storage");
    ESP.restart();
  } else {
    LOG("[CFG] NVS storage initialized");
  }
}

#define SUPPORT_POWER_MANAGEMENT
#ifdef SUPPORT_POWER_MANAGEMENT
#include "esp_pm.h"
#endif
NOINLINE
void setup_power_management(){
  #ifdef SUPPORT_POWER_MANAGEMENT
  LOG("[PM] Enabling power management");
  esp_pm_config_esp32_t pm_config;
  pm_config.max_freq_mhz = 160;
  pm_config.min_freq_mhz = 80;
  pm_config.light_sleep_enable = true;
  esp_err_t err = esp_pm_configure(&pm_config);
  if(err != ESP_OK) {
    LOG("[PM] Failed to configure power management, error: %d, %s", err, esp_err_to_name(err));
  } else {
    LOG("[PM] Power management configured: min %d MHz, max %d MHz, light sleep: %s",
    pm_config.min_freq_mhz,
    pm_config.max_freq_mhz,
    pm_config.light_sleep_enable ? "enabled" : "disabled");
  }
  #else
  LOG("[PM] Power management support not compiled in");
  #endif
}


#include <esp32-hal-cpu.h>
NOINLINE
void setup_cpu_speed(uint32_t freq_mhz = 160) {
  uint32_t xtal_f = getXtalFrequencyMhz();
  uint32_t cpu_f = getCpuFrequencyMhz();
  LOG("[ESP] Current CPU frequency: %d MHz", cpu_f);
  if(xtal_f != 0) {
    LOG("[ESP] Detected XTAL frequency: %d MHz", xtal_f);
    if(xtal_f != 40) {
      LOG("[ESP] Warning: Unusual XTAL frequency detected, expected 40 MHz");
    } else {
      if(xtal_f == 40) {
        // 40 MHz XTAL, set CPU to 80, 160
        // 40 MHz XTAL also allows 10, 20, 40 MHz CPU
        cpu_f = freq_mhz;
        LOG("[ESP] Setting CPU frequency to %d MHz for %d MHz XTAL", cpu_f, xtal_f);
        setCpuFrequencyMhz(cpu_f);
      }
    LOG("[ESP] New CPU frequency: %d MHz", getCpuFrequencyMhz());
    }
  } else {
    LOG("[ESP] Failed to detect XTAL frequency, defaulting to 40 MHz");
  }
}

#if defined(UART_AT) || defined(BLUETOOTH_UART_AT) || defined(BT_CLASSIC)

NOINLINE
void sc_cmd_handler(SerialCommands* s, const char* atcmdline) {
  if(s == NULL || atcmdline == NULL || strlen(atcmdline) == 0)
    return;
  D("SC: [%s]", atcmdline);
  const char *r = at_cmd_handler(atcmdline);
  if(r != NULL && strlen(r) > 0)
    s->GetSerial()->println(r);
}

const char *AT_short_help_string = R"EOF(Available AT Commands:
AT
AT?
AT+?
AT+HELP?
AT+RESET
AT+CPU_FREQ=|?
AT+ERASE=|1
)EOF"

#ifdef SUPPORT_WIFI
R"EOF(AT+WIFI_SSID=|?
AT+WIFI_PASS=
AT+WIFI_STATUS?
AT+WIFI_ENABLED=|?
AT+HOSTNAME=
AT+IPV4=
AT+IPV6=
AT+IP_STATUS?
)EOF"

#if defined(WIFI_WPS)
R"EOF(AT+WPS_PBC
AT+WPS_PIN=
AT+WPS_STOP
AT+WPS_STATUS?)EOF"
#endif

#ifdef SUPPORT_MDNS
R"EOF(AT+MDNS=|?
AT+MDNS_HOSTNAME=|?
AT+MDNS_STATUS?)EOF"
#endif

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP)
R"EOF(AT+TCP_PORT=|?
AT+TCP_HOST_IP=|?
AT+TCP_STATUS?
)EOF"
#endif

#ifdef SUPPORT_TLS
R"EOF(AT+TLS_ENABLE=|?
AT+TLS_PORT=|?
AT+TLS_VERIFY=|?
AT+TLS_SNI=|?
AT+TLS_CA_CERT=|?
AT+TLS_CLIENT_CERT=|?
AT+TLS_CLIENT_KEY=|?
AT+TLS_STATUS?
AT+TLS_CONNECT
AT+TLS_DISCONNECT
)EOF"
#endif

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
R"EOF(AT+TCP_SERVER_PORT=|?
AT+TCP_SERVER_MAX_CLIENTS=|?
AT+TCP_SERVER_STATUS?
AT+TCP_SERVER_START
AT+TCP_SERVER_STOP
AT+TCP_SERVER_SEND=
)EOF"
#endif

#if defined(SUPPORT_WIFI) && defined(SUPPORT_UDP)
R"EOF(AT+UDP_PORT=|?
AT+UDP_HOST_IP=|?
AT+UDP_LISTEN_PORT=|?
AT+UDP6_LISTEN_PORT=|?
AT+UDP_SEND=|?
)EOF"
#endif

#if defined(SUPPORT_WIFI) && defined(SUPPORT_NTP)
R"EOF(AT+NTP_ENABLED=|?
AT+NTP_HOST=|?
AT+NTP_STATUS?
)EOF"
#endif

#endif // SUPPORT_WIFI

#ifdef SUPPORT_UART1
R"EOF(AT+UART1=|?
)EOF"
#endif

#ifdef SUPPORT_GPIO
R"EOF(AT+GPIO=|?
)EOF"
#endif

#ifdef VERBOSE
R"EOF(AT+VERBOSE=|?
)EOF"
#endif

#ifdef TIMELOG
R"EOF(AT+TIMELOG=|?
)EOF"
#endif

#ifdef LOGUART
R"EOF(AT+LOG_UART=|?
)EOF"
#endif

#ifdef LOOP_DELAY
R"EOF(AT+LOOP_DELAY=|?
)EOF"
#endif

#if defined(BLUETOOTH_UART_AT) || defined(SUPPORT_BLE_UART1)
R"EOF(AT+BLE_PIN=|?
AT+BLE_SECURITY=|?
AT+BLE_IO_CAP=|?
AT+BLE_AUTH_REQ=|?
AT+BLE_ADDR_TYPE=|?
AT+BLE_ADDR=|?
AT+BLE_ADDR_GEN?
AT+BLE_STATUS?
AT+BLE_UART1=|?
AT+BLE_UART1_PASS=|?
)EOF"
#endif

R"EOF(
Use AT+HELP? for detailed help
)EOF";

const char AT_help_string[] = R"EOF(
ESP-AT Command Help:

Basic Commands:
  AT                            - Test AT startup
  AT?                           - Test AT startup
  AT+?                          - Show this help
  AT+HELP?                      - Show this help
  AT+ERASE                      - Erase all configuration, reset to factory defaults
  AT+ERASE=1                    - Erase all configuration and restart immediately)EOF"

#ifdef SUPPORT_WIFI
R"EOF(
WiFi Commands:
  AT+WIFI_ENABLED=<1|0>         - Enable/Disable WiFi (1=enable, 0=disable)
  AT+WIFI_ENABLED?              - Get WiFi enable status
  AT+WIFI_SSID=<ssid>           - Set WiFi SSID
  AT+WIFI_SSID?                 - Get WiFi SSID
  AT+WIFI_PASS=<pass>           - Set WiFi password
  AT+WIFI_STATUS?               - Get WiFi connection status
  AT+HOSTNAME=<name>            - Set device hostname
  AT+HOSTNAME?                  - Get device hostname
Network Commands:
  AT+IPV4=<config>              - Set IPv4 config (DHCP/DISABLE/ip,mask,gw[,dns])
  AT+IPV4?                      - Get IPv4 configuration
  AT+IPV6=<config>              - Set IPv6 configuration
  AT+IPV6?                      - Get IPv6 configuration
  AT+IP_STATUS?                 - Get current IP addresses)EOF"
#endif // SUPPORT_WIFI

#ifdef WIFI_WPS
R"EOF(
WPS Commands:
  AT+WPS_PBC                    - Start WPS Push Button Configuration
  AT+WPS_PIN=<pin>              - Start WPS PIN method
  AT+WPS_STOP                   - Stop WPS
  AT+WPS_STATUS?                - Get WPS status)EOF"
#endif // WIFI_WPS

#ifdef SUPPORT_MDNS
R"EOF(
mDNS Commands:
  AT+MDNS=<0|1>                 - Enable/disable mDNS responder
  AT+MDNS?                      - Get mDNS responder status
  AT+MDNS_HOSTNAME=<name>       - Set mDNS hostname (defaults to hostname)
  AT+MDNS_HOSTNAME?             - Get mDNS hostname)EOF"
#endif // SUPPORT_MDNS

#if defined(SUPPORT_TCP) || defined(SUPPORT_UDP)
R"EOF(
Network Configuration:
  AT+NETCONF?                   - Get current network configuration
  AT+NETCONF=(protocol,host,port)
                                - Configure TCP/UDP connection
    Examples:
      AT+NETCONF=(TCP,192.168.1.100,8080)
      AT+NETCONF=(UDP,192.168.1.200,9090)
      AT+NETCONF=(TCP,192.168.1.100,8080);(UDP,192.168.1.200,9090)
      AT+NETCONF=(TCP,2001:db8::1,8080);(UDP,2001:db8::2,9090)
      AT+NETCONF=(UDP_LISTEN,5678)
      AT+NETCONF=(UDP6_LISTEN,5679)
      AT+NETCONF=(TCP_SERVER,1234)
      AT+NETCONF=(TCP6_SERVER,1235)
      AT+NETCONF=(UDP_LISTEN,5678);(TCP_SERVER,1234)
      AT+NETCONF=(UDP6_LISTEN,5679);(TCP6_SERVER,1235)
      AT+NETCONF=(UDP_SEND,192.168.1.100,5678);(UDP_LISTEN,5679)
      AT+NETCONF=)EOF"
#endif // SUPPORT_TCP || SUPPORT_UDP

#ifdef SUPPORT_TCP
R"EOF(
TCP Commands (Legacy):
  AT+TCP_PORT=<port>            - Set TCP port
  AT+TCP_PORT?                  - Get TCP port
  AT+TCP_HOST_IP=<ip>           - Set TCP host IP
  AT+TCP_HOST_IP?               - Get TCP host IP
  AT+TCP_STATUS?                - Get TCP connection status)EOF"
#endif // SUPPORT_TCP

#ifdef SUPPORT_TCP_SERVER
R"EOF(
TCP Server Commands:
  AT+TCP_SERVER_PORT=<port>     - Set TCP server port
  AT+TCP_SERVER_PORT?           - Get TCP server port
  AT+TCP_SERVER_MAX_CLIENTS=<n> - Set max clients
  AT+TCP_SERVER_MAX_CLIENTS?    - Get max clients
  AT+TCP_SERVER_STATUS?         - Get TCP server status
  AT+TCP_SERVER_START           - Start TCP server
  AT+TCP_SERVER_STOP            - Stop TCP server
  AT+TCP_SERVER_SEND=<data>     - Send data to clients)EOF"
#endif // SUPPORT_TCP_SERVER

#ifdef SUPPORT_TLS
R"EOF(
TLS/SSL Commands:
  AT+TLS_ENABLE=<0|1>           - Enable/disable TLS for TCP connections
  AT+TLS_ENABLE?                - Get TLS enable status
  AT+TLS_PORT=<port>            - Set TLS port (defaults to TCP port if not set)
  AT+TLS_PORT?                  - Get TLS port
  AT+TLS_VERIFY=<0|1|2>         - Set certificate verification
                                    0=none
                                    1=optional
                                    2=required
  AT+TLS_VERIFY?                - Get certificate verification mode
  AT+TLS_SNI=<0|1>              - Enable/disable Server Name Indication
  AT+TLS_SNI?                   - Get SNI status
  AT+TLS_CA_CERT=<cert>         - Set CA certificate (PEM format)
  AT+TLS_CA_CERT?               - Check if CA certificate is set
  AT+TLS_CLIENT_CERT=<cert>     - Set client certificate (PEM format)
  AT+TLS_CLIENT_CERT?           - Check if client certificate is set
  AT+TLS_CLIENT_KEY=<key>       - Set client private key (PEM format)
  AT+TLS_CLIENT_KEY?            - Check if client key is set
  AT+TLS_STATUS?                - Get TLS connection status and cipher info
  AT+TLS_CONNECT                - Manually connect TLS
  AT+TLS_DISCONNECT             - Disconnect TLS)EOF"
#endif // SUPPORT_TLS

#ifdef SUPPORT_UDP
R"EOF(
UDP Commands (Legacy):
  AT+UDP_PORT=<port>            - Set UDP port
  AT+UDP_PORT?                  - Get UDP port
  AT+UDP_LISTEN_PORT=<port>     - Set UDP listen port
  AT+UDP_LISTEN_PORT?           - Get UDP listen port
  AT+UDP6_LISTEN_PORT=<port>    - Set UDP6 listen port (IPv6 only)
  AT+UDP6_LISTEN_PORT?          - Get UDP6 listen port
  AT+UDP_SEND=<ip:port>         - Set UDP send IP and port
  AT+UDP_SEND?                  - Get UDP send IP and port
  AT+UDP_HOST_IP=<ip>           - Set UDP host IP
  AT+UDP_HOST_IP?               - Get UDP host IP)EOF"
#endif // SUPPORT_UDP

#ifdef SUPPORT_NTP
R"EOF(
NTP Commands:
  AT+NTP_ENABLED=<1|0>          - Enable/disable NTP (1=enable, 0=disable)
  AT+NTP_ENABLED?               - Get NTP enable status
  AT+NTP_HOST=<host>            - Set NTP server hostname
  AT+NTP_HOST?                  - Get NTP server hostname
  AT+NTP_STATUS?                - Get NTP sync status)EOF"
#endif // SUPPORT_NTP

#ifdef SUPPORT_UART1
R"EOF(
UART1 Commands:
  AT+UART1=baud,data,parity,stop[,rx,tx,invert]
                                - Configure UART1 parameters
                                    baud: 300-115200,
                                    data: 5-8 bits,
                                    parity: 0=None/1=Even/2=Odd
                                    stop: 1-2 bits,
                                    rx/tx (optional):
                                      pin 0-39 (ESP32)
                                      pin 0-16 (ESP8266)
                                    invert (optional):
                                      0=normal, 1=inverted
  AT+UART1?                     - Get current UART1 configuration)EOF"
#endif // SUPPORT_UART1

#ifdef SUPPORT_GPIO
R"EOF(
GPIO Commands:
  AT+GPIO=<pin>,<mode>,<value>  - Configure GPIO pin
                                    mode: 0=INPUT, 1=OUTPUT, 2=INPUT_PULLUP, 3=INPUT_PULLDOWN
                                    value: 0=LOW, 1=HIGH (only for OUTPUT mode)
  AT+GPIO?                      - Get current GPIO configuration)EOF"
#endif // SUPPORT_GPIO

R"EOF(
System Commands:
  AT+RESET                      - Restart device
  AT+CPU_FREQ=<freq>            - Set CPU frequency (10, 20, 40, 80, 160 MHz)
  AT+CPU_FREQ?                  - Get current CPU frequency)EOF"

#ifdef LOOP_DELAY
R"EOF(
  AT+LOOP_DELAY=<ms>            - Set main loop delay
  AT+LOOP_DELAY?                - Get main loop delay)EOF"
#endif // LOOP_DELAY

#ifdef VERBOSE
R"EOF(
  AT+VERBOSE=<0|1>              - Enable/disable verbose logging
  AT+VERBOSE?                   - Get verbose logging status)EOF"
#endif // VERBOSE

#ifdef SUPPORT_ESP_LOG_INFO
R"EOF(
  AT+ESP_LOG_INTERVAL=<ms>      - Set ESP log info interval in ms (0=disable)
  AT+ESP_LOG_INTERVAL?          - Get ESP log info interval)EOF"
#endif //

#ifdef TIMELOG
R"EOF(
  AT+TIMELOG=<0|1>              - Enable/disable time logging
  AT+TIMELOG?                   - Get time logging status)EOF"
#endif // TIMELOG

#ifdef LOGUART
R"EOF(
  AT+LOG_UART=<0|1>             - Enable/disable UART logging
  AT+LOG_UART?                  - Get UART logging status)EOF"
#endif // LOGUART

#ifdef BLUETOOTH_UART_AT
R"EOF(
BLE Commands:
  AT+BLE_PIN=<pin>              - Set BLE PIN (6 digits)
  AT+BLE_PIN?                   - Get current BLE PIN
  AT+BLE_SECURITY=<mode>        - Set BLE security mode
                                    0=None
                                    1=PIN
                                    2=Bonding
  AT+BLE_SECURITY?              - Get BLE security mode
  AT+BLE_IO_CAP=<cap>           - Set BLE IO capability
                                    0=DisplayOnly
                                    1=DisplayYesNo
                                    2=KeyboardOnly
                                    3=NoInputNoOutput
                                    4=KeyboardDisplay
  AT+BLE_IO_CAP?                - Get BLE IO capability
  AT+BLE_AUTH_REQ=<req>         - Set authentication requirements
                                    0=None
                                    1=Bonding
                                    2=MITM
                                    3=Bonding+MITM
  AT+BLE_AUTH_REQ?              - Get authentication requirements
  AT+BLE_ADDR_TYPE=<type>       - Set BLE address type
                                    0=Public
                                    1=Random Static
                                    2=Private Resolvable
                                    3=Private Non-resolvable
  AT+BLE_ADDR_TYPE?             - Get BLE address type
  AT+BLE_ADDR=<address>         - Set custom BLE MAC address (format: XX:XX:XX:XX:XX:XX)
  AT+BLE_ADDR?                  - Get current BLE MAC address
  AT+BLE_ADDR_GEN?              - Generate new random static address
  AT+BLE_STATUS?                - Get BLE connection and security status
  AT+BLE_UART1=|?               - Enable/disable BLE UART1 bridge
  AT+BLE_UART1_PASS=<0|1>       - Enable/disable passthrough mode
                                    1=passthrough
                                    0=AT command mode
  AT+BLE_UART1_PASS?            - Get passthrough mode status)EOF"
#endif // BLUETOOTH_UART_AT

#ifdef SUPPORT_PLUGINS
R"EOF(
Plugin Commands:
  AT+PLUGINS?                   - Show plugin command help
)EOF"
#endif // SUPPORT_PLUGINS

R"EOF(

Note: Commands with '?' are queries, commands with '=' set values
)EOF";

#define at_cmd_check(a,b,c) COMMON::at_cmd_check(a,b,c)

const char* at_cmd_handler(const char* atcmdline) {
  unsigned int cmd_len = strlen(atcmdline);
  char *p = NULL;
  char *r = NULL;
  errno = 0;
  D("[AT] [%s], size: %d", atcmdline, cmd_len);
  if(cmd_len == 2 && (p = at_cmd_check("AT", atcmdline, cmd_len))) {
    return AT_R_OK;
  } else if(cmd_len == 3 && (p = at_cmd_check("AT?", atcmdline, cmd_len))) {
    return AT_R_OK;
  #ifdef LOOP_DELAY
  } else if(p = at_cmd_check("AT+LOOP_DELAY=", atcmdline, cmd_len)) {
    uint32_t new_c = (uint32_t)strtoul(p, &r, 10);
    if(errno != 0 || new_c < 0 || new_c > 86400000 || (r == p))
      return AT_R("+ERROR: invalid loop delay");
    if(new_c != cfg.main_loop_delay) {
      cfg.main_loop_delay = new_c;
      CFG_SAVE();
    }
    setup_button();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LOOP_DELAY?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.main_loop_delay);
  #endif // LOOP_DELAY
  #ifdef TIMELOG
  } else if(p = at_cmd_check("AT+TIMELOG=1", atcmdline, cmd_len)) {
    cfg.do_timelog = 1;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TIMELOG=0", atcmdline, cmd_len)) {
    cfg.do_timelog = 0;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TIMELOG?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.do_timelog);
  #endif // TIMELOG
  #ifdef VERBOSE
  } else if(p = at_cmd_check("AT+VERBOSE=1", atcmdline, cmd_len)) {
    cfg.do_verbose = 1;
    COMMON::_do_verbose = 1;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+VERBOSE=0", atcmdline, cmd_len)) {
    cfg.do_verbose = 0;
    COMMON::_do_verbose = 0;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+VERBOSE?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.do_verbose);
  #endif // VERBOSE
  #ifdef SUPPORT_ESP_LOG_INFO
  } else if(p = at_cmd_check("AT+ESP_LOG_INTERVAL=", atcmdline, cmd_len)) {
    uint32_t l_intv = (uint32_t)strtoul(p, &r, 10);
    if(errno != 0 || (r == p))
      return AT_R("+ERROR: invalid log interval, must be in ms (0=disable)");
    cfg.esp_log_interval = l_intv;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+ESP_LOG_INTERVAL?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.esp_log_interval);
  #endif // SUPPORT_ESP_LOG_INFO
  #ifdef LOGUART
  } else if(p = at_cmd_check("AT+LOG_UART=1", atcmdline, cmd_len)) {
    cfg.do_log = 1;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LOG_UART=0", atcmdline, cmd_len)) {
    cfg.do_log = 0;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LOG_UART?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.do_log);
  #endif // LOGUART
  #ifdef SUPPORT_GPIO
  } else if(p = at_cmd_check("AT+GPIO=", atcmdline, cmd_len)) {
    // Format: AT+GPIO=pin,mode,logic
    int8_t pin = -1, mode = -1, value = -1;
    char *r = NULL;
    pin = (int8_t)strtoul(p, &r, 10);
    if(errno != 0 || r == p || pin < 0 || pin > 39)
      return AT_R("+ERROR: Invalid pin number (0-39)");
    if(*r != ',')
      return AT_R("+ERROR: Format is AT+GPIO=pin,mode,logic");
    mode = (int8_t)strtoul(r+1, &r, 10);
    if(errno != 0 || r == p || mode < 0 || mode > 3)
      return AT_R("+ERROR: Invalid mode (0=normal, 1=pullup, 2=pulldown, 3=pullup+pulldown)");
    if(*r != ',')
      return AT_R("+ERROR: Format is AT+GPIO=pin,mode,logic");
    value = (int8_t)strtoul(r+1, &r, 10);
    if(errno != 0 || r == p || (value != 0 && value != 1))
      return AT_R("+ERROR: Invalid logic value (0=low, 1=high)");

    // Save to config (replace if exists, else add)
    bool found = false;
    for(uint8_t i=0; i<cfg.gpio_cfg_count; ++i) {
      if(cfg.gpio_cfg[i].pin == pin) {
        cfg.gpio_cfg[i].mode = mode;
        cfg.gpio_cfg[i].value = value;
        found = true;
        break;
      }
    }
    if(!found && cfg.gpio_cfg_count < 10) {
      cfg.gpio_cfg[cfg.gpio_cfg_count].pin = pin;
      cfg.gpio_cfg[cfg.gpio_cfg_count].mode = mode;
      cfg.gpio_cfg[cfg.gpio_cfg_count].value = value;
      cfg.gpio_cfg_count++;
    }
    CFG_SAVE();

    // Apply immediately
    esp_err_t err = ESP_OK;
    err = gpio_hold_dis((gpio_num_t)pin);
    if(err != ESP_OK) {
      LOG("[AT] GPIO pin %d hold disable failed: %s", pin, esp_err_to_name(err));
    }
    pinMode(pin, OUTPUT);

    // Set pullup/pulldown
    if(mode == 0) {
      LOG("[AT] GPIO pin %d set to normal (no pullup/pulldown), logic: %s", pin, value == 1 ? "HIGH" : "LOW");
      gpio_pullup_dis((gpio_num_t)pin);
      gpio_pulldown_dis((gpio_num_t)pin);
      digitalWrite(pin, value == 1 ? HIGH : LOW);
    } else if(mode == 1) {
      LOG("[AT] GPIO pin %d set to pullup", pin);
      gpio_pullup_en((gpio_num_t)pin);
      gpio_pulldown_dis((gpio_num_t)pin);
    } else if(mode == 2) {
      LOG("[AT] GPIO pin %d set to pulldown", pin);
      gpio_pullup_dis((gpio_num_t)pin);
      gpio_pulldown_en((gpio_num_t)pin);
    } else if(mode == 3) {
      LOG("[AT] GPIO pin %d set to pullup+pulldown", pin);
      gpio_pullup_en((gpio_num_t)pin);
      gpio_pulldown_en((gpio_num_t)pin);
    } else {
      return AT_R("+ERROR: Invalid mode (0=normal, 1=pullup, 2=pulldown, 3=pullup+pulldown)");
    }
    err = gpio_hold_en((gpio_num_t)pin);
    if(err != ESP_OK) {
      LOG("[AT] GPIO pin %d hold enable failed: %s", pin, esp_err_to_name(err));
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+GPIO?", atcmdline, cmd_len)) {
    // return all GPIO configs
    LOG("[AT] GPIO count: %d", cfg.gpio_cfg_count);
    ALIGN(4) static char _buf[128] = {0};
    memset(_buf, 0, sizeof(_buf));
    for(uint8_t i=0; i<cfg.gpio_cfg_count && i<10; ++i) {
      if(strlen(_buf) + 10 >= sizeof(_buf))
        break;
      if(i > 0)
        strcat(_buf, ";");
      sprintf(_buf + strlen(_buf), "%d,%d,%d", cfg.gpio_cfg[i].pin, cfg.gpio_cfg[i].mode, cfg.gpio_cfg[i].value);
    }
    if(strlen(_buf) == 0)
      return AT_R("+EMPTY");
    return AT_R_STR(_buf);
  #endif // SUPPORT_GPIO
  #ifdef SUPPORT_UART1
  } else if(p = at_cmd_check("AT+UART1=", atcmdline, cmd_len)) {
    // Parse format: baud,data,parity,stop,rx_pin,tx_pin
    // Example: AT+UART1=115200,8,0,1,0,1,0

    // Find comma positions
    char *ct = p;
    char *cp[6] = {0};
    int cc = 0;
    while(cc < 6 && ct < p+cmd_len) {
      ct = strchr(ct, ',');
      if(ct == NULL)
        break;
      ct++; // Move past comma
      cp[cc] = ct;
      cc++;
    }
    D("[AT] UART1 commas: %d", cc);
    if(cc < 3)
      return AT_R("+ERROR: Format: baud,data,parity,stop[,rx_pin,tx_pin,inverted]");

    // print the strings for debugging
    for(int i = 0; i < 6; i++) {
      if(cp[i] == NULL)
        break;
      D("[AT] UART1 param %d: %s", i, cp[i]);
    }

    // Parse and validate parameters
    uint32_t baud = strtoul(p, &r, 10);
    if(errno != 0)
      return AT_R("+ERROR: Invalid baud rate");
    if(r == p)
      baud = 115200; // default
    if(baud < 300 || baud > 115200)
      return AT_R("+ERROR: Baud rate must be 300-115200");

    uint8_t data = strtoul(cp[0], &r, 10);
    if(errno != 0)
      return AT_R("+ERROR: Invalid data bits");
    if(r == cp[0])
      data = 8; // default
    if(data < 5 || data > 8)
      return AT_R("+ERROR: Data bits must be 5-8");

    uint8_t parity = strtoul(cp[1], &r, 10);
    if(errno != 0)
      return AT_R("+ERROR: Invalid parity");
    if(r == cp[1])
      parity = 0; // default
    if(parity > 2)
      return AT_R("+ERROR: Parity: 0=None, 1=Even, 2=Odd");

    uint8_t stop = strtoul(cp[2], &r, 10);
    if(errno != 0)
      return AT_R("+ERROR: Invalid stop bits");
    if(r == cp[2])
      stop = 1; // default
    if(stop < 1 || stop > 2)
      return AT_R("+ERROR: Stop bits must be 1 or 2");

    uint8_t rx_pin = UART1_RX_PIN;
    uint8_t tx_pin = UART1_TX_PIN;
    if(cc >= 4 && cp[3] != NULL && cp[4] != NULL) {
        rx_pin = strtoul(cp[3], &r, 10);
        if(errno != 0)
          return AT_R("+ERROR: Invalid RX pin");
        if(r == cp[3])
          rx_pin = UART1_RX_PIN; // default
        tx_pin = strtoul(cp[4], &r, 10);
        if(errno != 0)
          return AT_R("+ERROR: Invalid TX pin");
        if(r == cp[4])
          tx_pin = UART1_TX_PIN; // default
        if(rx_pin > 39 || tx_pin > 39)
          return AT_R("+ERROR: Pin numbers must be 0-39");
    }
    uint8_t is_inverted = 0;
    if(cc >= 6 && cp[5] != NULL) {
        is_inverted = strtoul(cp[5], &r, 10);
        if(errno != 0)
          return AT_R("+ERROR: Invalid inverted flag: use 0 or 1");
        if(r == cp[5])
          is_inverted = 0;
        if(is_inverted != 0 && is_inverted != 1)
          return AT_R("+ERROR: Invalid inverted flag: use 0 or 1");
    }

    LOG("[AT] UART1 config: baud=%d, data=%d, parity=%d, stop=%d, rx=%d, tx=%d, inverted=%d", baud, data, parity, stop, rx_pin, tx_pin, is_inverted);

    // Update configuration
    cfg.uart1_baud   = baud;
    cfg.uart1_data   = data;
    cfg.uart1_parity = parity;
    cfg.uart1_stop   = stop;
    cfg.uart1_rx_pin = rx_pin;
    cfg.uart1_tx_pin = tx_pin;
    cfg.uart1_inv    = is_inverted;

    // Save configuration
    CFG_SAVE();

    // Apply new configuration
    setup_uart1();

    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UART1?", atcmdline, cmd_len)) {
    String response =
        String(cfg.uart1_baud)   + "," +
        String(cfg.uart1_data)   + "," +
        String(cfg.uart1_parity) + "," +
        String(cfg.uart1_stop)   + "," +
        String(cfg.uart1_rx_pin) + "," +
        String(cfg.uart1_tx_pin) + "," +
        String(cfg.uart1_inv);
    return AT_R_S(response);
  #endif // SUPPORT_UART1
  #ifdef SUPPORT_BLE_UART1
  } else if(p = at_cmd_check("AT+BLE_UART1=", atcmdline, cmd_len)) {
    int enable_ble_uart1 = strtoul(p, &r, 10);
    if(errno != 0 || r == p)
      return AT_R("+ERROR: Invalid parameter, use 1 to enable, 0 to disable");
    if(enable_ble_uart1 == 1) {
      cfg.ble_uart1_bridge = 1;
      CFG_SAVE();
      return AT_R_OK;
    } else if(enable_ble_uart1 == 0) {
      at_mode = AT_MODE; // Force AT mode
      cfg.ble_uart1_bridge = 0;
      CFG_SAVE();
      return AT_R_OK;
    } else {
      return AT_R("+ERROR: Use AT+BLE_UART1=1 to enable, AT+BLE_UART1=0 to disable");
    }
  } else if(p = at_cmd_check("AT+BLE_UART1?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.ble_uart1_bridge);
  } else if(p = at_cmd_check("AT+BLE_UART1_PASS=", atcmdline, cmd_len)) {
    // Switch between AT command mode and BLE UART1 passthrough mode
    // Only works if BLE UART1 bridge is enabled, otherwise always in AT mode
    if(!cfg.ble_uart1_bridge) {
      // Force AT mode
      ble_uart1_at_mode(AT_MODE);
      return AT_R("+ERROR: BLE UART1 bridge is disabled, enable with AT+BLE_UART1=1");
    }
    // Parse parameter
    uint8_t m_req = strtoul(p, &r, 10);
    if(errno != 0 || r == p)
      return AT_R("+ERROR: Invalid parameter, use 1 for AT mode, 0 for passthrough mode");
    if(m_req != 0 && m_req != 1)
      return AT_R("+ERROR: Use 1 for AT command mode, 0 for BLE UART1 passthrough mode");
    // Set mode
    if(m_req == 1) {
      // Switch to passthrough mode
      ble_uart1_at_mode(BRIDGE_MODE);
      return AT_R(""); // don't reply
    } else {
      // Stay in AT command mode
      ble_uart1_at_mode(AT_MODE);
      return AT_R_OK;  // reply OK
    }
  } else if (p = at_cmd_check("AT+BLE_UART1_PASS?", atcmdline, cmd_len)) {
    if(!cfg.ble_uart1_bridge) {
      return AT_R("+ERROR: BLE UART1 bridge is disabled, enable with AT+BLE_UART1=1");
    }
    if(at_mode == AT_MODE)
      return AT_R("0"); // Passthrough mode
    else
      return AT_R("1"); // AT command mode
  #endif // SUPPORT_BLE_UART1
  #ifdef SUPPORT_WIFI
  } else if(p = at_cmd_check("AT+WIFI_SSID=", atcmdline, cmd_len)) {
    if(strlen(p) > 31)
      return AT_R("+ERROR: WiFI SSID max 31 chars");
    if(strlen(p) == 0) {
      // Empty SSID, clear it
      memset((char *)&cfg.wifi_ssid, 0, MAX_WIFI_SSID);
      CFG_SAVE();
      restart_networking = 1;
      return AT_R_OK;
    }
    memset((char *)&cfg.wifi_ssid, 0, MAX_WIFI_SSID);
    strncpy((char *)&cfg.wifi_ssid, p, strlen(p));
    cfg.wifi_ssid[strlen(p)] = '\0';
    CFG_SAVE();
    restart_networking = 1;
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+WIFI_SSID?", atcmdline, cmd_len)) {
    if(strlen(cfg.wifi_ssid) == 0)
      return AT_R("+ERROR: WiFi SSID not set");
    else
      return AT_R_STR(cfg.wifi_ssid);
  } else if(p = at_cmd_check("AT+WIFI_PASS=", atcmdline, cmd_len)) {
    if(strlen(p) > 63)
      return AT_R("+ERROR: WiFi PASS max 63 chars");
    if(strlen(p) == 0) {
      // Empty password, clear it
      memset((char *)&cfg.wifi_pass, 0, MAX_WIFI_PASS);
      CFG_SAVE();
      restart_networking = 1;
      return AT_R_OK;
    }
    memset((char *)&cfg.wifi_pass, 0, MAX_WIFI_PASS);
    strncpy((char *)&cfg.wifi_pass, p, strlen(p));
    cfg.wifi_pass[strlen(p)] = '\0';
    CFG_SAVE();
    restart_networking = 1;
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+WIFI_STATUS?", atcmdline, cmd_len)) {
    if(!cfg.wifi_enabled)
      return AT_R("wifi disabled");
    uint8_t wifi_stat = WiFi.status();
    switch(wifi_stat) {
        case WL_CONNECTED:
          return AT_R("connected");
        case WL_CONNECT_FAILED:
          return AT_R("failed");
        case WL_CONNECTION_LOST:
          return AT_R("connection lost");
        case WL_DISCONNECTED:
          return AT_R("disconnected");
        case WL_IDLE_STATUS:
          return AT_R("idle");
        case WL_NO_SSID_AVAIL:
          return AT_R("no SSID configured");
        default:
          return AT_R_INT(wifi_stat);
    }
  #ifdef WIFI_WPS
  } else if(p = at_cmd_check("AT+WPS_PBC", atcmdline, cmd_len)) {
    if(start_wps(NULL)) {
      return AT_R_OK;
    } else {
      return AT_R("+ERROR: Failed to start WPS PBC");
    }
  } else if(p = at_cmd_check("AT+WPS_PIN=", atcmdline, cmd_len)) {
    if(strlen(p) != 8) {
      return AT_R("+ERROR: WPS PIN must be 8 digits");
    }
    // Verify PIN contains only digits
    for(int i = 0; i < 8; i++) {
      if(p[i] < '0' || p[i] > '9') {
        return AT_R("+ERROR: WPS PIN must contain only digits");
      }
    }
    if(start_wps(p)) {
      return AT_R_OK;
    } else {
      return AT_R("+ERROR: Failed to start WPS PIN");
    }
  } else if(p = at_cmd_check("AT+WPS_STOP", atcmdline, cmd_len)) {
    if(stop_wps()) {
      reset_networking();
      return AT_R_OK;
    } else {
      return AT_R("+ERROR: WPS not running");
    }
  } else if(p = at_cmd_check("AT+WPS_STATUS?", atcmdline, cmd_len)) {
    return AT_R(get_wps_status());
  #endif // WIFI_WPS
  } else if(p = at_cmd_check("AT+WIFI_ENABLED=1", atcmdline, cmd_len)) {
    cfg.wifi_enabled = 1;
    CFG_SAVE();
    restart_networking = 1;
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+WIFI_ENABLED=0", atcmdline, cmd_len)) {
    cfg.wifi_enabled = 0;
    CFG_SAVE();
    restart_networking = 1;
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+WIFI_ENABLED?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.wifi_enabled);
  #ifdef SUPPORT_NTP
  } else if(p = at_cmd_check("AT+NTP_ENABLED=1", atcmdline, cmd_len)) {
    cfg.ntp_enabled = 1;
    CFG_SAVE();
    setup_ntp();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+NTP_ENABLED=0", atcmdline, cmd_len)) {
    cfg.ntp_enabled = 0;
    CFG_SAVE();
    setup_ntp();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+NTP_ENABLED?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.ntp_enabled);
  } else if(p = at_cmd_check("AT+NTP_HOST=", atcmdline, cmd_len)) {
    if(strlen(p) > 63)
      return AT_R("+ERROR: NTP hostname max 63 chars");
    if(strlen(p) == 0) {
      // Empty hostname, clear it
      LOG("[AT] Clearing NTP hostname, will be using default %s", DEFAULT_NTP_SERVER);
      memset((char *)&cfg.ntp_host, 0, sizeof(cfg.ntp_host));
      cfg.ntp_host[0] = '\0';
      CFG_SAVE();
      setup_ntp();
      return AT_R_OK;
    }
    strncpy((char *)&cfg.ntp_host, p, strlen(p));
    cfg.ntp_host[sizeof(cfg.ntp_host) - 1] = '\0';
    LOG("[AT] Setting NTP hostname to: %s", cfg.ntp_host);
    CFG_SAVE();
    setup_ntp();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+NTP_HOST?", atcmdline, cmd_len)) {
    if(strlen(cfg.ntp_host) == 0)
      return AT_R("+ERROR: NTP hostname not set");
    else
      return AT_R_STR(cfg.ntp_host);
  } else if(p = at_cmd_check("AT+NTP_STATUS?", atcmdline, cmd_len)) {
    if(ntp_is_synced)
      return AT_R("ntp synced");
    else
      return AT_R("not ntp synced");
  #endif // SUPPORT_NTP
  #if defined(SUPPORT_UDP) || defined(SUPPORT_TCP)
  } else if(p = at_cmd_check("AT+NETCONF?", atcmdline, cmd_len)) {
    String response = "";
    #ifdef SUPPORT_TCP
    if(cfg.tcp_port > 0 && strlen(cfg.tcp_host_ip) > 0) {
      response += "TCP,";
      response += cfg.tcp_host_ip;
      response += ",";
      response += cfg.tcp_port;
    }
    #endif
    #ifdef SUPPORT_TCP_SERVER
    if(cfg.tcp_server_port > 0) {
      if(response.length() > 0) response += ";";
      response += "TCP_SERVER,";
      response += cfg.tcp_server_port;
      response += ",max_clients=";
      response += cfg.tcp_server_max_clients;
      if(tcp_server_sock != -1) {
        response += ",status=ACTIVE,clients=";
        response += get_tcp_server_client_count();
      } else {
        response += ",status=INACTIVE";
      }
    }
    if(cfg.tcp6_server_port > 0) {
      if(response.length() > 0) response += ";";
      response += "TCP6_SERVER,";
      response += cfg.tcp6_server_port;
      response += ",max_clients=";
      response += cfg.tcp_server_max_clients;
      if(tcp6_server_sock != -1) {
        response += ",status=ACTIVE,clients=";
        response += get_tcp_server_client_count();
      } else {
        response += ",status=INACTIVE";
      }
    }
    #endif
    #ifdef SUPPORT_UDP
    if(cfg.udp_port > 0 && strlen(cfg.udp_host_ip) > 0) {
      if(response.length() > 0) response += ";";
      response += "UDP,";
      response += cfg.udp_host_ip;
      response += ",";
      response += cfg.udp_port;
    }
    if(cfg.udp_listen_port > 0) {
      if(response.length() > 0) response += ";";
      response += "UDP_LISTEN,";
      response += cfg.udp_listen_port;
    }
    if(cfg.udp6_listen_port > 0) {
      if(response.length() > 0) response += ";";
      response += "UDP6_LISTEN,";
      response += cfg.udp6_listen_port;
    }
    if(cfg.udp_send_port > 0 && strlen(cfg.udp_send_ip) > 0) {
      if(response.length() > 0) response += ";";
      response += "UDP_SEND,";
      response += cfg.udp_send_ip;
      response += ",";
      response += cfg.udp_send_port;
    }
    #endif
    return AT_R_S(response);
  } else if(p = at_cmd_check("AT+NETCONF=", atcmdline, cmd_len)) {
    // Parse format: (protocol,host,port) or multiple configs separated by ;
    // Examples:
    //   AT+NETCONF=(TCP,192.168.1.100,8080)
    //   AT+NETCONF=(UDP,192.168.1.200,9090)
    //   AT+NETCONF=(TCP,192.168.1.100,8080);(UDP,192.168.1.200,9090)
    //   AT+NETCONF=(UDP_LISTEN,9090)
    //   AT+NETCONF=(UDP_SEND,192.168.1.100,9090)
    //   AT+NETCONF= (empty to disable all)

    if(strlen(p) == 0) {
      // Empty string means disable all network connections
      #ifdef SUPPORT_TCP
      cfg.tcp_port = 0;
      memset(cfg.tcp_host_ip, 0, sizeof(cfg.tcp_host_ip));
      #endif
      #ifdef SUPPORT_TCP_SERVER
      cfg.tcp_server_port = 0;
      cfg.tcp6_server_port = 0;
      #endif
      #ifdef SUPPORT_UDP
      cfg.udp_port = 0;
      cfg.udp_listen_port = 0;
      cfg.udp6_listen_port = 0;
      memset(cfg.udp_host_ip, 0, sizeof(cfg.udp_host_ip));
      cfg.udp_send_port = 0;
      memset(cfg.udp_send_ip, 0, sizeof(cfg.udp_send_ip));
      #endif
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }

    // Reset current configurations
    #ifdef SUPPORT_TCP
    cfg.tcp_port = 0;
    memset(cfg.tcp_host_ip, 0, sizeof(cfg.tcp_host_ip));
    #endif
    #ifdef SUPPORT_TCP_SERVER
    cfg.tcp_server_port = 0;
    #endif
    #ifdef SUPPORT_UDP
    cfg.udp_port = 0;
    cfg.udp_listen_port = 0;
    memset(cfg.udp_host_ip, 0, sizeof(cfg.udp_host_ip));
    cfg.udp_send_port = 0;
    memset(cfg.udp_send_ip, 0, sizeof(cfg.udp_send_ip));
    #endif

    // Parse configuration string
    char *config_str = strdup(p);
    if (!config_str) {
      return AT_R("+ERROR: memory allocation failed");
    }
    char *saveptr1, *saveptr2;
    char *config_token = strtok_r(config_str, ";", &saveptr1);

    while(config_token != NULL) {
      // Remove leading/trailing whitespace
      while(*config_token == ' ') config_token++;
      char *end = config_token + strlen(config_token) - 1;
      while(end > config_token && *end == ' ') end--;
      *(end + 1) = '\0';

      // Check for parentheses format
      if(*config_token != '(' || *(config_token + strlen(config_token) - 1) != ')') {
        free(config_str);
        return AT_R("+ERROR: invalid format, use (protocol,host,port)");
      }

      // Remove parentheses
      config_token++;
      *(config_token + strlen(config_token) - 1) = '\0';

      // Parse protocol,host,port or protocol,port for server
      char *protocol = strtok_r(config_token, ",", &saveptr2);
      char *host_or_port = strtok_r(NULL, ",", &saveptr2);
      char *port_str = strtok_r(NULL, ",", &saveptr2);

      if(!protocol || !host_or_port) {
        free(config_str);
        return AT_R("+ERROR: invalid format, use (protocol,host,port) or (TCP_SERVER,port)");
      }

      // Remove whitespace
      while(*protocol == ' ') protocol++;
      while(*host_or_port == ' ') host_or_port++;
      if(port_str) while(*port_str == ' ') port_str++;

      // Handle TCP_SERVER case (only needs port)
      if(strcasecmp(protocol, "TCP_SERVER") == 0) {
        #ifdef SUPPORT_TCP_SERVER
        uint16_t server_port = (uint16_t)strtol(host_or_port, &r, 10);
        if(server_port == 0 || errno != 0 || server_port > 65535 || (r == host_or_port)) {
          free(config_str);
          return AT_R("+ERROR: invalid TCP server port number");
        }
        cfg.tcp_server_port = server_port;
        #else
        free(config_str);
        return AT_R("+ERROR: TCP_SERVER not supported");
        #endif
      } else if(strcasecmp(protocol, "TCP6_SERVER") == 0) {
        #ifdef SUPPORT_TCP_SERVER
        uint16_t server_port = (uint16_t)strtol(host_or_port, &r, 10);
        if(server_port == 0 || errno != 0 || server_port > 65535 || (r == host_or_port)) {
          free(config_str);
          return AT_R("+ERROR: invalid TCP6 server port number");
        }
        cfg.tcp6_server_port = server_port;
        #else
        free(config_str);
        return AT_R("+ERROR: TCP6_SERVER not supported");
        #endif
      } else {
        // Regular client protocols need host and port
        if(!port_str) {
          free(config_str);
          return AT_R("+ERROR: invalid format, use (protocol,host,port)");
        }

        char *host = host_or_port;
        uint16_t port = (uint16_t)strtol(port_str, &r, 10);
        if(port == 0 || errno != 0 || port > 65535 || (r == port_str)) {
          free(config_str);
          return AT_R("+ERROR: invalid port number");
        }

        if(strlen(host) >= 40) {
          free(config_str);
          return AT_R("+ERROR: host too long (>=40 chars)");
        }

        // Validate IP address
        IPAddress tst;
        if(!tst.fromString(host)) {
          free(config_str);
          return AT_R("+ERROR: invalid host IP address");
        }

        // Set configuration based on protocol
        if(strcasecmp(protocol, "TCP") == 0) {
          #ifdef SUPPORT_TCP
          cfg.tcp_port = port;
          strncpy(cfg.tcp_host_ip, host, 40-1);
          cfg.tcp_host_ip[40-1] = '\0';
          #else
          free(config_str);
          return AT_R("+ERROR: TCP not supported");
          #endif
        } else if(strcasecmp(protocol, "UDP") == 0) {
          #ifdef SUPPORT_UDP
          cfg.udp_port = port;
          strncpy(cfg.udp_host_ip, host, 40-1);
          cfg.udp_host_ip[40-1] = '\0';
          #else
          free(config_str);
          return AT_R("+ERROR: UDP not supported");
          #endif
        } else if(strcasecmp(protocol, "UDP_LISTEN") == 0) {
          #ifdef SUPPORT_UDP
          cfg.udp_listen_port = port;
          #else
          free(config_str);
          return AT_R("+ERROR: UDP_LISTEN not supported");
          #endif
        } else if(strcasecmp(protocol, "UDP6_LISTEN") == 0) {
          #ifdef SUPPORT_UDP
          cfg.udp6_listen_port = port;
          #else
          free(config_str);
          return AT_R("+ERROR: UDP6_LISTEN not supported");
          #endif
        } else if(strcasecmp(protocol, "UDP_SEND") == 0) {
          #ifdef SUPPORT_UDP
          cfg.udp_send_port = port;
          strncpy(cfg.udp_send_ip, host, 40-1);
          cfg.udp_send_ip[40-1] = '\0';
          #else
          free(config_str);
          return AT_R("+ERROR: UDP_SEND not supported");
          #endif
        } else if(strcasecmp(protocol, "TCP_SERVER") == 0) {
          #ifdef SUPPORT_TCP_SERVER
          cfg.tcp_server_port = port;
          #else
          free(config_str);
          return AT_R("+ERROR: TCP_SERVER not supported");
          #endif
        } else if(strcasecmp(protocol, "TCP6_SERVER") == 0) {
          #ifdef SUPPORT_TCP_SERVER
          cfg.tcp6_server_port = port;
          #else
          free(config_str);
          return AT_R("+ERROR: TCP6_SERVER not supported");
          #endif
        } else {
          free(config_str);
          return AT_R("+ERROR: invalid protocol, use TCP, UDP, UDP_LISTEN, UDP6_LISTEN, UDP_SEND, TCP_SERVER or TCP6_SERVER");
        }
      }

      config_token = strtok_r(NULL, ";", &saveptr1);
    }

    free(config_str);
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  #endif // SUPPORT_UDP || SUPPORT_TCP
  #ifdef SUPPORT_UDP
  } else if(p = at_cmd_check("AT+UDP_SEND?", atcmdline, cmd_len)) {
    if(cfg.udp_send_port == 0 || strlen(cfg.udp_send_ip) == 0)
      return AT_R("+ERROR: UDP send not configured");
    String response = String(cfg.udp_send_ip) + ":" + String(cfg.udp_send_port);
    return AT_R_S(response);
  } else if(p = at_cmd_check("AT+UDP_SEND=", atcmdline, cmd_len)) {
    if(strlen(p) == 0) {
      // Empty string means disable UDP send
      LOG("[AT] Disabling UDP send configuration");
      cfg.udp_send_port = 0;
      memset(cfg.udp_send_ip, 0, sizeof(cfg.udp_send_ip));
      cfg.udp_send_ip[0] = '\0';
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    // Expect format ip:port
    LOG("[AT] Setting UDP send configuration to: %s", p);
    char *sep = strchr(p, ':');
    if(!sep)
      return AT_R("+ERROR: invalid format, use ip:port");
    *sep = '\0';
    char *ip_str = p;
    char *port_str = sep + 1;
    if(strlen(ip_str) >= 40)
      return AT_R("+ERROR: invalid udp send ip (too long, >=40)");
    uint16_t port = (uint16_t)strtol(port_str, &r, 10);
    if(port == 0 || errno != 0 || port > 65535 || (r == port_str))
      return AT_R("+ERROR: invalid udp send port");
    IPAddress tst;
    if(!tst.fromString(ip_str))
      return AT_R("+ERROR: invalid udp send ip");
    LOG("[AT] UDP send IP and port valid: %s:%d", ip_str, port);
    // Accept IPv4 or IPv6 string
    strncpy(cfg.udp_send_ip, ip_str, 40-1);
    cfg.udp_send_ip[40-1] = '\0';
    cfg.udp_send_port = port;
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UDP_LISTEN_PORT?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.udp_listen_port);
  } else if(p = at_cmd_check("AT+UDP_LISTEN_PORT=", atcmdline, cmd_len)) {
    if(strlen(p) == 0) {
      // Empty string means disable UDP
      cfg.udp_listen_port = 0;
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_udp_port = (uint16_t)strtol(p, &r, 10);
    if(new_udp_port == 0 || errno != 0 || new_udp_port > 65535 || (r == p))
      return AT_R("+ERROR: invalid UDP port");
    if(new_udp_port != cfg.udp_listen_port) {
      cfg.udp_listen_port = new_udp_port;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UDP6_LISTEN_PORT?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.udp6_listen_port);
  } else if(p = at_cmd_check("AT+UDP6_LISTEN_PORT=", atcmdline, cmd_len)) {
    if(strlen(p) == 0) {
      // Empty string means disable UDP6
      cfg.udp6_listen_port = 0;
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_udp6_port = (uint16_t)strtol(p, &r, 10);
    if(new_udp6_port == 0 || errno != 0 || new_udp6_port > 65535 || (r == p))
      return AT_R("+ERROR: invalid UDP6 port");
    if(new_udp6_port != cfg.udp6_listen_port) {
      cfg.udp6_listen_port = new_udp6_port;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UDP_PORT?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.udp_port);
  } else if(p = at_cmd_check("AT+UDP_PORT=", atcmdline, cmd_len)) {
    if(strlen(p) == 0) {
      // Empty string means disable UDP
      cfg.udp_port = 0;
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_udp_port = (uint16_t)strtol(p, &r, 10);
    if(new_udp_port == 0 || errno != 0 || new_udp_port > 65535 || (r == p))
      return AT_R("+ERROR: invalid UDP port");
    if(new_udp_port != cfg.udp_port) {
      cfg.udp_port = new_udp_port;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UDP_HOST_IP?", atcmdline, cmd_len)) {
    return AT_R_STR(cfg.udp_host_ip);
  } else if(p = at_cmd_check("AT+UDP_HOST_IP=", atcmdline, cmd_len)) {
    if(strlen(p) >= 40)
      return AT_R("+ERROR: invalid udp host ip (too long, >=40)");
    if(strlen(p) == 0) {
      // Empty string means disable UDP
      memset(cfg.udp_host_ip, 0, sizeof(cfg.udp_host_ip));
      cfg.udp_host_ip[0] = '\0';
    } else {
      IPAddress tst;
      if(!tst.fromString(p))
        return AT_R("+ERROR: invalid udp host ip");
      // Accept IPv4 or IPv6 string
      strncpy(cfg.udp_host_ip, p, 40-1);
      cfg.udp_host_ip[40-1] = '\0';
    }
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  #endif // SUPPORT_UDP
  #ifdef SUPPORT_TCP
  } else if(p = at_cmd_check("AT+TCP_PORT?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.tcp_port);
  } else if(p = at_cmd_check("AT+TCP_PORT=", atcmdline, cmd_len)) {
    if(strlen(p) == 0) {
      // Empty string means disable TCP
      cfg.tcp_port = 0;
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_tcp_port = (uint16_t)strtol(p, &r, 10);
    if(new_tcp_port == 0 || errno != 0 || new_tcp_port > 65535 || (r == p))
      return AT_R("+ERROR: invalid TCP port");
    if(new_tcp_port != cfg.tcp_port) {
      cfg.tcp_port = new_tcp_port;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TCP_HOST_IP?", atcmdline, cmd_len)) {
    return AT_R_STR(cfg.tcp_host_ip);
  } else if(p = at_cmd_check("AT+TCP_HOST_IP=", atcmdline, cmd_len)) {
    if(strlen(p) >= 40)
      return AT_R("+ERROR: invalid tcp host ip (too long, >=40)");
    if(strlen(p) == 0) {
      // Empty string means disable TCP
      memset(cfg.tcp_host_ip, 0, sizeof(cfg.tcp_host_ip));
      cfg.tcp_host_ip[0] = '\0';
    } else {
      IPAddress tst;
      if(!tst.fromString(p))
        return AT_R("+ERROR: invalid tcp host ip");
      // Accept IPv4 or IPv6 string
      strncpy(cfg.tcp_host_ip, p, 40-1);
      cfg.tcp_host_ip[40-1] = '\0';
    }
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  #endif // SUPPORT_TCP
  #ifdef SUPPORT_TCP_SERVER
  } else if(p = at_cmd_check("AT+TCP_SERVER_PORT?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.tcp_server_port);
  } else if(p = at_cmd_check("AT+TCP_SERVER_PORT=", atcmdline, cmd_len)) {
    if(strlen(p) == 0) {
      // Empty string means disable TCP server
      cfg.tcp_server_port = 0;
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_tcp_server_port = (uint16_t)strtol(p, &r, 10);
    if(new_tcp_server_port == 0 || errno != 0 || new_tcp_server_port > 65535 || (r == p))
      return AT_R("+ERROR: invalid TCP server port");
    if(new_tcp_server_port != cfg.tcp_server_port) {
      cfg.tcp_server_port = new_tcp_server_port;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TCP_SERVER_MAX_CLIENTS?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.tcp_server_max_clients);
  } else if(p = at_cmd_check("AT+TCP_SERVER_MAX_CLIENTS=", atcmdline, cmd_len)) {
    uint8_t new_max_clients = (uint8_t)strtol(p, &r, 10);
    if(new_max_clients == 0 || errno != 0 || new_max_clients > 8 || (r == p))
      return AT_R("+ERROR: invalid max clients (1-8)");
    if(new_max_clients != cfg.tcp_server_max_clients) {
      cfg.tcp_server_max_clients = new_max_clients;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TCP_SERVER_STATUS?", atcmdline, cmd_len)) {
    if(tcp_server_sock != -1) {
      String response = "";
      response += "ACTIVE,port=";
      response += cfg.tcp_server_port;
      response += ",clients=";
      response += get_tcp_server_client_count();
      response += "/";
      response += cfg.tcp_server_max_clients;
      return AT_R_S(response);
    } else {
      return AT_R("INACTIVE");
    }
  } else if(p = at_cmd_check("AT+TCP_SERVER_START", atcmdline, cmd_len)) {
    if(cfg.tcp_server_port == 0)
      return AT_R("+ERROR: TCP server port not configured");
    stop_tcp_servers();
    start_tcp_servers();
    if(tcp_server_sock != -1)
      return AT_R_OK;
    else
      return AT_R("+ERROR: failed to start TCP server");
  } else if(p = at_cmd_check("AT+TCP_SERVER_STOP", atcmdline, cmd_len)) {
    stop_tcp_servers();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TCP_SERVER_SEND=", atcmdline, cmd_len)) {
    if(tcp_server_sock == -1)
      return AT_R("+ERROR: TCP server not active");
    int clients_sent = send_tcp_server_data((const uint8_t*)p, strlen(p));
    if(clients_sent > 0) {
      String response = "SENT to ";
      response += clients_sent;
      response += " clients";
      return AT_R_S(response);
    } else {
      return AT_R("+ERROR: no connected clients");
    }
  #endif // SUPPORT_TCP_SERVER
  } else if(p = at_cmd_check("AT+HOSTNAME=", atcmdline, cmd_len)) {
    if(strlen(p) > 63)
      return AT_R("+ERROR: hostname max 63 chars");
    strncpy((char *)&cfg.hostname, p, strlen(p));
    cfg.hostname[sizeof(cfg.hostname) - 1] = '\0';
    CFG_SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+HOSTNAME?", atcmdline, cmd_len)) {
    if(strlen(cfg.hostname) == 0)
      return AT_R_STR(DEFAULT_HOSTNAME);
    else
      return AT_R_STR(cfg.hostname);
  #ifdef SUPPORT_MDNS
  } else if(p = at_cmd_check("AT+MDNS=", atcmdline, cmd_len)) {
    uint8_t enable_mdns = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || (enable_mdns != 0 && enable_mdns != 1) || (r == p))
      return AT_R("+ERROR: use 0 or 1");
    cfg.mdns_enabled = enable_mdns;
    CFG_SAVE();
    if(WiFi.status() == WL_CONNECTED) {
      if(cfg.mdns_enabled) {
        setup_mdns();
      } else {
        stop_mdns();
      }
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+MDNS?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.mdns_enabled);
  } else if(p = at_cmd_check("AT+MDNS_HOSTNAME=", atcmdline, cmd_len)) {
    if(strlen(p) > 63)
      return AT_R("+ERROR: mDNS hostname max 63 chars");
    strncpy((char *)&cfg.mdns_hostname, p, strlen(p));
    cfg.mdns_hostname[sizeof(cfg.mdns_hostname) - 1] = '\0';
    CFG_SAVE();
    if(WiFi.status() == WL_CONNECTED && cfg.mdns_enabled) {
      stop_mdns();
      setup_mdns();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+MDNS_HOSTNAME?", atcmdline, cmd_len)) {
    if(strlen(cfg.mdns_hostname) == 0) {
      if(strlen(cfg.hostname) == 0)
        return AT_R_STR(DEFAULT_HOSTNAME);
      else
        return AT_R_STR(cfg.hostname);
    } else {
      return AT_R_STR(cfg.mdns_hostname);
    }
  #endif // SUPPORT_MDNS
  } else if(p = at_cmd_check("AT+IPV4=", atcmdline, cmd_len)) {
    String params = String(p);
    params.trim();

    if(params.equalsIgnoreCase("DHCP")) {
      // Enable IPv4 DHCP
      cfg.ip_mode = (cfg.ip_mode & ~IPV4_STATIC) | IPV4_DHCP;
      memset(cfg.ipv4_addr, 0, sizeof(cfg.ipv4_addr));
      memset(cfg.ipv4_gw, 0, sizeof(cfg.ipv4_gw));
      memset(cfg.ipv4_mask, 0, sizeof(cfg.ipv4_mask));
      memset(cfg.ipv4_dns, 0, sizeof(cfg.ipv4_dns));
    } else if(params.equalsIgnoreCase("DISABLE")) {
      // Disable IPv4
      cfg.ip_mode &= ~(IPV4_DHCP | IPV4_STATIC);
      memset(cfg.ipv4_addr, 0, sizeof(cfg.ipv4_addr));
      memset(cfg.ipv4_gw, 0, sizeof(cfg.ipv4_gw));
      memset(cfg.ipv4_mask, 0, sizeof(cfg.ipv4_mask));
      memset(cfg.ipv4_dns, 0, sizeof(cfg.ipv4_dns));
    } else {
      // Parse static IPv4: ip,netmask,gateway[,dns]
      int commaPos1 = params.indexOf(',');
      int commaPos2 = params.indexOf(',', commaPos1 + 1);
      int commaPos3 = params.indexOf(',', commaPos2 + 1);

      if(commaPos1 == -1 || commaPos2 == -1)
        return AT_R("+ERROR: IPv4 options: DHCP, DISABLE, or ip,netmask,gateway[,dns]");

      String ip = params.substring(0, commaPos1);
      String netmask = params.substring(commaPos1 + 1, commaPos2);
      String gateway = params.substring(commaPos2 + 1, commaPos3 == -1 ? params.length() : commaPos3);
      String dns = commaPos3 == -1 ? DEFAULT_DNS_IPV4 : params.substring(commaPos3 + 1);

      // Parse IP addresses
      if(!ip.length() || !netmask.length() || !gateway.length())
        return AT_R("+ERROR: missing ip, netmask, or gateway");

      // Parse and validate IP address
      int ip_parts[4], mask_parts[4], gw_parts[4], dns_parts[4];
      if(sscanf(ip.c_str(), "%d.%d.%d.%d", &ip_parts[0], &ip_parts[1], &ip_parts[2], &ip_parts[3]) != 4 ||
         sscanf(netmask.c_str(), "%d.%d.%d.%d", &mask_parts[0], &mask_parts[1], &mask_parts[2], &mask_parts[3]) != 4 ||
         sscanf(gateway.c_str(), "%d.%d.%d.%d", &gw_parts[0], &gw_parts[1], &gw_parts[2], &gw_parts[3]) != 4 ||
         sscanf(dns.c_str(), "%d.%d.%d.%d", &dns_parts[0], &dns_parts[1], &dns_parts[2], &dns_parts[3]) != 4) {
        return AT_R("+ERROR: invalid IP address format");
      }

      // Validate IP ranges (0-255)
      for(int i = 0; i < 4; i++) {
        if(ip_parts[i] < 0 || ip_parts[i] > 255 || mask_parts[i] < 0 || mask_parts[i] > 255 ||
           gw_parts[i] < 0 || gw_parts[i] > 255 || dns_parts[i] < 0 || dns_parts[i] > 255)
          return AT_R("+ERROR: IP address parts must be 0-255");
        cfg.ipv4_addr[i] = ip_parts[i];
        cfg.ipv4_mask[i] = mask_parts[i];
        cfg.ipv4_gw[i] = gw_parts[i];
        cfg.ipv4_dns[i] = dns_parts[i];
      }

      // Enable static IPv4
      cfg.ip_mode = (cfg.ip_mode & ~IPV4_DHCP) | IPV4_STATIC;
    }

    CFG_SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+IPV4?", atcmdline, cmd_len)) {
    if(cfg.ip_mode & IPV4_DHCP) {
      return AT_R("DHCP");
    } else if(cfg.ip_mode & IPV4_STATIC) {
      String response;
      response = String(cfg.ipv4_addr[0]) + "." + String(cfg.ipv4_addr[1]) + "." +
                 String(cfg.ipv4_addr[2]) + "." + String(cfg.ipv4_addr[3]) + "," +
                 String(cfg.ipv4_mask[0]) + "." + String(cfg.ipv4_mask[1]) + "." +
                 String(cfg.ipv4_mask[2]) + "." + String(cfg.ipv4_mask[3]) + "," +
                 String(cfg.ipv4_gw[0]) + "." + String(cfg.ipv4_gw[1]) + "." +
                 String(cfg.ipv4_gw[2]) + "." + String(cfg.ipv4_gw[3]) + "," +
                 String(cfg.ipv4_dns[0]) + "." + String(cfg.ipv4_dns[1]) + "." +
                 String(cfg.ipv4_dns[2]) + "." + String(cfg.ipv4_dns[3]);
      return AT_R_S(response);
    } else {
      return AT_R("DISABLED");
    }
  } else if(p = at_cmd_check("AT+IPV6=", atcmdline, cmd_len)) {
    String params = String(p);
    params.trim();

    if(params.equalsIgnoreCase("DHCP")) {
      // Enable IPv6 DHCP
      cfg.ip_mode |= IPV6_SLAAC;
    } else if(params.equalsIgnoreCase("DISABLE")) {
      // Disable IPv6
      cfg.ip_mode &= ~IPV6_SLAAC;
    } else {
      return AT_R("+ERROR: IPv6 options: DHCP or DISABLE");
    }

    CFG_SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+IPV6?", atcmdline, cmd_len)) {
    if(cfg.ip_mode & IPV6_SLAAC)
      return AT_R("DHCP");
    else
      return AT_R("DISABLED");
  } else if(p = at_cmd_check("AT+IP_STATUS?", atcmdline, cmd_len)) {
    String response = "";
    bool hasIP = false;

    // Check WiFi connection status first
    if(WiFi.status() != WL_CONNECTED && WiFi.status() != WL_IDLE_STATUS)
      return AT_R("+ERROR: WiFi not connected");

    // IPv4 status
    IPAddress ipv4 = WiFi.localIP();
    if(ipv4 != IPAddress(0,0,0,0) && ipv4 != IPAddress(127,0,0,1)) {
      response += "IPv4: " + ipv4.toString();
      IPAddress gateway = WiFi.gatewayIP();
      IPAddress subnet = WiFi.subnetMask();
      IPAddress dns = WiFi.dnsIP();
      if(gateway != IPAddress(0,0,0,0)) {
        response += ", gw: " + gateway.toString();
      }
      if(subnet != IPAddress(0,0,0,0)) {
        response += ", nm: " + subnet.toString();
      }
      if(dns != IPAddress(0,0,0,0)) {
        response += ", dns: " + dns.toString();
      }
      hasIP = true;
    }

    // IPv6 status
    if(cfg.ip_mode & IPV6_SLAAC) {
      IPAddress ipv6_global = WiFi.globalIPv6();
      IPAddress ipv6_local = WiFi.linkLocalIPv6();

      if(ipv6_global != IPAddress((uint32_t)0)) {
        if(hasIP)
            response += "\n";
        response += "IPv6 global: " + ipv6_global.toString();
        hasIP = true;
      }

      if(ipv6_local != IPAddress((uint32_t)0)) {
        if(hasIP)
            response += "\n";
        response += "IPv6 link-local: " + ipv6_local.toString();
        hasIP = true;
      }
    }

    if(!hasIP)
      response = "No IP addresses assigned";
    return AT_R_S(response);
  #ifdef SUPPORT_TCP
  } else if(p = at_cmd_check("AT+TCP_STATUS?", atcmdline, cmd_len)) {
    if(strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0) {
      return AT_R("TCP not configured");
    } else {
      String response = "";
      response = "TCP Host: " + String(cfg.tcp_host_ip) + ":" + String(cfg.tcp_port);
      return AT_R_S(response);
    }
  #endif // SUPPORT_TCP
  #ifdef SUPPORT_TLS
  } else if(p = at_cmd_check("AT+TLS_ENABLE?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.tls_enabled);
  } else if(p = at_cmd_check("AT+TLS_ENABLE=", atcmdline, cmd_len)) {
    uint8_t new_tls_enabled = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || (new_tls_enabled != 0 && new_tls_enabled != 1) || (r == p))
      return AT_R("+ERROR: TLS enable must be 0 or 1");
    if(new_tls_enabled != cfg.tls_enabled) {
      cfg.tls_enabled = new_tls_enabled;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_PORT?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.tls_port);
  } else if(p = at_cmd_check("AT+TLS_PORT=", atcmdline, cmd_len)) {
    if(strlen(p) == 0) {
      // Empty string means use tcp_port
      cfg.tls_port = 0;
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_tls_port = (uint16_t)strtol(p, &r, 10);
    if(new_tls_port == 0 || errno != 0 || new_tls_port > 65535 || (r == p))
      return AT_R("+ERROR: invalid TLS port");
    if(new_tls_port != cfg.tls_port) {
      cfg.tls_port = new_tls_port;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_VERIFY?", atcmdline, cmd_len)) {
    const char* verify_modes[] = {"none", "optional", "required"};
    return AT_R_STR(verify_modes[cfg.tls_verify_mode]);
  } else if(p = at_cmd_check("AT+TLS_VERIFY=", atcmdline, cmd_len)) {
    uint8_t new_verify_mode = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || new_verify_mode > 2 || (r == p))
      return AT_R("+ERROR: TLS verify mode must be 0-2 (0=none, 1=optional, 2=required)");
    if(new_verify_mode != cfg.tls_verify_mode) {
      cfg.tls_verify_mode = new_verify_mode;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_SNI?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.tls_use_sni);
  } else if(p = at_cmd_check("AT+TLS_SNI=", atcmdline, cmd_len)) {
    uint8_t new_sni = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || (new_sni != 0 && new_sni != 1) || (r == p))
      return AT_R("+ERROR: TLS SNI must be 0 or 1");
    if(new_sni != cfg.tls_use_sni) {
      cfg.tls_use_sni = new_sni;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_CA_CERT=", atcmdline, cmd_len)) {
    if(strlen(p) >= 2048)
      return AT_R("+ERROR: CA certificate too long");
    if(save_tls_file_to_spiffs("/ca_cert.pem", p)) {
      reconfigure_network_connections();
      return AT_R_OK;
    } else {
      return AT_R("+ERROR: failed to save CA certificate");
    }
  } else if(p = at_cmd_check("AT+TLS_CA_CERT?", atcmdline, cmd_len)) {
    uint8_t tls_buf[2048] = {0};
    if(load_tls_file_from_spiffs("/ca_cert.pem", tls_buf, sizeof(tls_buf)))
      return AT_R_STR("SET");
    else
      return AT_R_STR("NOT_SET");
  } else if(p = at_cmd_check("AT+TLS_CLIENT_CERT=", atcmdline, cmd_len)) {
    if(strlen(p) >= 2048)
      return AT_R("+ERROR: Client certificate too long");
    if(save_tls_file_to_spiffs("/client_cert.pem", p)){
      reconfigure_network_connections();
      return AT_R_OK;
    } else {
      return AT_R("+ERROR: failed to save client certificate");
    }
  } else if(p = at_cmd_check("AT+TLS_CLIENT_CERT?", atcmdline, cmd_len)) {
    uint8_t tls_buf[2048] = {0};
    if(load_tls_file_from_spiffs("/client_cert.pem", tls_buf, sizeof(tls_buf)))
      return AT_R_STR("SET");
    else
      return AT_R_STR("NOT_SET");
  } else if(p = at_cmd_check("AT+TLS_CLIENT_KEY=", atcmdline, cmd_len)) {
    if(strlen(p) >= 2048)
      return AT_R("+ERROR: Client key too long");
    if(save_tls_file_to_spiffs("/client_key.pem", p)) {
      reconfigure_network_connections();
      return AT_R_OK;
    } else {
      return AT_R("+ERROR: failed to save client key");
    }
  } else if(p = at_cmd_check("AT+TLS_CLIENT_KEY?", atcmdline, cmd_len)) {
    uint8_t tls_buf[2048] = {0};
    if(load_tls_file_from_spiffs("/client_key.pem", tls_buf, sizeof(tls_buf)))
      return AT_R_STR("SET");
    else
      return AT_R_STR("NOT_SET");
  } else if(p = at_cmd_check("AT+TLS_STATUS?", atcmdline, cmd_len)) {
    if(!cfg.tls_enabled) {
      return AT_R("TLS disabled");
    } else if(strlen(cfg.tcp_host_ip) == 0 || (cfg.tcp_port == 0 && cfg.tls_port == 0)) {
      return AT_R("TLS not configured");
    } else {
      String response = "";
      uint16_t port_to_use = cfg.tls_port ? cfg.tls_port : cfg.tcp_port;
      response = "TLS Host: " + String(cfg.tcp_host_ip) + ":" + String(port_to_use);
      response += ", Status: " + String(tls_connected ? "connected" : "disconnected");
      if(tls_connected && tls_handshake_complete) {
        response += ", Secure: yes";
        // Note: Cipher suite info varies by platform and may not be available
      }
      return AT_R_S(response);
    }
  } else if(p = at_cmd_check("AT+TLS_CONNECT", atcmdline, cmd_len)) {
    if(!cfg.tls_enabled)
      return AT_R("+ERROR: TLS is disabled");
    if(strlen(cfg.tcp_host_ip) == 0 || (cfg.tcp_port == 0 && cfg.tls_port == 0))
      return AT_R("+ERROR: TLS host/port not configured");
    connect_tls();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_DISCONNECT", atcmdline, cmd_len)) {
    close_tls_connection();
    return AT_R_OK;
  #endif // SUPPORT_TLS
  #endif // SUPPORT_WIFI
  } else if(p = at_cmd_check("AT+ERASE", atcmdline, cmd_len)) {
    // Erase all configuration and reset to factory defaults
    LOG("[ERASE] Erasing configuration and resetting to factory defaults");

    #ifdef SUPPORT_WIFI
    // Stop all network connections before erasing config
    stop_networking();
    #endif // SUPPORT_WIFI

    // Clear WIFI config if enabled
    CFG_CLEAR();

    // Clear PLUGINS config if enabled
    #ifdef SUPPORT_PLUGINS
    if(PLUGINS::clear_config){
      LOG("[ERASE] Clearing plugins configuration");
      PLUGINS::clear_config();
    }
    #endif // SUPPORT_PLUGINS

    LOG("[ERASE] Configuration erased, clearing RTC memory");
    extern char _rtc_bss_start, _rtc_bss_end, _rtc_data_start, _rtc_data_end;
    // Clear RTC BSS (uninitialized data)
    memset(&_rtc_bss_start, 0, (&_rtc_bss_end - &_rtc_bss_start));
    // Clear RTC DATA (initialized data)
    memset(&_rtc_data_start, 0, (&_rtc_data_end - &_rtc_data_start));

    // Restart immediately
    LOG("[ERASE] Restarting after factory reset");
    LOGFLUSH();
    esp_restart();
  } else if(p = at_cmd_check("AT+RESET", atcmdline, cmd_len)) {
    LOG("[RESET] Restarting device per AT+RESET command");
    LOGFLUSH();
    esp_restart();
  } else if(p = at_cmd_check("AT+CPU_FREQ=", atcmdline, cmd_len)) {
    uint8_t freq = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || (freq < 10 || freq > 160) || (r == p))
      return AT_R("+ERROR: CPU frequency must be 10-160, steps of 10, even MHz");
    setup_cpu_speed(freq);
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+CPU_FREQ?", atcmdline, cmd_len)) {
    return AT_R_INT(getCpuFrequencyMhz());
  } else if(p = at_cmd_check("AT+HELP?", atcmdline, cmd_len)) {
    return AT_R_F(AT_help_string);
  } else if(p = at_cmd_check("AT+?", atcmdline, cmd_len)) {
    return AT_R_F(AT_short_help_string);
  #ifdef SUPPORT_PLUGINS
  } else if(p = at_cmd_check("AT+PLUGINS?", atcmdline, cmd_len)){
    if(PLUGINS::at_get_help_string){
      return AT_R_F(PLUGINS::at_get_help_string());
    } else {
      return AT_R("+ERROR: No plugins help available");
    }
  #endif // SUPPORT_PLUGINS
  #ifdef BLUETOOTH_UART_AT
  } else if(p = at_cmd_check("AT+BLE_PIN=", atcmdline, cmd_len)) {
    if(strlen(p) != 6)
      return AT_R("+ERROR: BLE PIN must be exactly 6 digits");
    uint32_t pin = strtoul(p, &r, 10); // Just to check for conversion errors
    if(errno != 0 || r == p)
      return AT_R("+ERROR: BLE PIN invalid, must be 6 digits");
    cfg.ble_pin = pin;
    CFG_SAVE();
    // Restart BLE with new PIN
    uint8_t want_advertising = (ble_advertising_start != 0);
    destroy_ble();
    setup_ble();
    if(want_advertising)
      start_advertising_ble();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+BLE_PIN?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.ble_pin);
  } else if(p = at_cmd_check("AT+BLE_SECURITY=", atcmdline, cmd_len)) {
    uint8_t mode = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || mode > 2 || (p == r))
      return AT_R("+ERROR: BLE security mode must be 0-2 (0=None, 1=PIN, 2=Bonding)");
    if(mode == 1 && cfg.ble_pin == 0)
      return AT_R("+ERROR: Cannot set security mode to PIN when PIN is 0 (no PIN). Set a valid PIN first with AT+BLE_PIN.");
    if(mode == 2 && cfg.ble_pin == 0 && cfg.ble_io_cap == 3)
      return AT_R("+ERROR: Cannot set security mode to Bonding when PIN is 0 and IO capability is NoInputNoOutput. Set a valid PIN or change IO capability first.");
    if(mode != cfg.ble_security_mode) {
      LOG("[BLE] Setting security mode to %d", mode);
      cfg.ble_security_mode = mode;
      CFG_SAVE();
      // Restart BLE with new PIN
      uint8_t want_advertising = (ble_advertising_start != 0);
      destroy_ble();
      setup_ble();
      if(want_advertising)
        start_advertising_ble();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+BLE_SECURITY?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.ble_security_mode);
  } else if(p = at_cmd_check("AT+BLE_IO_CAP=", atcmdline, cmd_len)) {
    uint8_t cap = strtol(p, &r, 10);
    if(errno != 0 || cap > 4 || (p == r))
      return AT_R("+ERROR: BLE IO capability must be 0-4 (0=DisplayOnly, 1=DisplayYesNo, 2=KeyboardOnly, 3=NoInputNoOutput, 4=KeyboardDisplay)");
    if(cap == 3 && cfg.ble_security_mode == 2 && cfg.ble_pin == 0)
      return AT_R("+ERROR: Cannot set IO capability to NoInputNoOutput when security mode is Bonding and PIN is 0. Set a valid PIN or change security mode first.");
    if(cap != cfg.ble_io_cap) {
      LOG("[BLE] Setting IO capability to %d", cap);
      cfg.ble_io_cap = cap;
      // Restart BLE with new IO capability
      uint8_t want_advertising = (ble_advertising_start != 0);
      destroy_ble();
      setup_ble();
      if(want_advertising)
        start_advertising_ble();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+BLE_IO_CAP?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.ble_io_cap);
  } else if(p = at_cmd_check("AT+BLE_AUTH_REQ=", atcmdline, cmd_len)) {
    uint8_t req = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || req > 3 || (p == r))
      return AT_R("+ERROR: BLE auth requirements must be 0-3 (0=None, 1=Bonding, 2=MITM, 3=Bonding+MITM)");
    if(req != cfg.ble_auth_req) {
      LOG("[BLE] Setting auth requirements to %d", req);
      cfg.ble_auth_req = req;
      CFG_SAVE();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+BLE_AUTH_REQ?", atcmdline, cmd_len)) {
    return AT_R_INT(cfg.ble_auth_req);
  } else if(p = at_cmd_check("AT+BLE_STATUS?", atcmdline, cmd_len)) {
    String status = "BLE: ";
    if(ble_advertising_start == 0) {
      status += "disabled";
    } else if(deviceConnected == 1) {
      status += "connected";
      if(securityRequestPending == 1) {
        status += ", security pending";
      } else {
        status += ", authenticated";
      }
      if(passkeyForDisplay != 0) {
        status += ", PIN: " + String(passkeyForDisplay, DEC);
      }
    } else {
      status += "advertising";
    }
    status += ", Security mode: " + String(cfg.ble_security_mode);
    status += ", IO cap: " + String(cfg.ble_io_cap);
    status += ", Auth req: " + String(cfg.ble_auth_req);
    status += ", Addr type: " + String(get_ble_addr_type_name(cfg.ble_addr_type));
    return AT_R_S(status);
  } else if(p = at_cmd_check("AT+BLE_ADDR_TYPE=", atcmdline, cmd_len)) {
    uint8_t type = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || (p == r && type == 0) || type > 3)
      return AT_R("+ERROR: BLE address type must be 0-3 (0=Public, 1=Random Static, 2=Private Resolvable, 3=Private Non-resolvable)");
    if(type != cfg.ble_addr_type) {
      LOG("[BLE] Setting address type to %d", type);
      // If changing from or to public address, need to restart BLE
      bool restart_ble = (type == 0 || cfg.ble_addr_type == 0);
      cfg.ble_addr_type = type;
      if(restart_ble) {
        uint8_t want_advertising = (ble_advertising_start != 0);
        destroy_ble();
        setup_ble();
        if(want_advertising)
          start_advertising_ble();
      }
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+BLE_ADDR_TYPE?", atcmdline, cmd_len)) {
    return AT_R_STR(cfg.ble_addr_type == 0 ? "0 (public)" :
                    cfg.ble_addr_type == 1 ? "1 (random static)" :
                    cfg.ble_addr_type == 2 ? "2 (private resolvable)" :
                    cfg.ble_addr_type == 3 ? "3 (private non-resolvable)" : "unknown");
  } else if(p = at_cmd_check("AT+BLE_ADDR=", atcmdline, cmd_len)) {
    // Parse MAC address in format XX:XX:XX:XX:XX:XX
    uint8_t addr[6];
    int parsed = sscanf(p, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
                       &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]);
    if(parsed != 6) {
      return AT_R("+ERROR: Invalid MAC address format. Use XX:XX:XX:XX:XX:XX");
    }

    // Validate address based on type
    if(cfg.ble_addr_type == 1) { // Random static
      if(!is_valid_static_random_address(addr)) {
        return AT_R("+ERROR: Invalid static random address. First byte must be 0xC0-0xFF");
      }
    } else if(!is_valid_ble_address(addr)) {
      return AT_R("+ERROR: Invalid address (all zeros)");
    }

    memcpy(cfg.ble_custom_addr, addr, 6);
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+BLE_ADDR?", atcmdline, cmd_len)) {
    return AT_R_S(String(get_current_ble_address()));
  } else if(p = at_cmd_check("AT+BLE_ADDR_GEN?", atcmdline, cmd_len)) {
    // Generate a new random static address
    uint8_t new_addr[6];
    generate_static_random_address(new_addr);
    memcpy(cfg.ble_custom_addr, new_addr, 6);
    cfg.ble_addr_type = 1; // Set to random static
    CFG_SAVE();

    char addr_str[26] = {0};
    snprintf(addr_str, sizeof(addr_str), "Generated: %02X:%02X:%02X:%02X:%02X:%02X",
            new_addr[0], new_addr[1], new_addr[2], new_addr[3], new_addr[4], new_addr[5]);
    return AT_R_S(String(addr_str));
  #endif // BLUETOOTH_UART_AT
  } else {
  #ifdef SUPPORT_PLUGINS
    if(PLUGINS::at_cmd_handler){
      return PLUGINS::at_cmd_handler(atcmdline);
    } else {
      return AT_R("+ERROR: unknown command");
    }
  #else
    return AT_R("+ERROR: unknown command");
  #endif // SUPPORT_PLUGINS
  }
  return AT_R("+ERROR: unknown error");
}
#endif

size_t inlen = 0;

// BLE UART Service - Nordic UART Service UUID
#if defined(BLUETOOTH_UART_AT) || defined(SUPPORT_BLE_UART1)

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;

// BLE UART buffer
ALIGN(4) char ble_cmd_buffer[512] = {0};
char * ble_cmd_ptr = &ble_cmd_buffer[0];
uint16_t ble_cmd_ready = 0;

// BLE negotiated MTU (default to AT buffer size)
#define BLE_MTU_MIN     128
#define BLE_MTU_MAX     512
#define BLE_MTU_DEFAULT 128

uint16_t ble_mtu = BLE_MTU_DEFAULT;

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      doYIELD;
      deviceConnected = 1;
      securityRequestPending = 0;
      LOG("[BLE] Device connected, MTU: %d", pServer->getPeerMTU(1));
    };

    void onDisconnect(BLEServer* pServer) {
      doYIELD;
      deviceConnected = 0;
      securityRequestPending = 0;
      passkeyForDisplay = 0;
      ble_advertising_start = 0;
      LOG("[BLE] disconnected");
    }

    // TODO: use/fix once ESP32 BLE MTU negotiation is implemented
    #if defined(ESP32) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    void onMTU(uint16_t mtu, BLEServer* /*pServer*/) {
      doYIELD;
      if (mtu < BLE_MTU_MIN) {
        LOG("[BLE] MTU request too small (%d), keeping %d", mtu, BLE_MTU_DEFAULT);
        return;
      }
      if (mtu > BLE_MTU_MAX)
          mtu = BLE_MTU_MAX;
      if (mtu > BLE_MTU_MIN) {
        ble_mtu = mtu;
        BLEDevice::setMTU(ble_mtu);
        LOG("[BLE] MTU set to: %d", ble_mtu);
      } else {
        LOG("[BLE] MTU unchanged (current: %d, requested: %d)", ble_mtu, mtu);
      }
    }
    #endif // ESP32
};

// BLE Security Callbacks
class MySecurity : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() {
    doYIELD;
    LOG("[BLE Security] PassKey Request, using static PIN: %06d", cfg.ble_pin);
    return cfg.ble_pin;
  }

  void onPassKeyNotify(uint32_t pass_key) {
    doYIELD;
    passkeyForDisplay = pass_key;
    LOG("[BLE Security] PassKey Notify: %06d", pass_key);
    // Notify via UART that PIN is being displayed
    ble_send_response(("+BLEPIN: " + String(pass_key, DEC)).c_str());
  }

  bool onConfirmPIN(uint32_t pass_key) {
    doYIELD;
    LOG("[BLE Security] Confirm PIN: %06d", pass_key);
    // Auto-confirm for now, could be enhanced with user interaction
    return true;
  }

  bool onSecurityRequest() {
    doYIELD;
    LOG("[BLE Security] Security Request");
    securityRequestPending = 1;
    return true;
  }

  void onAuthenticationComplete(ble_gap_conn_desc* desc) {
    doYIELD;
    securityRequestPending = 0;
    if (desc) {
      LOG("[BLE Security] Authentication Complete - connection handle: %d", desc->conn_handle);
      ble_send_response("+BLEAUTH: SUCCESS");
    } else {
      LOG("[BLE Security] Authentication Failed");
      ble_send_response("+BLEAUTH: FAILED");
    }
  }
};


#ifdef SUPPORT_BLE_UART1
// Temporary buffer for incoming BLE data when not in AT mode
#define BLE_UART1_READ_BUFFER_SIZE   BLE_MTU_MAX*4
ALIGN(4) uint8_t ble_rx_buffer[BLE_UART1_READ_BUFFER_SIZE] = {0};
uint16_t ble_rx_len = 0;
#endif // SUPPORT_BLE_UART1

// BLE Characteristic Callbacks
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pC) {
      static uint8_t ble_on_data_processing = 0;

      // Handle data written to BLE RX characteristic
      // check processing flag
      if(ble_on_data_processing == 1) {
        D("[BLE] onWrite called while processing previous data, ignoring new data");
        return;
      }
      ble_on_data_processing = 1;

      String v = pC->getValue(); // get value written to characteristic
      size_t b_len = v.length(); // get length of data written
      pC->setValue("");          // clear characteristic value
      if(b_len == 0) {
        ble_on_data_processing = 0;
        return;
      }

      uint8_t* ble_rx_buf = (uint8_t*)v.c_str();

      #ifdef SUPPORT_BLE_UART1
      // When NOT in AT mode, store data in buffer for later
      // use (e.g. UART1 bridge)
      if(cfg.ble_uart1_bridge == 1 && at_mode == BRIDGE_MODE) {
        if(ble_rx_len >= BLE_UART1_READ_BUFFER_SIZE) {
          // Buffer full, drop incoming data
          D("[BLE] RX buffer full, dropping data, got %d bytes, buffer size: %d", b_len, BLE_UART1_READ_BUFFER_SIZE);
          ble_on_data_processing = 0;
          return;
        } else {
          D("[BLE] in AT bridge mode, keeping data in buffer");
          b_len = min(b_len, (size_t)(BLE_UART1_READ_BUFFER_SIZE - ble_rx_len));
          memcpy(ble_rx_buffer + ble_rx_len, ble_rx_buf, b_len);
          ble_rx_len += b_len;
          ble_on_data_processing = 0;
          return;
        }
      }
      #endif

      // when in AT command mode, process incoming data
      b_len--;
      uint8_t ble_rx_local[b_len+1] = {0};
      memcpy(ble_rx_local, ble_rx_buf, b_len);
      ble_rx_buf = NULL;
      ble_rx_buf = &ble_rx_local[0];
      ble_rx_buf[b_len] = 0; // null-terminate for logging
      D("[BLE] RX LEN: %d,BUF>>%s<<", b_len, ble_rx_buf);

      // When in AT command mode, add data to command buffer
      // and check \n or \r terminators
      // Process each byte individually to handle command terminators
      // properly
      char *ble_str = ble_cmd_ptr;
      char *ble_last = ble_str;
      char *ble_cmd_max = ble_cmd_buffer + sizeof(ble_cmd_buffer) - 2;
      char next_c = (char)(*ble_rx_buf);
      while(next_c != 0 && b_len-- > 0){

        // Check for buffer overflow
        if(ble_cmd_ptr >= ble_cmd_max) {
          // reset last buffer pointer
          memset(ble_last, 0, ble_cmd_max + 2 - ble_last);
          ble_cmd_ptr = ble_last;
          // send error
          LOG("[BLE] Command buffer overflow");
          ble_send_response("+ERROR: BLE command too long");
          ble_on_data_processing = 0;
          return;
        }

        // set to 0, and advance buffer pointer
        *ble_rx_buf++ = 0;

        // Verify character for command termination
        D("[BLE] RX CHAR: %02X '%c', ptr: %p, ptr_last: %p", next_c, isprint(next_c) ? next_c : '.', ble_cmd_ptr, ble_last);
        if(next_c == '\r' || next_c == '\n') {
          ble_rx_buf++;   // don't include \n in command string
          *ble_cmd_ptr++ = 0; // null-terminate command string, advance pointer
          ble_last = ble_cmd_ptr; // save last pointer position
          D("[BLE] Command Ready: %d, %s, nr: %d", strlen(ble_str), ble_str, 1+ble_cmd_ready);
          ble_cmd_ready++;
        } else {
          // Add character to command buffer, advance pointer of destination
          *ble_cmd_ptr++ = next_c;
        }

        // Get next character
        next_c = (char)(*ble_rx_buf);
      }

      // Clear processing flag
      D("[BLE] Processed BLE writes");
      ble_on_data_processing = 0;
      return;
    }
};

// BLE MAC Address utility functions
NOINLINE
bool is_valid_ble_address(const uint8_t* addr) {
  // Check if address is all zeros (invalid)
  for(int i = 0; i < 6; i++) {
    if(addr[i] != 0) return true;
  }
  return false;
}

NOINLINE
bool is_valid_static_random_address(const uint8_t* addr) {
  // Static random addresses must have the two most significant bits set to '11'
  // This means the first byte (MSB) must be in range 0xC0-0xFF
  return (addr[0] & 0xC0) == 0xC0 && is_valid_ble_address(addr);
}

NOINLINE
void generate_static_random_address(uint8_t* addr) {
  // Generate a valid static random address
  // First byte must have MSBs = 11 (0xC0-0xFF)
  addr[0] = 0xC0 + (esp_random() & 0x3F);

  // Remaining 5 bytes can be any value except all zeros
  for(int i = 1; i < 6; i++) {
    addr[i] = esp_random() & 0xFF;
  }

  // Ensure address is not all zeros (except for the fixed MSBs)
  uint32_t sum = 0;
  for(int i = 1; i < 6; i++) {
    sum += addr[i];
  }

  // If all random bytes are zero, set the last one to 1
  if(sum == 0) {
    addr[5] = 1;
  }
}

NOINLINE
const char* get_ble_addr_type_name(uint8_t type) {
  switch(type) {
    case 0: return "Public";
    case 1: return "Random Static";
    case 2: return "Private Resolvable";
    case 3: return "Private Non-resolvable";
    default: return "Unknown";
  }
}

NOINLINE
void setup_ble_address() {
  LOG("[BLE] Configuring MAC address (type: %s)", get_ble_addr_type_name(cfg.ble_addr_type));

  switch(cfg.ble_addr_type) {
    case 0: // Public address
      if(is_valid_ble_address(cfg.ble_custom_addr)) {
        // Try to set custom public address using NimBLE
        ble_addr_t addr;
        addr.type = BLE_ADDR_PUBLIC;
        memcpy(addr.val, cfg.ble_custom_addr, 6);

        int rc = ble_hs_id_set_rnd(addr.val);
        if(rc == 0) {
          LOG("[BLE] Set custom public address: %02X:%02X:%02X:%02X:%02X:%02X",
              cfg.ble_custom_addr[0], cfg.ble_custom_addr[1], cfg.ble_custom_addr[2],
              cfg.ble_custom_addr[3], cfg.ble_custom_addr[4], cfg.ble_custom_addr[5]);
        } else {
          LOG("[BLE] Failed to set custom public address (error: %d), using default", rc);
        }
      } else {
        LOG("[BLE] Using default public address");
      }
      break;

    case 1: // Random static address
      {
        uint8_t static_addr[6];
        bool address_set = false;

        if(is_valid_static_random_address(cfg.ble_custom_addr)) {
          // Use provided static random address
          memcpy(static_addr, cfg.ble_custom_addr, 6);
          LOG("[BLE] Using configured static random address");
        } else if(cfg.ble_addr_auto_random) {
          // Auto-generate static random address
          generate_static_random_address(static_addr);
          LOG("[BLE] Generated new static random address");
          // Save the generated address back to config
          memcpy(cfg.ble_custom_addr, static_addr, 6);
        } else {
          LOG("[BLE] Invalid static random address provided and auto-generation disabled");
          return;
        }

        // Set the random static address using NimBLE API
        int rc = ble_hs_id_set_rnd(static_addr);
        if(rc == 0) {
          LOG("[BLE] Set static random address: %02X:%02X:%02X:%02X:%02X:%02X",
              static_addr[0], static_addr[1], static_addr[2],
              static_addr[3], static_addr[4], static_addr[5]);
          address_set = true;
        } else {
          LOG("[BLE] Failed to set static random address (error: %d)", rc);
        }

        if(!address_set) {
          LOG("[BLE] Falling back to default address generation");
        }
      }
      break;

    case 2: // Private resolvable address
      LOG("[BLE] Private resolvable addresses require IRK setup");
      // Enable privacy mode with resolvable addresses
      // This would typically involve setting up an IRK and enabling privacy
      LOG("[BLE] Privacy mode not fully implemented yet");
      break;

    case 3: // Private non-resolvable address
      LOG("[BLE] Private non-resolvable addresses change automatically");
      // These addresses are handled automatically by the BLE stack
      break;

    default:
      LOG("[BLE] Unknown address type %d, using default public address", cfg.ble_addr_type);
      break;
  }
}

NOINLINE
char * get_current_ble_address() {
  ALIGN(4) static char addr_str[64] = {0};
  memset(addr_str, 0, sizeof(addr_str));

  // Try to get the actual address being used by the BLE stack
  int pr = 0;
  ble_addr_t addr;
  int rc = ble_hs_id_copy_addr(BLE_ADDR_RANDOM, addr.val, NULL);
  if(rc == 0) {
    // Successfully got random address from stack
    pr = snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X (active random)",
            addr.val[5], addr.val[4], addr.val[3], addr.val[2], addr.val[1], addr.val[0]);
  } else {
    // Try to get public address
    rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr.val, NULL);
    if(rc == 0) {
      pr = snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X (active public)",
              addr.val[5], addr.val[4], addr.val[3], addr.val[2], addr.val[1], addr.val[0]);
    } else {
      // Fallback to configured address
      if(is_valid_ble_address(cfg.ble_custom_addr)) {
        pr = snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X (configured)",
                cfg.ble_custom_addr[0], cfg.ble_custom_addr[1], cfg.ble_custom_addr[2],
                cfg.ble_custom_addr[3], cfg.ble_custom_addr[4], cfg.ble_custom_addr[5]);
      } else {
        pr = snprintf(addr_str, sizeof(addr_str), "Default address (type: %s)", get_ble_addr_type_name(cfg.ble_addr_type));
      }
    }
  }

  return addr_str;
}

// called from AT command handler when changes are made
NOINLINE
void destroy_ble() {
  if(ble_advertising_start != 0) {
    ble_advertising_start = 0;
    deviceConnected = 0;
    securityRequestPending = 0;
    passkeyForDisplay = 0;
    BLEDevice::deinit(false);
    delay(100);
    LOG("[BLE] Deinitialized");
  }
}

void log_base_bt_mac() {
  uint8_t base_mac_addr[6] = {0};
  esp_err_t ret = esp_base_mac_addr_get(base_mac_addr);
  if (ret == ESP_OK) {
    LOG("[BT] Base MAC address: %02X:%02X:%02X:%02X:%02X:%02X", 
        base_mac_addr[0], base_mac_addr[1], base_mac_addr[2], 
        base_mac_addr[3], base_mac_addr[4], base_mac_addr[5]);
  } else {
    LOG("[BT] Failed to get base MAC address: %s", esp_err_to_name(ret));
  }

  // Also try to get BT-specific MAC
  uint8_t bt_mac_addr[6] = {0};
  ret = esp_read_mac(bt_mac_addr, ESP_MAC_BT);
  if (ret == ESP_OK) {
    LOG("[BT] Derived BT MAC address: %02X:%02X:%02X:%02X:%02X:%02X", 
        bt_mac_addr[0], bt_mac_addr[1], bt_mac_addr[2], 
        bt_mac_addr[3], bt_mac_addr[4], bt_mac_addr[5]);
  } else {
    LOG("[BT] Failed to get derived BT MAC address: %s", esp_err_to_name(ret));
  }
}

NOINLINE
void setup_ble() {
  LOG("[BLE] Setup");

  // Log base Bluetooth MAC addresses
  log_base_bt_mac();

  // Create the BLE Device
  BLEDevice::init(BLUETOOTH_UART_DEVICE_NAME);
  BLEDevice::setMTU(ble_mtu); // Request MTU matching AT buffer size

  // Configure BLE MAC address after BLE init but before services
  setup_ble_address();

  // Log the actual BLE MAC address being used
  char *current_ble_addr = get_current_ble_address();
  LOG("[BLE] Active MAC address: %s", current_ble_addr);

  // Configure BLE Security based on configuration
  if(cfg.ble_security_mode > 0) {
    LOG("[BLE] Security mode %d requested", cfg.ble_security_mode);

    // Set security callbacks for NimBLE
    BLEDevice::setSecurityCallbacks(new MySecurity());

    // Log PIN configuration
    if(cfg.ble_security_mode == 1) {
      LOG("[BLE] Security: PIN mode with static PIN: %06d", cfg.ble_pin);
    } else if(cfg.ble_security_mode == 1) {
      LOG("[BLE] Security: PIN mode with dynamic PIN");
    } else {
      LOG("[BLE] Security: Basic encryption enabled");
    }
  } else {
    LOG("[BLE] Security: None");
  }

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  LOG("[BLE] Server created");
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  pService = pServer->createService(SERVICE_UUID);
  LOG("[BLE] Service created");

  // Create a BLE Characteristic for TX (notifications to client)
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  LOG("[BLE] TX Characteristic created");

  pTxCharacteristic->addDescriptor(new BLE2902());
  LOG("[BLE] TX Descriptor added");

  // Create a BLE Characteristic for RX (writes from client)
  pRxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );
  LOG("[BLE] RX Characteristic created");

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();
  LOG("[BLE] Service started");

  // advertising config
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  // set value to 0x00 to not advertise this parameter
  pAdvertising->setMinPreferred(0x0);

  // don't start advertising
  ble_advertising_start = 0;
  deviceConnected = 0;

  LOG("[BLE] Setup complete");
}

NOINLINE
void handle_ble_commands(){
  if(deviceConnected == 0 || ble_cmd_ready == 0)
    return;
  LOG("[BLE] Handling %d pending commands", ble_cmd_ready);

  char * ble_rd_ptr = &ble_cmd_buffer[0];
  size_t cmd_len = strlen(ble_rd_ptr);
  while(ble_rd_ptr < (ble_cmd_buffer + sizeof(ble_cmd_buffer)) && cmd_len > 0 && ble_cmd_ready > 0) {
    handle_ble_command(ble_rd_ptr, cmd_len);
    memset(ble_rd_ptr, 0, cmd_len);
    ble_rd_ptr += cmd_len + 1;
    cmd_len = strlen(ble_rd_ptr);
    ble_cmd_ready--;
  }
  if(ble_cmd_ptr && ble_rd_ptr == ble_cmd_ptr)
    // all commands processed, reset pointer
    ble_cmd_ptr = &ble_cmd_buffer[0];
}

NOINLINE
void handle_ble_command(char * cmd, size_t cmd_len) {
  D("[BLE] Processing command: %d, size:%d, buf:>>%s<<", ble_advertising_start, cmd_len, cmd);

  #ifdef DEBUG
  // buffer log/debug in hex
  D("[BLE] command buffer in hex: ");
  for (size_t i = 0; i < cmd_len; i++) {
    R("%02X", (unsigned char)cmd[i]);
  }
  R("\n");
  #endif

  // Check if the command starts with "AT"
  if(cmd_len >= 2 && strncmp(cmd, "AT", 2) == 0) {
    // Handle AT command
    LOG("[BLE] Handling AT command: '%s'", cmd);
    ble_send_response(at_cmd_handler(cmd));
  } else {
    LOG("[BLE] Invalid command received: '%s'", cmd);
    ble_send_response((const char*)("+ERROR: invalid command"));
  }

  D("[BLE] Command processing complete");
}

NOINLINE
void ble_send_response(const char *response) {
  if (ble_advertising_start == 0 || deviceConnected == 0 || !pTxCharacteristic || response == NULL)
    return;

  // sanity check
  size_t ssz = strlen(response);
  if(ssz == 0)
    return;

  // Send response with line terminator
  uint8_t local_buf[2 + ssz] = {0};
  uint8_t ok = 1;
  memcpy(local_buf, response, ssz);
  memcpy(local_buf + ssz, "\r\n", 2);
  ok &= ble_send_n((const uint8_t *)&local_buf, ssz + 2);
  if(!ok) {
    LOG("[BLE] Failed to send response");
  } else {
    LOG("[BLE] Response sent successfully: %d bytes", ssz + 2);
    #ifdef DEBUG
    D("[BLE] Response sent: '%s'", response);
    #endif // DEBUG
  }
}

NOINLINE
uint8_t ble_send_n(const uint8_t *bstr, size_t len) {
  if (ble_advertising_start == 0)
    return 0;

  /*
  #ifdef DEBUG
  D("[BLE] TX mtu: %d, connected: %d, length: %d", ble_mtu, deviceConnected, len);
  D("[BLE] TX mtu buffer in hex: ");
  for (uint16_t i = 0; i < len; i++) {
    R("%02X", (unsigned char)bstr[i]);
  }
  R("\n");
  D("[BLE] TX mtu buffer in ascii: ");
  for (uint16_t i = 0; i < len; i++) {
    if(bstr[i] == '\n') {
      R("\n");
    } else {
      R("%s", isprint(bstr[i]) ? (char[]) {bstr[i], '\0'} : ".");
    }
  }
  R("\n");
  #endif // DEBUG
  */

  if (deviceConnected == 1 && pTxCharacteristic) {
    static size_t snr = 0;
    snr++;
    D("[BLE] Sending response, total length: %d", len);
    // Split response into chunks (BLE characteristic limit), use negotiated MTU
    size_t o = 0;
    size_t cs = 0;
    while (o < len && deviceConnected == 1) {
      // yield to other tasks
      doYIELD;

      // multitasking, can unset deviceConnected or pTxCharacteristic
      // double check after doYIELD
      if(pTxCharacteristic == NULL) {
        LOG("[BLE] Stopped sending, characteristic is NULL");
        break;
      }
      if(pService == NULL || pService->getServer() == NULL) {
        LOG("[BLE] Stopped sending, server is NULL while waiting to notify");
        break;
      }

      // chunk size ?
      cs = ble_mtu - 3;      // ATT_MTU-3 for payload
      cs = min(cs, len - o); // smaller of remaining
      if(cs == 0) {
        LOOP_D("[BLE] chunk size is 0");
        break;
      }
      #ifdef DEBUG
      T(""); R("[BLE] NOTIFY #%04d, len:%04d, chunk:%04d, sent:%04d, data: >>\n", snr, len, cs, o);
      for(uint16_t i = 0; i < cs; i++) {
        R("%c", (unsigned char)bstr[o + i]);
      }
      R("<<\n");
      #endif // DEBUG

      // let's use ble_gatts_notify_custom() directly to get the proper error code
      uint16_t conn_handle = 0;
      for(auto &z: pService->getServer()->getPeerDevices(false)) {
        conn_handle = z.first;
        break;
      }
      if(!conn_handle) {
        LOG("[BLE] Stopped sending, no valid connection handle");
        break;
      }
      uint8_t nr_retries = 0;
      os_mbuf *ble_out_msg = NULL;
      REDO_SEND: {
        doYIELD;
        D("[BLE] NOTIFY attempt: a:%d, l:%d, chunk:%d, retry:%d", snr, len, cs, nr_retries);
        // create m_buf in each loop iteration to avoid memory leak, it gets consumed with each call
        ble_out_msg = ble_hs_mbuf_from_flat((uint8_t *)(bstr + o), cs);
        if(ble_out_msg == NULL) {
          LOG("[BLE] notify failed, cannot allocate memory for %d bytes", cs);
          doYIELD;
          delayMicroseconds(100);
          nr_retries++;
          if(nr_retries >= 5) {
            LOG("[BLE] notify failed, maximum retries reached for memory allocation");
            return 0;
          }
          goto REDO_SEND;
        }
        D("[BLE] notifying: a:%d, l:%d, chunk:%d", snr, len, cs);
        esp_err_t err = ble_gatts_notify_custom(conn_handle, pTxCharacteristic->getHandle(), ble_out_msg);
        if(err != ESP_OK) {
          D("[BLE] notify failed with error: a:%d, l:%d, c:%d, e:%d, %s", snr, inlen, cs, err, err == 6 ? "ENOMEM": "UNKNOWN");

          // destroy m_buf after use to avoid memory leak
          os_mbuf_free_chain(ble_out_msg);
          ble_out_msg = NULL;

          // doYIELD for other things, we're in a GOTO loop
          doYIELD;
          // after doYIELD, check if still connected and characteristic valid
          if(deviceConnected == 0) {
            LOG("[BLE] Stopped sending, not connected anymore while waiting to notify");
            break;
          }
          if(pTxCharacteristic == NULL) {
            LOG("[BLE] Stopped sending, characteristic is NULL while waiting to notify");
            break;
          }
          if(pService == NULL || pService->getServer() == NULL) {
            LOG("[BLE] Stopped sending, server is NULL while waiting to notify");
            break;
          }
          goto REDO_SEND;
        } else {
          D("[BLE] notify sent successfully: a:%d, l:%d, chunk:%d", snr, len, cs);
        }

        // success, ble_out_msg is consumed, yield
        doYIELD;
      }
      ble_out_msg = NULL;

      // advance
      o += cs;
    }
    if(o < len) {
      doYIELD;
      LOG("[BLE] Stopped sending, not connected anymore, sent %d of %d bytes", o, len);
      return 0;
    } else {
      doYIELD;
      D("[BLE] Sending complete, total %d bytes sent", o);
      return 1;
    }
  }
}

NOINLINE
uint8_t ble_send(const char *dstr) {
  return ble_send_n((const uint8_t *)dstr, strlen(dstr));
}

NOINLINE
void start_advertising_ble() {
  /*
  #ifdef SUPPORT_WIFI
  // stop WiFi
  if(cfg.wifi_enabled && strlen(cfg.wifi_ssid) != 0) {
    esp_err_t err = esp_wifi_stop();
    if(err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED) {
      LOG("[BLE] Failed to stop WiFi before starting BLE: %s", esp_err_to_name(err));
    } else {
      LOG("[BLE] WiFi stopped before starting BLE");
    }
  }
  #endif // SUPPORT_WIFI
  */

  // start BLE
  LOG("[BLE] Enabling Bluetooth and starting advertising");
  if (pServer) {
    pServer->getAdvertising()->stop();
    pServer->getAdvertising()->start();
  }
  ble_advertising_start = millis();
  LOG("[BLE] Advertising started, waiting for client connection");
}

NOINLINE
void stop_advertising_ble() {
  // Mark as disabled
  ble_advertising_start = 0;

  if(deviceConnected == 1) {
    LOG("[BLE] Disconnecting from connected device");
    pServer->disconnect(0);
    deviceConnected = 0;
  }

  // Stop advertising
  LOG("[BLE] Stopping advertising and disabling Bluetooth");
  if (pServer)
    pServer->getAdvertising()->stop();

  LOG("[BLE] Bluetooth disabled");

  /*
  #ifdef SUPPORT_WIFI
  if(cfg.wifi_enabled && strlen(cfg.wifi_ssid) != 0) {
    esp_err_t err = esp_wifi_start();
    if(err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED) {
      LOG("[BLE] Failed to start WiFi after stopping BLE: %s", esp_err_to_name(err));
    } else {
      LOG("[BLE] WiFi started after stopping BLE");
    }
  }
  #endif // SUPPORT_WIFI
  */
}
#endif

NOINLINE
void setup_cfg() {
  // read
  CFG_LOAD();
  // was (or needs) initialized?
  LOG("[CONFIG] init=%08X, version=%08X, size=%d", cfg.initialized, cfg.version, sizeof(cfg));
  if(cfg.initialized != CFGINIT || cfg.version != CFGVERSION) {
    #ifdef VERBOSE
    cfg.do_verbose = 1;
    #endif
    LOG("[CONFIG] reinitializing");
    // clear
    memset(&cfg, 0, sizeof(cfg));
    // reinit
    cfg.initialized       = CFGINIT;
    cfg.version           = CFGVERSION;
    #ifdef VERBOSE
    cfg.do_verbose        = 1;
    #endif
    #ifdef TIMELOG
    cfg.do_timelog        = 0;
    #endif
    #ifdef LOGUART
    cfg.do_log            = 0;
    #endif
    #ifdef LOOP_DELAY
    cfg.main_loop_delay   = 0;
    #endif
    #ifdef SUPPORT_ESP_LOG_INFO
    cfg.esp_log_interval  = 60000;
    #endif
    #if defined(SUPPORT_WIFI) && defined(SUPPORT_NTP)
    cfg.ntp_enabled = 1;
    #endif // SUPPORT_WIFI && SUPPORT_NTP
    #ifdef SUPPORT_WIFI
    cfg.ip_mode = IPV4_DHCP | IPV6_SLAAC;
    #endif // SUPPORT_WIFI
    #ifdef SUPPORT_UART1
    // Initialize UART1 with default values
    cfg.uart1_baud = 115200;
    cfg.uart1_data = 8;
    cfg.uart1_parity = 0;
    cfg.uart1_stop = 1;
    cfg.uart1_rx_pin = UART1_RX_PIN;
    cfg.uart1_tx_pin = UART1_TX_PIN;
    cfg.uart1_inv    = 0;
    #endif
    // write config
    CFG_SAVE();
    LOG("[CONFIG] reinitializing done");
  }

  // default BLE security to usable values
  #if defined(BLUETOOTH_UART_AT)
  cfg.ble_pin = BLUETOOTH_UART_DEFAULT_PIN; // Default PIN
  cfg.ble_security_mode = 0; // No security
  cfg.ble_io_cap = 3;        // NoInputNoOutput
  cfg.ble_auth_req = 0;      // No authentication
  #endif

  #ifdef VERBOSE
  COMMON::_do_verbose = cfg.do_verbose;
  #endif
}

#define UART1_RX_BUFFER_SIZE   2048 // max size of UART1 buffer Rx
#define UART1_TX_BUFFER_SIZE   2048 // max size of UART1 buffer Tx, 0 means no buffer, direct write and wait

#ifdef SUPPORT_UART1
NOINLINE
void setup_uart1() {

  // Convert config values to Arduino constants
  uint32_t config = 0;

  // Build configuration based on data bits, parity, and stop bits
  if(cfg.uart1_data == 5) {
    if(cfg.uart1_parity == 1)
      config = (cfg.uart1_stop == 2) ? SERIAL_5E2 : SERIAL_5E1;
    else if(cfg.uart1_parity == 2)
      config = (cfg.uart1_stop == 2) ? SERIAL_5O2 : SERIAL_5O1;
    else
      config = (cfg.uart1_stop == 2) ? SERIAL_5N2 : SERIAL_5N1;
  } else if(cfg.uart1_data == 6) {
    if(cfg.uart1_parity == 1)
      config = (cfg.uart1_stop == 2) ? SERIAL_6E2 : SERIAL_6E1;
    else if(cfg.uart1_parity == 2)
      config = (cfg.uart1_stop == 2) ? SERIAL_6O2 : SERIAL_6O1;
    else
      config = (cfg.uart1_stop == 2) ? SERIAL_6N2 : SERIAL_6N1;
  } else if(cfg.uart1_data == 7) {
    if(cfg.uart1_parity == 1)
      config = (cfg.uart1_stop == 2) ? SERIAL_7E2 : SERIAL_7E1;
    else if(cfg.uart1_parity == 2)
      config = (cfg.uart1_stop == 2) ? SERIAL_7O2 : SERIAL_7O1;
    else
      config = (cfg.uart1_stop == 2) ? SERIAL_7N2 : SERIAL_7N1;
  } else { // 8 bits (default)
    if(cfg.uart1_parity == 1)
      config = (cfg.uart1_stop == 2) ? SERIAL_8E2 : SERIAL_8E1;
    else if(cfg.uart1_parity == 2)
      config = (cfg.uart1_stop == 2) ? SERIAL_8O2 : SERIAL_8O1;
    else
      config = (cfg.uart1_stop == 2) ? SERIAL_8N2 : SERIAL_8N1;
  }

  // Stop UART1 if already running
  UART1.flush();
  UART1.end();

  // Configure UART1 with new parameters
  // Use APB (Advanced Peripheral Bus) clock for better accuracy,
  // uses more power, allows faster baud rates
  // XTAL clock is fixed at 40MHz, APB clock is 80MHz
  #if defined(UART_CLK_SRC_APB)
    UART1.setClockSource(UART_CLK_SRC_APB);
  #elif defined(UART_CLK_SRC_XTAL)
    UART1.setClockSource(UART_CLK_SRC_XTAL);
  #else
    #warning "No UART clock source defined, using default"
  #endif
  UART1.setMode(UART_MODE_UART);

  // Set buffer sizes, before begin()!
  size_t bufsize = 0;
  bufsize = UART1.setRxBufferSize(UART1_RX_BUFFER_SIZE);
  LOG("[UART1] RX buffer size set to %d bytes", bufsize);
  bufsize = UART1.setTxBufferSize(UART1_TX_BUFFER_SIZE);
  LOG("[UART1] TX buffer size set to %d bytes", bufsize);

  // Initialize UART1
  LOG("[UART1] Initializing with %lu baud, rx pin: %d, tx pin: %d, inverted: %d, config: %d", cfg.uart1_baud, cfg.uart1_rx_pin, cfg.uart1_tx_pin, cfg.uart1_inv, config);
  UART1.begin(cfg.uart1_baud, config, cfg.uart1_rx_pin, cfg.uart1_tx_pin);
  // no way of finding out whether this works

  // Non-blocking read
  LOG("[UART1] Setting non-blocking read");
  UART1.setTimeout(0);

  // Invert?
  if(cfg.uart1_inv == 1) {
    LOG("[UART1] Inverting RX");
    UART1.setRxInvert(true);
  }

  // Error handling
  UART1.onReceiveError([](hardwareSerial_error_t event) {
    LOOP_D("[UART1] Receive error: %d, %s", event,
        event == UART_NO_ERROR          ? "NO ERROR" :
        event == UART_BREAK_ERROR       ? "BREAK ERROR" :
        event == UART_BUFFER_FULL_ERROR ? "BUFFER FULL ERROR" :
        event == UART_FIFO_OVF_ERROR    ? "FIFO OVERFLOW ERROR" :
        event == UART_FRAME_ERROR       ? "FRAME ERROR" :
        event == UART_PARITY_ERROR      ? "PARITY ERROR" :
        "UNKNOWN");
  });

  // Enable CTS/RTS hardware flow control, 60 bytes RX FIFO threshold
  LOG("[UART1] Enabling CTS/RTS hardware flow control");
  UART1.setHwFlowCtrlMode(UART_HW_FLOWCTRL_CTS_RTS, 60);

  // Trigger RX FIFO interrupt when at least this amount of bytes is available
  LOG("[UART1] Setting RX FIFO full threshold to 64 bytes");
  UART1.setRxFIFOFull(64);

  // Trigger the onReceive internal call back when not enough data for the
  // FIFOFull check to happen but still timeout, calculated sleep by IDF.
  // E.g.: For baud: 115200baud, symbol: SERIAL_8N1 (10bit), symbols_timeout: 1t
  // > print(1000ms * 1t / (115200baud / 10bit))
  // 0.086805555555556 ms timeout ~ 86 microseconds
  LOG("[UART1] Setting RX timeout to 1 symbol time");
  UART1.setRxTimeout(1);

  LOG("[UART1] Configured: %lu baud, %d%c%d, RX=%d, TX=%d",
      cfg.uart1_baud, cfg.uart1_data,
      (cfg.uart1_parity == 0) ? 'N' : (cfg.uart1_parity == 1) ? 'E' : 'O',
      cfg.uart1_stop, cfg.uart1_rx_pin, cfg.uart1_tx_pin);
}


NOINLINE
void do_uart1_read() {
  // Read all available bytes from UART, but only for as much data as fits in
  // inbuf, read per X chars to be sure we don't overflow
  LOOP_D("[LOOP] Checking for available data, inlen: %d", inlen);
  uint8_t *b_old = inbuf + inlen;
  uint8_t *b_new = b_old;
  while(b_new < inbuf_max) {
    // read bytes into inbuf
    size_t to_r = UART1.readBytes(b_new, (size_t)(inbuf_max - b_new));
    if(to_r <= 0)
        break; // nothing read
    inlen += to_r;
    b_new += to_r;
    // slight delay to allow more data to arrive
    //delayMicroseconds(50);
    LOOP_D("[UART1] READ %04d bytes from UART1, buf:%04d", to_r, inlen);
    doYIELD;
  }
  if(b_old != b_new) {
    // null terminate, even if b_new = inbuf_max, we have space for the \0
    *b_new = '\0';
    LOOP_D("[UART1]: Total bytes in inbuf: %d", inlen);
    D("[UART1] inbuf: >>%s<<", b_old);
    #ifdef LED
    last_activity = millis(); // Trigger LED activity for UART1 receive
    #endif // LED
  } else {
    LOOP_D("[UART1]: No new data read from UART1");
  }
  doYIELD;
}

#endif // SUPPORT_UART1

#if defined(SUPPORT_WIFI) && defined(WIFI_WPS)
/* WPS (WiFi Protected Setup) Functions both PBC and PIN */
bool start_wps(const char *pin) {
  if (cfg.wifi_enabled == 0) {
    LOG("[WPS] WiFi is disabled in config");
    return false;
  }
  if (wps_running) {
    LOG("[WPS] WPS already running");
    return false;
  }

  LOG("[WPS] Starting WPS Push Button Configuration, timeout %d seconds", WPS_TIMEOUT_MS / 1000);

  // Stop any current WiFi connections
  stop_networking();
  WiFi.removeEvent(WiFiEvent);
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK); // require WPA2
  WiFi.setScanMethod(WIFI_FAST_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  esp_wps_config_t wps_config;
  if(pin == NULL) {
    // Configure WPS - modern ESP32 API
    wps_config.wps_type = WPS_TYPE_PBC;

    // set optional device name and manufacturer
    snprintf((char*)wps_config.factory_info.manufacturer, sizeof(wps_config.factory_info.manufacturer), "HOMEKIT");
    snprintf((char*)wps_config.factory_info.model_name, sizeof(wps_config.factory_info.model_name), "UART");
    snprintf((char*)wps_config.factory_info.model_number, sizeof(wps_config.factory_info.model_number), "1.0");
    snprintf((char*)wps_config.factory_info.device_name, sizeof(wps_config.factory_info.device_name), DEFAULT_HOSTNAME);
  } else {
    // Configure WPS - modern ESP32 API
    wps_config.wps_type = WPS_TYPE_PIN;
    LOG("[WPS] Starting WPS with PIN: %s", pin);
  }

  // Start WPS
  esp_err_t result = esp_wifi_wps_enable(&wps_config);
  if (result != ESP_OK) {
    LOG("[WPS] Failed to enable WPS: %s", esp_err_to_name(result));
    return false;
  }

  result = esp_wifi_wps_start(0);
  if (result != ESP_OK) {
    LOG("[WPS] Failed to start WPS: %s", esp_err_to_name(result));
    esp_wifi_wps_disable();
    return false;
  }

  wps_running = true;
  wps_start_time = millis();
  #ifdef LED
  last_activity = millis(); // Trigger LED activity for WPS
  #endif // LED
  LOG("[WPS] WPS PBC started successfully");
  return true;
}

bool stop_wps() {
  if (!wps_running) {
    LOG("[WPS] WPS not running");
    return false;
  }

  LOG("[WPS] Stopping WPS");
  esp_wifi_wps_disable();
  wps_running = false;
  wps_start_time = 0;
  LOG("[WPS] WPS stopped");
  return true;
}

NOINLINE
const char* get_wps_status() {
  if (!wps_running) {
    return "stopped";
  }

  unsigned long elapsed = millis() - wps_start_time;
  if (elapsed > WPS_TIMEOUT_MS) {
    return "timeout";
  }

  return "running";
}
#endif // SUPPORT_WIFI && WIFI_WPS

#ifdef SUPPORT_WIFI
NOINLINE
void on_wifi_stop(){
  #ifdef SUPPORT_NTP
  if(esp_sntp_enabled()){
    LOG("[NTP] Stopping NTP client due to WiFi disconnection");
    esp_sntp_stop();
  }
  #endif // SUPPORT_NTP

  #ifdef SUPPORT_MDNS
  stop_mdns();
  #endif // SUPPORT_MDNS
}

NOINLINE
void on_wifi_start(){
  #ifdef SUPPORT_MDNS
  setup_mdns();
  #endif // SUPPORT_MDNS

  #ifdef SUPPORT_NTP
  setup_ntp();
  #endif // SUPPORT_NTP
}

NOINLINE
void wifi_start(){
  if(cfg.wifi_enabled == 0) {
    LOG("[WiFi] WiFi is disabled in config, not connecting");
    return;
  }
  if(strlen((char*)cfg.wifi_ssid) == 0) {
    LOG("[WiFi] No SSID configured, cannot connect");
    return;
  }
  LOG("[WiFi] STA started, connecting to %s", cfg.wifi_ssid);
  esp_err_t err = esp_wifi_connect();
  if (err != ESP_OK)
    LOG("[WiFi] Failed to initiate connection: %s", esp_err_to_name(err));
}

void WiFiEvent(WiFiEvent_t event) {
  doYIELD;
  switch(event) {
      case ARDUINO_EVENT_WIFI_READY: {
          LOG("[WiFi] ready");
          break;
      }
      case ARDUINO_EVENT_WIFI_STA_START: {
          LOG("[WiFi] STA started");
          break;
      }
      case ARDUINO_EVENT_WIFI_STA_STOP: {
          LOG("[WiFi] STA stopped");
          break;
      }
      case ARDUINO_EVENT_WIFI_STA_CONNECTED: {
          LOG("[WiFi] STA connected to %s", WiFi.SSID().c_str());
          break;
      }
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
          LOG("[WiFi] STA disconnected");
          #ifdef SUPPORT_MDNS
          stop_mdns();
          #endif // SUPPORT_MDNS
          break;
      }
      case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE: {
          LOG("[WiFi] STA auth mode changed");
          break;
      }
      case ARDUINO_EVENT_WIFI_STA_GOT_IP6: {
          LOGT("[WiFi] STA got IPV6: ga: %s", WiFi.globalIPv6().toString().c_str());
          LOGR(", ll: %s", WiFi.linkLocalIPv6().toString().c_str());
          LOGR("\n");
          reconfigure_network_connections();
          #ifdef SUPPORT_MDNS
          setup_mdns();
          #endif // SUPPORT_MDNS
          break;
      }
      case ARDUINO_EVENT_WIFI_STA_GOT_IP: {
          LOG("[WiFi] STA got IP: %s", WiFi.localIP().toString().c_str());
          reconfigure_network_connections();
          #ifdef SUPPORT_MDNS
          setup_mdns();
          #endif // SUPPORT_MDNS
          break;
      }
      case ARDUINO_EVENT_WIFI_STA_LOST_IP: {
          LOG("[WiFi] STA lost IP");
          #ifdef SUPPORT_MDNS
          stop_mdns();
          #endif // SUPPORT_MDNS
          stop_network_connections();
          break;
      }
      case ARDUINO_EVENT_WPS_ER_SUCCESS: {
          #ifdef WIFI_WPS
          LOG("[WPS] succeeded");
          wps_running = false;
          wps_start_time = 0;
          esp_wifi_wps_disable();

          // Use esp_wifi_get_config() to read saved credentials
          wifi_config_t saved_config;
          if (esp_wifi_get_config(WIFI_IF_STA, &saved_config) == ESP_OK) {
            if(strlen((char*)saved_config.sta.ssid) == 0) {
              LOG("[WPS] No SSID received, WPS failed");
              break;
            }
            if(strlen((char*)saved_config.sta.password) == 0) {
              LOG("[WPS] No Password received, WPS failed");
              break;
            }
            if(strlen((char*)saved_config.sta.ssid) > sizeof(cfg.wifi_ssid) - 1) {
              LOG("[WPS] SSID too long, WPS failed");
              break;
            }
            if(strlen((char*)saved_config.sta.password) > sizeof(cfg.wifi_pass) - 1) {
              LOG("[WPS] Password too long, WPS failed");
              break;
            }

            // clear the IP config, we're ok with WPS now
            cfg.ip_mode &= ~IPV4_STATIC;
            cfg.ip_mode |= IPV4_DHCP;
            cfg.ip_mode |= IPV6_SLAAC;
            memset((char*)cfg.ipv4_addr, 0, sizeof(cfg.ipv4_addr));
            memset((char*)cfg.ipv4_gw, 0, sizeof(cfg.ipv4_gw));
            memset((char*)cfg.ipv4_mask, 0, sizeof(cfg.ipv4_mask));
            memset((char*)cfg.ipv4_dns, 0, sizeof(cfg.ipv4_dns));

            // Save new credentials to config
            strncpy((char*)cfg.wifi_ssid, (char*)saved_config.sta.ssid, sizeof(cfg.wifi_ssid) - 1);
            strncpy((char*)cfg.wifi_pass, (char*)saved_config.sta.password, sizeof(cfg.wifi_pass) - 1);
            LOG("[WPS] Saved SSID: %s", cfg.wifi_ssid);
            LOG("[WPS] Saved Pass: ********", cfg.wifi_pass);
            D("[WPS] Saved Pass (clear): %s", cfg.wifi_pass);
            CFG_SAVE();

            // WPS success, credentials are automatically saved
            // Restart WiFi connection with new credentials
            LOG("[WPS] Restarting WiFi with new credentials");
            reset_networking();
          }
          #endif
          break;
      }
      case ARDUINO_EVENT_WPS_ER_FAILED: {
          #ifdef WIFI_WPS
          LOG("[WPS] failed");
          wps_running = false;
          wps_start_time = 0;
          esp_wifi_wps_disable();
          #endif
          break;
      }
      case ARDUINO_EVENT_WPS_ER_TIMEOUT: {
          #ifdef WIFI_WPS
          LOG("[WPS] timed out");
          wps_running = false;
          wps_start_time = 0;
          esp_wifi_wps_disable();
          #endif
          break;
      }
      case ARDUINO_EVENT_WPS_ER_PIN: {
          #ifdef WIFI_WPS
          LOG("[WPS] PIN received");
          #endif
          break;
      }
      default:
          break;
  }
}
#endif // SUPPORT_WIFI

#ifdef BT_CLASSIC
void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  doYIELD;
  if(event == ESP_SPP_START_EVT) {
    LOG("BlueTooth UART Initialized SPP");
  } else if(event == ESP_SPP_SRV_OPEN_EVT) {
    LOG("BlueTooth UART Client connected");
  } else if(event == ESP_SPP_CLOSE_EVT) {
    LOG("BlueTooth UART Client disconnected");
  } else if(event == ESP_SPP_DATA_IND_EVT) {
    LOG("BlueTooth UART Data received");
    // any new AT command?
    ATScBT.ReadSerial();
  }
}
#endif

#ifdef SUPPORT_ESP_LOG_INFO
NOINLINE
void log_esp_info() {
  LOG("[ESP] === ESP32 System Information ===");
  LOG("[ESP] Firmware version: %s", ESP.getSdkVersion());
  LOG("[ESP] Chip Model: %06X", ESP.getChipModel());
  LOG("[ESP] Chip Revision: %d", ESP.getChipRevision());
  LOG("[ESP] CPU Frequency: %d MHz", ESP.getCpuFreqMHz());
  LOG("[ESP] Flash Size: %d MB", ESP.getFlashChipSize() / (1024 * 1024));
  LOG("[ESP] Sketch Size: %d bytes", ESP.getSketchSize());
  LOG("[ESP] Sketch Free Space: %d bytes", ESP.getFreeSketchSpace());
  LOG("[ESP] ESP Core Version: %s", ESP.getCoreVersion());
  LOG("[ESP] Boot Flash Size: %d", ESP.getFlashChipSize());
  LOG("[ESP] Boot Flash Speed: %d", ESP.getFlashChipSpeed());
  LOG("[ESP] Boot Flash Mode: %d", ESP.getFlashChipMode());
  LOG("[ESP] CPU Cores: %d", ESP.getChipCores());
  LOG("[ESP] SDK Version: %s", ESP.getSdkVersion());
  LOG("[ESP] Uptime: %lu seconds", millis() / 1000);

  #if defined(BLUETOOTH_UART_AT) || defined(SUPPORT_BLE_UART)
  log_base_bt_mac();
  #endif

  // Log partition information
  LOG("[ESP] === Partition Information ===");
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
  while (it != NULL) {
    const esp_partition_t* partition = esp_partition_get(it);
    LOG("[ESP] Partition: %s, Type: 0x%02x, SubType: 0x%02x, Address: 0x%08x, Size: %d KB",
        partition->label, partition->type, partition->subtype, partition->address, partition->size / 1024);
    it = esp_partition_next(it);
  }
  esp_partition_iterator_release(it);

  // Log NVS statistics
  LOG("[ESP] === NVS Statistics ===");
  nvs_stats_t nvs_stats;
  esp_err_t err = nvs_get_stats(NULL, &nvs_stats);
  if (err == ESP_OK) {
    LOG("[ESP] NVS Used Entries: %d", nvs_stats.used_entries);
    LOG("[ESP] NVS Free Entries: %d", nvs_stats.free_entries);
    LOG("[ESP] NVS Total Entries: %d", nvs_stats.total_entries);
    LOG("[ESP] NVS Utilization: %d%%", (nvs_stats.used_entries * 100) / nvs_stats.total_entries);
    LOG("[ESP] NVS Available Entries: %d", nvs_stats.available_entries);
    LOG("[ESP] NVS Namespace Count: %d", nvs_stats.namespace_count);
  } else {
    LOG("[ESP] Failed to get NVS stats: %s", esp_err_to_name(err));
  }

  // Log SPIFFS statistics
  LOG("[ESP] === SPIFFS Statistics ===");
  if (SPIFFS.begin(true)) {
    size_t total_bytes = SPIFFS.totalBytes();
    size_t used_bytes = SPIFFS.usedBytes();
    size_t free_bytes = total_bytes - used_bytes;
    LOG("[ESP] SPIFFS Total: %d bytes (%.2f KB)", total_bytes, total_bytes / 1024.0);
    LOG("[ESP] SPIFFS Used: %d bytes (%.2f KB)", used_bytes, used_bytes / 1024.0);
    LOG("[ESP] SPIFFS Free: %d bytes (%.2f KB)", free_bytes, free_bytes / 1024.0);
    LOG("[ESP] SPIFFS Utilization: %.1f%%", (used_bytes * 100.0) / total_bytes);
    SPIFFS.end();
  } else {
    LOG("[ESP] SPIFFS not available or failed to mount");
  }

  // Log specific NVS partitions
  LOG("[ESP] === NVS Partition Details ===");
  const esp_partition_t* nvs_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
  if (nvs_partition != NULL) {
    LOG("[ESP] NVS Partition: %s, Size: %d KB, Address: 0x%08x",
        nvs_partition->label, nvs_partition->size / 1024, nvs_partition->address);
    // Now list all namespaces
    nvs_iterator_t it;
    err = nvs_entry_find(nvs_partition->label, NULL, NVS_TYPE_ANY, &it);
    while (err == ESP_OK ) {
      nvs_entry_info_t info;
      if (nvs_entry_info(it, &info) == ESP_OK)
        LOG("[ESP] - NVS Entry Partition: %s, Namespace: %s, Key: %s, Type: %d", nvs_partition->label, info.namespace_name, info.key, info.type);
      err = nvs_entry_next(&it);
    }
  }

  // Log the "esp-at" NVS partition if it exists
  nvs_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, CFG_PARTITION);
  if (nvs_partition != NULL) {
    LOG("[ESP] NVS Partition: %s, Size: %d KB, Address: 0x%08x",
        nvs_partition->label, nvs_partition->size / 1024, nvs_partition->address);
    // Now list all namespaces
    nvs_iterator_t it;
    err = nvs_entry_find(nvs_partition->label, NULL, NVS_TYPE_ANY, &it);
    while (err == ESP_OK ) {
      nvs_entry_info_t info;
      if (nvs_entry_info(it, &info) == ESP_OK)
        LOG("[ESP] - NVS Entry Partition: %s, Namespace: %s, Key: %s, Type: %d", nvs_partition->label, info.namespace_name, info.key, info.type);
      err = nvs_entry_next(&it);
    }
  }

  const esp_partition_t* nvs_keys_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS, NULL);
  if (nvs_keys_partition != NULL) {
    LOG("[ESP] NVS Keys Partition: %s, Size: %d KB, Address: 0x%08x",
        nvs_keys_partition->label, nvs_keys_partition->size / 1024, nvs_keys_partition->address);
  }

  // log heap information
  LOG("[ESP] === Heap Information ===");
  LOG("[ESP] Minimum Free Heap: %d bytes", ESP.getMinFreeHeap());
  LOG("[ESP] PSRAM Size: %d bytes", ESP.getPsramSize());
  LOG("[ESP] Free PSRAM: %d bytes", ESP.getFreePsram());
  LOG("[ESP] Minimum Free PSRAM: %d bytes", ESP.getMinFreePsram());
  LOG("[ESP] Total Heap: %d bytes", ESP.getHeapSize());
  LOG("[ESP] Free Heap: %d bytes", ESP.getFreeHeap());
  size_t hs = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  LOG("[ESP] Free heap size (8BIT): %d bytes", hs);
  hs = heap_caps_get_free_size(MALLOC_CAP_32BIT);
  LOG("[ESP] Free heap size (32BIT): %d bytes", hs);
  hs = heap_caps_get_free_size(MALLOC_CAP_DMA);
  LOG("[ESP] Free heap size (DMA): %d bytes", hs);
  hs = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  LOG("[ESP] Free heap size (INTERNAL): %d bytes", hs);
  hs = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  LOG("[ESP] Free heap size (SPIRAM): %d bytes", hs);
  hs = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  LOG("[ESP] Free heap size (DEFAULT): %d bytes", hs);
  hs = heap_caps_get_free_size(MALLOC_CAP_EXEC);
  LOG("[ESP] Free heap size (EXEC): %d bytes", hs);
  hs = heap_caps_get_free_size(MALLOC_CAP_IRAM_8BIT);
  LOG("[ESP] Free heap size (IRAM_8BIT): %d bytes", hs);
  hs = heap_caps_get_free_size(MALLOC_CAP_RTCRAM);
  LOG("[ESP] Free heap size (RTCRAM): %d bytes", hs);

  LOG("[ESP] === MISC Information ===");
  // log the size of cfg_t
  LOG("[ESP] Size of cfg: %d bytes", sizeof(cfg_t));
  // log the size of global buffers
  LOG("[ESP] Size of inbuf: %d bytes", sizeof(inbuf));
  LOG("[ESP] Size of outbuf: %d bytes", sizeof(outbuf));
  #ifdef UART_AT
  LOG("[ESP] Size of AT cmd buffer: %d bytes", sizeof(atscbu));
  LOG("[ESP] Size of AT object: %d bytes", sizeof(ATSc));
  #endif
  #ifdef BT_CLASSIC
  LOG("[ESP] Size of BT Classic cmd buffer: %d bytes", sizeof(atscbt));
  LOG("[ESP] Size of BT Classic AT object: %d bytes", sizeof(ATScBT));
  LOG("[ESP] Size of BT Classic SPP object: %d bytes", sizeof(SerialBT));
  #endif
  #ifdef SUPPORT_WIFI
  LOG("[ESP] Size of WiFi object: %d bytes", sizeof(WiFi));
  LOG("[ESP] Size of WiFiClass object: %d bytes", sizeof(WiFiClass));
  #endif

  LOG("[ESP] === End of ESP Information ===");
}

void esp_heap_trace_alloc_hook(void *ptr, size_t size, uint8_t alloc_type) {
  size_t free_heap = ESP.getFreeHeap();
  LOG("[ESP] Heap allocation hook triggered, ptr: %p, free heap: %d bytes, cap: ", ptr, free_heap, alloc_type);
}

void esp_heap_trace_free_hook(void *ptr) {
  size_t free_heap = ESP.getFreeHeap();
  LOG("[ESP] Heap free hook triggered, ptr: %p, free heap: %d bytes", ptr, free_heap);
}
#endif // SUPPORT_ESP_LOG_INFO

#ifdef SUPPORT_WIFI
void log_wifi_info(const char *msg) {
  LOG("%s[WiFi] status: %d, %s", msg, WiFi.status(),
    WiFi.status() == WL_CONNECTED ? "connected" :
    WiFi.status() == WL_NO_SHIELD ? "no shield" :
    WiFi.status() == WL_IDLE_STATUS ? "idle" :
    WiFi.status() == WL_NO_SSID_AVAIL ? "no ssid available" :
    WiFi.status() == WL_SCAN_COMPLETED ? "scan completed" :
    WiFi.status() == WL_CONNECT_FAILED ? "connect failed" :
    WiFi.status() == WL_CONNECTION_LOST ? "connection lost" :
    WiFi.status() == WL_DISCONNECTED ? "disconnected" : "unknown");
  if(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS) {
    LOGT("%s[WiFi] connected: SSID:%s", msg, WiFi.SSID().c_str());
    LOGR(", MAC:%s", WiFi.macAddress().c_str());
    LOGR(", RSSI:%hu", WiFi.RSSI());
    LOGR(", BSSID:%s", WiFi.BSSIDstr().c_str());
    LOGR(", CHANNEL:%d", WiFi.channel());
    LOGR("\n");
    LOGT("%s[IPV4] ADDR:%s", msg, WiFi.localIP().toString().c_str());
    LOGR(", GW:%s", WiFi.gatewayIP().toString().c_str());
    LOGR(", NM:%s", WiFi.subnetMask().toString().c_str());
    LOGR(", DNS:%s", WiFi.dnsIP().toString().c_str());
    LOGR("\n");
    if(cfg.ip_mode & IPV6_SLAAC) {
      IPAddress g_ip6 = WiFi.globalIPv6();
      IPAddress l_ip6 = WiFi.linkLocalIPv6();
      LOGT("%s[IPV6] SLAAC:%s", msg, g_ip6.toString().c_str());
      LOGR(", LINK-LOCAL:%s", l_ip6.toString().c_str());
      LOGR("\n");
    }
  }
}
#endif // SUPPORT_WIFI

#ifdef LED

// LED configuration
volatile bool led_state = false;
bool last_led_state = false;
int last_led_interval = 0;
int8_t last_led_brightness = 0;
int8_t led_brightness_off = LED_BRIGHTNESS_OFF;
int8_t led_brightness_on  = LED_BRIGHTNESS_LOW;

hw_timer_t *led_t = NULL;
portMUX_TYPE led_timer_mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR ledBlinkTimer() {
  portENTER_CRITICAL_ISR(&led_timer_mux);
  led_state = !led_state;
  portEXIT_CRITICAL_ISR(&led_timer_mux);
}

// Helper function to set LED brightness (0-255 on ESP32, digital on/off on ESP8266)
void set_led_brightness(int brightness) {
  if(last_led_brightness == brightness) {
    return; // no change
  }
  last_led_brightness = brightness;
  #if defined(SUPPORT_LED_BRIGHTNESS)
  if (led_pwm_enabled) {
    // Use hardware PWM for smooth brightness control with channel-based API
    if(!ledcWriteChannel(LED_PWM_CHANNEL, brightness)) {
      // PWM failed, fallback to digital control
      digitalWrite(LED, brightness > LED_BRIGHTNESS_LOW ? HIGH : LOW);
    }
  } else {
    // PWM failed, fallback to digital control
    digitalWrite(LED, brightness > LED_BRIGHTNESS_LOW ? HIGH : LOW);
  }
  #else
  // ESP8266 fallback: treat anything above LOW threshold as HIGH
  digitalWrite(LED, brightness > LED_BRIGHTNESS_LOW ? HIGH : LOW);
  #endif
}

void led_on() {
  set_led_brightness(led_brightness_on);
  led_state = true;
}

void led_off() {
  set_led_brightness(led_brightness_off);
  led_state = false;
}

void setup_led() {
  LOG("[LED] Setup on pin %d", LED);

  // LED pin setup
  pinMode(LED, OUTPUT);

  #if defined(SUPPORT_LED_BRIGHTNESS)
  // ESP32 PWM setup
  ledc_clk_cfg_t t = ledcGetClockSource();
  LOG("[LED] LED PWM clock source: %d", t);
  // Setup LED PWM channel
  if (ledcAttachChannel(LED, LED_PWM_FREQUENCY, LED_PWM_RESOLUTION, LED_PWM_CHANNEL)) {
    led_pwm_enabled = true;
    LOG("[LED] PWM control enabled on pin %d, channel %d", LED, LED_PWM_CHANNEL);
  } else {
    // Fallback to digital control if PWM setup fails
    led_pwm_enabled = false;
    LOG("[LED] PWM setup failed, using digital control on pin %d", LED);
  }
  #endif // SUPPORT_LED_BRIGHTNESS

  // Start with LED on, and on/off are normal brightness values
  led_on();

  // setup a LED blink timer, default to 1 second interval -> AFTER pwm setup,
  // use timer 1, as 0 is used by PWM internally? Pick the same as PWM channel,
  // this gets reused internally
  LOG("[LED] setting up LED blink timer");
  led_t = timerBegin(1000);
  if(led_t == NULL) {
    LOG("[LED] Failed to initialize timer for LED");
  } else {
    LOG("[LED] Timer initialized successfully");
    timerAttachInterrupt(led_t, &ledBlinkTimer);
    LOG("[LED] Timer interrupt attached");
    timerAlarm(led_t, LED_BLINK_INTERVAL_NORMAL, true, 0);
    LOG("[LED] Timer alarm set to %d ms", LED_BLINK_INTERVAL_NORMAL);
    timerWrite(led_t, 0);
    timerStart(led_t);
    LOG("[LED] Timer started");
    LOG("[LED] LED setup completed successfully");
  }
  LOG("[LED] LED setup done, starting blink loop");
}

int determine_led_state() {
  // Enhanced LED control with new behavior patterns
  int led_interval = 0;
  unsigned long now = millis();
  bool comm_active = (now - last_activity < COMM_ACTIVITY_LED_DURATION);

  #ifdef SUPPORT_WIFI
  bool is_wifi_connected = (WiFi.status() == WL_CONNECTED);
  #endif // SUPPORT_WIFI

  // Determine LED behavior based on priority (highest to lowest):
  if (comm_active) {
    // Data transmission: tiny flicker on top of current state
    led_interval = LED_BLINK_INTERVAL_FLICKER;
    led_brightness_on = LED_BRIGHTNESS_LOW + LED_BRIGHTNESS_FLICKER;
    led_brightness_off = LED_BRIGHTNESS_LOW;
    #ifdef SUPPORT_WIFI
    if (is_wifi_connected) {
      led_brightness_on = LED_BRIGHTNESS_MEDIUM + LED_BRIGHTNESS_FLICKER; // Flicker on top of steady
      led_brightness_off = LED_BRIGHTNESS_MEDIUM; // Return to steady connected state
    }
    #endif // SUPPORT_WIFI
  #if defined(SUPPORT_BLE_UART1) || defined(BLUETOOTH_UART_AT)
  } else if ((ble_advertising_start != 0) && !(deviceConnected == 1)) {
    // BLE advertising (button pressed, waiting for connection): fast blink
    led_interval = LED_BLINK_INTERVAL_FAST;
    led_brightness_on = LED_BRIGHTNESS_HIGH;
    led_brightness_off = LED_BRIGHTNESS_LOW;
  } else if (deviceConnected == 1) {
    // BLE device connected: slow blink until disconnected
    led_interval = LED_BLINK_INTERVAL_SLOW;
    led_brightness_on = LED_BRIGHTNESS_MEDIUM;
    led_brightness_off = LED_BRIGHTNESS_LOW;
  #endif
  #ifdef WIFI_WPS
  } else if (wps_running) {
    // WPS active: slow blink
    led_interval = LED_BLINK_INTERVAL_QUICK;
    led_brightness_on = LED_BRIGHTNESS_HIGH;
    led_brightness_off = LED_BRIGHTNESS_DIM;
  #endif // WIFI_WPS
  #ifdef SUPPORT_WIFI
  } else if (is_wifi_connected) {
    // WiFi connected: full on at medium brightness (not too bright)
    led_interval = 0; // No blinking, steady on
    led_brightness_on = LED_BRIGHTNESS_MEDIUM;
    led_brightness_off = LED_BRIGHTNESS_MEDIUM; // Same as on = steady
  #endif // SUPPORT_WIFI
  } else {
    // Not connected: slowly blinking
    led_interval = LED_BLINK_INTERVAL_HALF;
    led_brightness_on = LED_BRIGHTNESS_LOW;
    led_brightness_off = LED_BRIGHTNESS_OFF;
  }
  return led_interval;
}

void update_led_state() {
  if(led_state != last_led_state) {
    last_led_state = led_state;
    if(led_state) {
      led_on();
    } else {
      led_off();
    }
  }
}

void set_led_blink(int interval_ms) {
  if(interval_ms != last_led_interval) {
    D("[LED] Setting LED blink interval to %d ms", interval_ms);
    last_led_interval = interval_ms;
    if(interval_ms == 0) {
      // Steady on or off
      if(led_brightness_on == led_brightness_off) {
        // Same brightness, just set it and stop timer
        timerStop(led_t);
        last_led_state = !led_state; // force update
        if(led_brightness_on > LED_BRIGHTNESS_LOW) {
          led_on();
        } else {
          led_off();
        }
      }
    } else {
      timerStop(led_t);
      timerAlarm(led_t, last_led_interval, true, 0);
      D("[LED] Timer alarm set to %d ms", last_led_interval);
      timerWrite(led_t, 0);
      timerStart(led_t);
    }
  }
}
#endif // LED

#ifdef SUPPORT_GPIO
void setup_gpio() {
  // Initialize GPIO pins from config
  for(uint8_t i=0; i<cfg.gpio_cfg_count; ++i) {
    int8_t pin = cfg.gpio_cfg[i].pin;
    int8_t mode = cfg.gpio_cfg[i].mode;
    int8_t value = cfg.gpio_cfg[i].value;
    esp_err_t err = gpio_hold_dis((gpio_num_t)pin);
    if(err != ESP_OK) {
      LOG("[GPIO] Failed to disable hold on GPIO %d: %s", pin, esp_err_to_name(err));
    }
    pinMode(pin, OUTPUT);
    if(mode == 0) {
      gpio_pullup_dis((gpio_num_t)pin);
      gpio_pulldown_dis((gpio_num_t)pin);
      digitalWrite(pin, value == 1 ? HIGH : LOW);
    } else if(mode == 1) {
      gpio_pullup_en((gpio_num_t)pin);
      gpio_pulldown_dis((gpio_num_t)pin);
    } else if(mode == 2) {
      gpio_pullup_dis((gpio_num_t)pin);
      gpio_pulldown_en((gpio_num_t)pin);
    } else if(mode == 3) {
      gpio_pullup_en((gpio_num_t)pin);
      gpio_pulldown_en((gpio_num_t)pin);
    }
    err = gpio_hold_en((gpio_num_t)pin);
    if(err != ESP_OK) {
      LOG("[GPIO] Failed to enable hold on GPIO %d: %s", pin, esp_err_to_name(err));
    } else {
      LOG("[GPIO] GPIO %d initialized as mode %d with value %d and hold enabled", pin, mode, value);
    }
  }
}
#endif // SUPPORT_GPIO

// button handling settings
#define BUTTON_DEBOUNCE_MS       30
#define BUTTON_SHORT_PRESS_MS    60
#define BUTTON_NORMAL_PRESS_MS  120
#define BUTTON_LONG_PRESS_1_MS 2000
#define BUTTON_LONG_PRESS_2_MS 5000

// Button configuration
unsigned long press_start = 0;
unsigned long press_duration = 0;
uint8_t last_button_state = 0;
uint8_t button_action = 0;

void determine_button_state() {
  LOOP_D("[BUTTON] Checking button state");

  // check/time button press
  last_button_state = digitalRead(current_button) == LOW;
  if(last_button_state) {
    if(press_start == 0) {
      D("[BUTTON] Button press start time not set, setting to now");
      press_start = millis();
      press_duration = 0;
    } else {
      press_duration = millis() - press_start;
    }
    D("[BUTTON] Button is currently pressed, %lu ms so far", press_duration);
  } else {
    if(press_start != 0) {
      D("[BUTTON] Button was pressed, but now released, clearing press start time, duration: %lu ms\n", press_duration);
      press_duration = millis() - press_start;
      press_start = 0;
    } else {
      LOOP_D("[BUTTON] Button is currently not pressed");
      press_duration = 0;
    }
  }

  // read current state
  LOOP_D("[BUTTON] Current button state: %s, action: %d, %lu ms", last_button_state ? "PRESSED" : "RELEASED", button_action, press_duration);

  // State machine for button press handling
  if(last_button_state && button_action == 0) {
    button_action = 1;
    // pressed, just continue timing
    LOG("[BUTTON] Button pressed, duration: %lu ms", press_duration);
    #ifdef LED
    LOG("[LED] Button pressed, switching to quick blink for long press section");
    set_led_blink(LED_BLINK_INTERVAL_QUICK);
    #endif // LED
    return;
  }
  if(!last_button_state && button_action == 1) {
    button_action = 0;
    // Button released, use the timed duration to decide action
    LOG("[BUTTON] Button released after press, duration: %lu ms", press_duration);
    if(press_duration < BUTTON_DEBOUNCE_MS) {
      // Ignore very short presses (debounce)
      LOG("[BUTTON] Press duration too short (%lu ms), ignoring", press_duration);
    } else if (BUTTON_DEBOUNCE_MS <= press_duration && press_duration < BUTTON_SHORT_PRESS_MS) {
      // reset button pressed flag
      LOG("[BUTTON] Short press detected (%lu ms)", press_duration);

      // Short press - stop BLE advertising if active
      #if defined(SUPPORT_BLE_UART1) || defined(BLUETOOTH_UART_AT)
      if (ble_advertising_start != 0) {
        LOG("[BUTTON] Short press detected (%lu ms), stopping BLE advertising if active", press_duration);
        // BLE is currently enabled, stop advertising
        stop_advertising_ble();
        LOG("[BUTTON] BLE advertising stopped");
        #ifdef LED
        set_led_blink(LED_BLINK_OFF);
        #endif // LED
      }
      #endif

      #ifdef WIFI_WPS
      // Short press - stop WPS if active
      if(wps_running) {
        LOG("[BUTTON] Short press detected (%lu ms), stopping WPS", press_duration);
        if(stop_wps())
          reset_networking();
        #ifdef LED
        set_led_blink(LED_BLINK_OFF);
        #endif // LED
      }
      #endif // WIFI_WPS
    } else if (BUTTON_SHORT_PRESS_MS <= press_duration && press_duration < BUTTON_LONG_PRESS_1_MS) {
      // reset button pressed flag
      LOG("[BUTTON] Normal press detected (%lu ms)", press_duration);
      #ifdef SUPPORT_BLE_UART1
      LOG("[BUTTON] Checking BLE UART1 bridge mode for normal press");
      if (cfg.ble_uart1_bridge == 1) {
        // BLE UART1 bridge is enabled and in bridge mode
        if(at_mode == BRIDGE_MODE) {
          // Switch to AT mode
          ble_uart1_at_mode(AT_MODE);
          ble_advertising_start = millis();
          LOG("[BUTTON] BLE AT Mode enabled");
        } else {
          // Switch to Bridge mode
          ble_uart1_at_mode(BRIDGE_MODE);
          ble_advertising_start = millis();
          LOG("[BUTTON] BLE Bridge Mode enabled");
        }
      } else {
        // Normal press - toggle BLE advertising as before
        if (ble_advertising_start == 0) {
          start_advertising_ble();
          LOG("[BUTTON] BLE advertising started - will stop on timeout if no connection, or when button pressed again");
          #ifdef LED
          set_led_blink(LED_BLINK_INTERVAL_FAST);
          #endif // LED
        } else {
          LOG("[BUTTON] Normal press detected (%lu ms), stopping BLE advertising if active", press_duration);
          stop_advertising_ble();
          LOG("[BUTTON] BLE advertising stopped");
          #ifdef LED
          set_led_blink(LED_BLINK_OFF);
          #endif // LED
        }
      }
      #else
      #if defined(BLUETOOTH_UART_AT)
      LOG("[BUTTON] Checking BLE for normal press");
      // Normal press - toggle BLE advertising as before
      if (ble_advertising_start == 0) {
        start_advertising_ble();
        LOG("[BUTTON] BLE advertising started - will stop on timeout if no connection, or when button pressed again");
        #ifdef LED
        set_led_blink(LED_BLINK_INTERVAL_FAST);
        #endif // LED
      } else {
        LOG("[BUTTON] Normal press detected (%lu ms), stopping BLE advertising if active", press_duration);
        stop_advertising_ble();
        LOG("[BUTTON] BLE advertising stopped");
        #ifdef LED
        set_led_blink(LED_BLINK_OFF);
        #endif // LED
      }
      #endif
      #endif
    } else if (BUTTON_LONG_PRESS_1_MS <= press_duration && press_duration < BUTTON_LONG_PRESS_2_MS) {
      // Long press - handle WPS
      LOG("[BUTTON] Long press detected (%lu ms), handling WPS", press_duration);
      #ifdef WIFI_WPS
      LOG("[BUTTON] Starting WPS");
      start_wps(NULL);
      #ifdef LED
      set_led_blink(LED_BLINK_INTERVAL_SLOW);
      #endif // LED
      #endif // WIFI_WPS
    #ifdef SUPPORT_BLE_UART1
    } else if (press_duration >= BUTTON_LONG_PRESS_2_MS) {
      // disable a UART1 BLE BRIDGE passthrough
      // Force AT mode
      ble_uart1_at_mode(AT_MODE);
      start_advertising_ble();
    #endif // SUPPORT_BLE_UART1
    }
    return;
  }
  if(!last_button_state && button_action > 1) {
    // Button released after action taken, reset state
    LOG("[BUTTON] Button released after action taken, resetting state");
    button_action = 0;
    return;
  }
  // No state change, ignore
  LOOP_D("[BUTTON] No state change detected, ignoring");
}

void setup_button() {
  #ifdef USE_BUTTON
  current_button = USE_BUTTON;
  #else
  #ifdef LOOP_DELAY
  // LOOP_DELAY is defined, we cant use GPIO_NUM_9 (builtin button) as it
  // is not supported as a wakeup source on ESP32c3/ESP32s3 with light sleep
  // so we pick a different pin, GPIO3_NUM_3/A3
  current_button = GPIO_NUM_3;
  #else
  current_button = BUTTON_BUILTIN;
  #endif
  #endif
  pinMode(current_button, INPUT_PULLUP);
  LOG("[BUTTON] Pin %d configured as INPUT_PULLUP", current_button);
}

#define WATCHDOG_TIMEOUT_MS 5000 // 1 second timeout
esp_task_wdt_user_handle_t wdt_user_handle;

void WDT(void) {
  // CRITICAL: Must use abort() not esp_restart() in ISR context!
  // esp_restart() is NOT ISR-safe and will trigger Interrupt WDT
  Serial.println("[WDT] 1 Task watchdog timeout occurred!");
}

NOINLINE
void setup_wtd(){
  esp_err_t ok;

  // First, deinitialize the Arduino core's WDT if it exists
  esp_task_wdt_deinit();

  // Now initialize with our custom config
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WATCHDOG_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = false,  // Set to false to allow custom handler to be called
  };
  ok = esp_task_wdt_init(&wdt_config);
  if(ok != ESP_OK) {
    LOG("[WDT] Failed to initialize task watchdog: %s", esp_err_to_name(ok));
    return;
  }
  LOG("[WDT] Task watchdog initialized with timeout %d ms, trigger_panic=false", WATCHDOG_TIMEOUT_MS);

  // Register watchdog timeout callback
  ok = esp_task_wdt_add_user((char*)"main", &wdt_user_handle);
  if(ok != ESP_OK) {
    LOG("[WDT] Failed to register task: %s", esp_err_to_name(ok));
    return;
  } else {
    LOG("[WDT] Watchdog timeout callback registered successfully");
  }
}

RTC_DATA_ATTR int boot_count = 0;

INLINE
void do_setup() {
  // setup WTD
  setup_wtd();

  // setup nvs
  setup_nvs();

  // setup cfg
  setup_cfg();

  // Setup AT command handler
  #ifdef UART_AT
  ATSc.SetDefaultHandler(&sc_cmd_handler);
  #endif

  // BlueTooth SPP setup possible?
  #if defined(BLUETOOTH_UART_AT) || defined(SUPPORT_BLE_UART1)
  setup_ble();
  // Set BLE UART1 bridge as default if enabled
  #ifdef SUPPORT_BLE_UART1
  // Bridge mode by default
  if(cfg.ble_uart1_bridge == 1) {
    ble_uart1_at_mode(BRIDGE_MODE);
    start_advertising_ble();
  }
  #endif
  #endif

  #ifdef BT_CLASSIC
  LOG("[BT] setting up Bluetooth Classic");
  SerialBT.begin(BLUETOOTH_UART_DEVICE_NAME);
  SerialBT.setPin(BLUETOOTH_UART_DEFAULT_PIN);
  SerialBT.register_callback(BT_EventHandler);
  ATScBT.SetDefaultHandler(&sc_cmd_handler);
  // Log Bluetooth Classic MAC address
  LOG("[BT] Classic MAC address: %s", SerialBT.getMacString().c_str());
  #endif

  // setup WiFi with ssid/pass if set
  #ifdef SUPPORT_WIFI
  setup_wifi();
  #endif // SUPPORT_WIFI

  #ifdef SUPPORT_UART1
  // use UART1 with configurable parameters
  setup_uart1();
  #endif // SUPPORT_UART1

  #ifdef SUPPORT_GPIO
  // GPIO setup from config
  setup_gpio();
  #endif // SUPPORT_GPIO

  #ifdef LED
  // Setup LED with PWM for brightness control
  setup_led();
  #endif // LED

  // Button setup
  setup_button();

  // setup power management
  setup_power_management();

  // set CPU to 160 MHz if possible
  setup_cpu_speed(160);
}

#ifdef SUPPORT_ESP_LOG_INFO
void do_esp_log() {
  #ifdef VERBOSE
  if(cfg.do_verbose == 0)
    return;
  #endif
  if(cfg.esp_log_interval == 0)
    return;
  // Log ESP info periodically when DEBUG is enabled
  LOOP_D("[LOOP] ESP info log check");
  if(last_esp_info_log == 0 || millis() - last_esp_info_log > cfg.esp_log_interval) { // Log every 30 seconds
    log_esp_info();
    last_esp_info_log = millis();
  }
}
#endif // SUPPORT_ESP_LOG_INFO

#ifdef SUPPORT_WIFI
#define WIFI_RECONNECT_INTERVAL 30000
#define WIFI_LOG_CHECK_INTERVAL 60000
#define WIFI_LOG_INTERVAL       60000
INLINE
void do_wifi_check() {
  LOOP_D("[LOOP] WiFi check, restart: %d, %d ms since last check", restart_networking, last_wifi_check == 0 ? 0 : millis() - last_wifi_check);
  if(restart_networking == 1){
    LOG("[WiFi] Restarting networking as requested");
    restart_networking = 0;
    last_wifi_reconnect = millis();
    setup_wifi();
  } else {
    if(cfg.wifi_enabled && strlen(cfg.wifi_ssid) != 0 && (last_wifi_check == 0 || millis() - last_wifi_check > WIFI_LOG_CHECK_INTERVAL)){
      last_wifi_check = millis();
      if(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS){
        LOG("[WiFi] Connected to SSID: %s", cfg.wifi_ssid);
        last_wifi_reconnect = millis();
      } else {
        // not connected, try to reconnect
        if(last_wifi_reconnect == 0 || millis() - last_wifi_reconnect > WIFI_RECONNECT_INTERVAL) {
          last_wifi_reconnect = millis();
          LOG("[WiFi] Not connected, attempting to reconnect to SSID: %s, status: %d", cfg.wifi_ssid, WiFi.status());
          setup_wifi();
        }
      }
    }
  }

  if(cfg.wifi_enabled && strlen(cfg.wifi_ssid) != 0 && (last_wifi_info_log == 0 || millis() - last_wifi_info_log > WIFI_LOG_INTERVAL)) {
    last_wifi_info_log = millis();
    #ifdef VERBOSE
    if(cfg.do_verbose)
      log_wifi_info("[LOOP]");
    #endif
  }
}
#endif // SUPPORT_WIFI

#if defined(SUPPORT_WIFI) && (defined(SUPPORT_TCP) || defined(SUPPORT_TLS))
INLINE
void do_connections_check() {
  // TCP connection check at configured interval
  LOOP_D("[LOOP] TCP/TLS check");
  if(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS) {
    // connected, check every 500ms
    if(last_tcp_check == 0 || millis() - last_tcp_check > 500) {
      last_tcp_check = millis();
      #if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP)
      if(strlen(cfg.tcp_host_ip) != 0 && cfg.tcp_port != 0) {
        doYIELD;
        int conn_ok = check_tcp_connection(0);
        if(!conn_ok) {
          D("[LOOP] TCP Connection lost");
          connect_tcp();
        }
      }
      #endif // SUPPORT_WIFI && SUPPORT_TCP

      #ifdef SUPPORT_TLS
      if(cfg.tls_enabled && strlen(cfg.tcp_host_ip) != 0 && (cfg.tcp_port != 0 || cfg.tls_port != 0)) {
        doYIELD;
        int tls_conn_ok = check_tls_connection();
        if(!tls_conn_ok) {
          D("[LOOP] TLS Connection lost");
          connect_tls();
        }
      }
      #endif // SUPPORT_TLS
    }
  }
}
#endif // SUPPORT_TCP || SUPPORT_TLS

#ifdef SUPPORT_NTP
#define NTP_LOG_INTERVAL 600000
INLINE
void do_ntp_check() {
  // NTP check
  LOOP_D("[LOOP] NTP check");
  if(last_ntp_log == 0 || millis() - last_ntp_log > NTP_LOG_INTERVAL) {
    last_ntp_log = millis();
    if((WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS) && cfg.ntp_host[0] != 0 && esp_sntp_enabled()) {
      doYIELD;
      // check if synced
      D("[NTP] Checking NTP sync status");
      if(sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
        // synced
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);
        if(timeinfo.tm_hour != last_hour) {
          LOG("[NTP] NTP is synced, current time: %s", COMMON::PT());
          last_hour = timeinfo.tm_hour;
        }
      } else if(sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && last_hour != -1) {
        D("[NTP] NTP sync ok");
      } else {
        D("[NTP] not yet synced");
      }
    }
  }
}
#endif // SUPPORT_NTP

#ifdef SUPPORT_TCP_SERVER
INLINE
void do_tcp_server_check() {
  // if not configured, skip
  if((tcp_server_sock == -1 && tcp6_server_sock == -1) || (cfg.tcp_server_port == 0 && cfg.tcp6_server_port == 0)) {
    // no TCP server configured
    return;
  }

  // TCP Server handling
  LOOP_D("[LOOP] Check TCP server connections");
  if(tcp_server_sock != -1) {
    handle_tcp_server();
    // Update last activity time if we have clients
    if(get_tcp_server_client_count() > 0) {
      #ifdef LED
      last_activity = millis(); // Trigger LED activity for TCP server
      #endif // LED
    }
  }

  // TCP6 Server handling
  LOOP_D("[LOOP] Check TCP6 server connections");
  if(tcp6_server_sock != -1) {
    handle_tcp6_server();
    // Update last activity time if we have clients
    if(get_tcp_server_client_count() > 0) {
      #ifdef LED
      last_activity = millis(); // Trigger LED activity for TCP6 server
      #endif // LED
    }
  }

  if (tcp_server_sock != -1 || tcp6_server_sock != -1) {
    if(inlen > 0) {
      LOOP_D("[LOOP] TCP_SERVER Sending data to clients, len: %d", inlen);
      int clients_sent = send_tcp_server_data((const uint8_t*)inbuf, inlen);
      if (clients_sent > 0) {
        #ifdef LED
        last_activity = millis(); // Trigger LED activity for TCP server send
        #endif // LED
        D("[TCP_SERVER] Sent %d bytes to %d clients, data: >>%s<<", inlen, clients_sent, inbuf);
        sent_ok |= 1; // mark as sent
      } else {
        LOOP_D("[TCP_SERVER] No clients connected to send data to");
        // Don't mark as error if no clients are connected
      }
    }

    if (outlen + TCP_READ_SIZE > sizeof(outbuf)) {
      D("[TCP_SERVER] outbuf full, cannot read more data, outlen: %d", outlen);
    } else {
      LOOP_D("[LOOP] TCP_SERVER Checking for incoming data");
      int r = recv_tcp_server_data((uint8_t*)outbuf + outlen, TCP_READ_SIZE);
      if (r > 0) {
        // data received
        #ifdef LED
        last_activity = millis(); // Trigger LED activity for TCP server receive
        #endif // LED
        D("[TCP_SERVER] Received %d bytes, total: %d, data: >>%s<<", r, outlen + r, outbuf);
        outlen += r;
      } else if (r == 0) {
        // connection closed by remote host
        LOG("[TCP_SERVER] connection closed by remote host");
      } else if (r == -1) {
        // error occurred, check errno
        if(errno && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINPROGRESS) {
          // Error occurred, log it
          LOGE("[TCP_SERVER] closing connection");
        } else {
          // No data available, just yield
          LOOP_E("[TCP_SERVER] no data available", errno);
        }
      }
    }
  }

  LOOP_D("[LOOP] Check TCP_SERVER disconnects");
  if(tcp_server_sock != -1 || tcp6_server_sock != -1)
    handle_tcp_server_disconnects();
}
#endif // SUPPORT_TCP_SERVER

#ifdef SUPPORT_UDP

NOINLINE
bool have_ip_address(bool do_log = false){
  // check if we have an IP address, ipv4 or ipv6
  #ifdef SUPPORT_WIFI
  IPAddress local_ip = WiFi.localIP();
  if(do_log)
    LOG("[WIFI] checking for IPv4 address: '%s'", local_ip.toString().c_str());
  // Check for valid IPv4 address (not 0.0.0.0 and not 127.0.0.1)
  if(local_ip == IPAddress(0,0,0,0) || local_ip == IPAddress(127, 0, 0, 1))
    return false;
  if(cfg.ip_mode & IPV6_SLAAC) {
    IPAddress _ip6 = WiFi.globalIPv6();
    if(do_log)
      LOG("[WIFI] checking for IPv6 address: '%s'", _ip6.toString().c_str());
    if(_ip6 == IPAddress((uint32_t)0))
      return false;
    _ip6 = WiFi.linkLocalIPv6();
    if(do_log)
      LOG("[WIFI] checking for link-local IPv6 address: '%s'", _ip6.toString().c_str());
    if(_ip6 == IPAddress((uint32_t)0))
      return false;
  } else {
    if(do_log)
      LOG("[WIFI] IPv6 SLAAC not enabled, skipping IPv6 address check");
  }
  #endif // SUPPORT_WIFI
  return true;
}

NOINLINE
bool wifi_connected(bool do_log = false) {
  #ifdef SUPPORT_WIFI
  if(do_log)
    LOG("[WIFI] checking WiFi connection status: %d ?= %d|%d", WiFi.status(), WL_CONNECTED, WL_IDLE_STATUS);
  if(!(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS))
    return false;
  return true;
  #else
  return false;
  #endif // SUPPORT_WIFI
}

NOINLINE
bool fd_write_ok(const char *m, int sock, bool do_log = false) {
  // check for valid socket
  if(sock < 0) {
    if(do_log)
      LOGE("%sinvalid socket -1", m);
    return false;
  }
  // check if wifi is connected
  if(!wifi_connected(do_log)) {
    if(do_log)
      D("%sWiFi not connected, socket %d not writable", m, sock);
    return false;
  }
  // check if we have an IP address, ipv4 or ipv6
  if(!have_ip_address(do_log)) {
    if(do_log)
      D("%sNo IP address assigned, socket %d not writable", m, sock);
    return false;
  }

  // check socket for writability via select()
  fd_set writefds;
  fd_set errfds;
  FD_ZERO(&writefds);
  FD_ZERO(&errfds);
  FD_SET(sock, &writefds);
  FD_SET(sock, &errfds);

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  D("%schecking if socket %d is writable", m, sock);
  int ready = select(sock + 1, NULL, &writefds, &errfds, &timeout);
  if (ready < 0) {
    LOGE("%sselect error on socket %d", m, sock);
    return false;
  }
  return FD_ISSET(sock, &writefds) && !FD_ISSET(sock, &errfds) && ready > 0;
}

NOINLINE
void wait_for_wifi_connection(const char *m, int sock) {
  bool ok_to_send = fd_write_ok(m, sock, true);
  unsigned long start_wait = millis();
  while(!ok_to_send) {
    doYIELD;
    ok_to_send = fd_write_ok(m, sock, false);
    if(millis() - start_wait > 5000) {
      LOGE("%sTimeout waiting for WiFi connection on socket %d", m, sock);
      break;
    }
  }
}

INLINE
void do_udp_check() {
  // recreate UDP socks if config changed or -1

  // in/out udp receive/send socket
  if(strlen(cfg.udp_host_ip) > 0 || cfg.udp_port != 0)
    in_out_socket_udp(udp_sock);

  // receive-only udp socket (IPv4 only)
  if(cfg.udp_listen_port != 0 || cfg.udp6_listen_port != 0)
    in_socket_udp(udp_listen_sock, cfg.udp_listen_port?cfg.udp_listen_port:cfg.udp6_listen_port);

  // receive-only udp socket (IPv6 only)
  if(cfg.udp6_listen_port != 0)
    in_socket_udp6(udp6_listen_sock, cfg.udp6_listen_port?cfg.udp6_listen_port:cfg.udp_listen_port);

  // send-only udp socket
  if(strlen(cfg.udp_send_ip) > 0 || cfg.udp_send_port != 0)
    out_socket_udp(udp_out_sock, cfg.udp_send_port, cfg.udp_send_ip);

  // no UDP configured?
  if(udp_sock == -1 && udp_out_sock == -1 && udp_listen_sock == -1 && udp6_listen_sock == -1) {
    // no UDP configured
    return;
  }

  // UDP send
  LOOP_D("[LOOP] Check for outgoing UDP data fd: %d: inlen: %d", udp_sock, inlen);
  if(inlen > 0) {
    // in/out UDP socket send 
    if (udp_sock != -1){
      // send data
      int sent = send_udp_data(udp_sock, (const uint8_t*)inbuf, inlen, cfg.udp_host_ip, cfg.udp_port, "[UDP]");
      if (sent > 0) {
        #ifdef LED
        last_activity = millis(); // Trigger LED activity for UDP send
        #endif // LED
        D("[UDP] Sent %d bytes, total: %d, data: >>%s<<", sent, inlen, inbuf);
        sent_ok |= 1; // mark as sent
      } else if (sent < 0) {
        LOGE("[UDP] Sent error %d bytes, total: %d", sent, inlen);
      } else if (sent == 0) {
        D("[UDP] Sent 0 bytes, total: %d", inlen);
      }
    }

    // out UDP socket send
    if (udp_out_sock != -1){
      // send data
      int sent = send_udp_data(udp_out_sock, (const uint8_t*)inbuf, inlen, cfg.udp_send_ip, cfg.udp_send_port, "[UDP_SEND]");
      if (sent > 0) {
        #ifdef LED
        last_activity = millis(); // Trigger LED activity for UDP send
        #endif // LED
        D("[UDP_SEND] Sent %d bytes, total: %d, data: >>%s<<", sent, inlen, inbuf);
        sent_ok |= 1; // mark as sent
      } else if (sent < 0) {
        LOGE("[UDP_SEND] Sent error %d bytes, total: %d", sent, inlen);
      } else if (sent == 0) {
        D("[UDP_SEND] Sent 0 bytes, total: %d", inlen);
      }
    }
  }

  // UDP read
  LOOP_D("[LOOP] Check for incoming UDP data");

  // in/out UDP socket read
  udp_read(udp_sock, outbuf, outlen, UDP_READ_MSG_SIZE, REMOTE_BUFFER_SIZE, "[UDP]");

  // in UDP socket read
  udp_read(udp_listen_sock, outbuf, outlen, UDP_READ_MSG_SIZE, REMOTE_BUFFER_SIZE, "[UDP_LISTEN]");

  // in UDP6 socket read
  udp_read(udp6_listen_sock, outbuf, outlen, UDP_READ_MSG_SIZE, REMOTE_BUFFER_SIZE, "[UDP6_LISTEN]");
}
#endif // SUPPORT_UDP

#ifdef SUPPORT_TCP
INLINE
void do_tcp_check() {
  if(tcp_sock == -1) {
    // no TCP configured
    return;
  }

  // TCP send
  LOOP_D("[LOOP] Check for outgoing TCP data");
  if (tcp_sock != -1 && inlen > 0) {
    if (!tcp_connection_writable) {
      LOOP_D("[TCP] No valid connection, cannot send data");
    } else {
      int sent = send_tcp_data((const uint8_t*)inbuf, inlen);
      if (sent > 0) {
        #ifdef LED
        last_activity = millis(); // Trigger LED activity for TCP send
        #endif // LED
        D("[TCP] Sent %d bytes, total: %d", sent, inlen);
        sent_ok |= 1; // mark as sent
      } else if (sent == -1) {
        if(errno && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINPROGRESS) {
          // Error occurred, log it
          LOGE("[TCP] send error, closing connection");
        } else {
          // Socket not ready for writing, data will be retried on next loop
          E("[TCP] socket not ready for writing, will retry", errno);
        }
      } else if (sent == 0) {
        // Socket not ready for writing, data will be retried on next loop
        LOG("[TCP] connection closed by remote host");
      }
    }
  }

  // TCP read
  LOOP_D("[LOOP] Check for incoming TCP data");
  if (tcp_sock != -1 && tcp_connection_writable) {
    if (outlen + TCP_READ_SIZE > sizeof(outbuf)) {
      D("[TCP] outbuf full, cannot read more data, outlen: %d", outlen);
      // no space in outbuf, cannot read more data
      // just yield and wait for outbuf to be cleared
    } else {
      // no select(), just read from TCP socket and ignore ENOTCONN etc..
      int os = recv_tcp_data((uint8_t*)outbuf + outlen, TCP_READ_SIZE);
      if (os > 0) {
        // data received
        #ifdef LED
        last_activity = millis(); // Trigger LED activity for TCP receive
        #endif // LED
        D("[TCP] Received %d bytes, total: %d, data: >>%s<<", os, outlen + os, outbuf);
        outlen += os;
      } else if (os == 0) {
        // connection closed by remote host
        LOG("[TCP] connection closed by remote host");
      } else if (os == -1) {
        // error occurred, check errno
        if(errno && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINPROGRESS) {
          // Error occurred, log it
          LOGE("[TCP] closing connection");
        } else {
          // No data available, just yield
          LOOP_E("[TCP] no data available", errno);
        }
      }
    }
  }
}
#endif // SUPPORT_TCP

#ifdef SUPPORT_TLS
INLINE
void do_tls_check() {
  // TLS send (if TLS is enabled and connected)
  LOOP_D("[LOOP] Check for outgoing TLS data");
  if (cfg.tls_enabled && tls_connected && tls_handshake_complete && inlen > 0) {
    int sent = send_tls_data((const uint8_t*)inbuf, inlen);
    if (sent > 0) {
      #ifdef LED
      last_activity = millis(); // Trigger LED activity for TLS send
      #endif // LED
      D("[TLS] Sent %d bytes, total: %d", sent, inlen);
      sent_ok |= 1; // mark as sent
    } else if (sent == -1) {
      LOG("[TLS] send error or connection lost");
    } else if (sent == 0) {
      LOG("[TLS] connection closed by remote host");
    }
  } else if (cfg.tls_enabled && !tls_connected && inlen > 0) {
    D("[TLS] No valid TLS connection, cannot send data");
  }

  // TLS read
  LOOP_D("[LOOP] Check for incoming TLS data");
  if (cfg.tls_enabled && tls_connected && tls_handshake_complete) {
    if (outlen + TCP_READ_SIZE > sizeof(outbuf)) {
      D("[TLS] outbuf full, cannot read more data, outlen: %d", outlen);
      // no space in outbuf, cannot read more data
      // just yield and wait for outbuf to be cleared
    } else {
      int os = recv_tls_data((uint8_t*)outbuf + outlen, TCP_READ_SIZE);
      if (os > 0) {
        // data received
        #ifdef LED
        last_activity = millis(); // Trigger LED activity for TLS receive
        #endif // LED
        D("[TLS] Received %d bytes, total: %d, data: >>%s<<", os, outlen + os, outbuf);
        outlen += os;
      } else if (os == 0) {
        // connection closed by remote host
        LOG("[TLS] connection closed by remote host");
      } else if (os == -1) {
        // no data available or error
        LOOP_D("[TLS] no data available");
      }
    }
  }
}
#endif // SUPPORT_TLS

#ifdef SUPPORT_BLE_UART1
void do_ble_uart1_bridge() {
  // BLE <-> UART1 bridge enabled via AT command?
  if(cfg.ble_uart1_bridge == 0 || at_mode == AT_MODE)
    return; // BLE <-> UART1 bridge disabled

  // Bridge data between UART1 and BLE if connected
  LOOP_D("[LOOP] Check BLE <-> UART1 bridge");
  if(deviceConnected == 1) {
    // BLE connected, check if we have data from UART1 to send over BLE
    if(inlen > 0) {
      uint8_t ok_send = ble_send_n((const uint8_t *)inbuf, inlen);
      if(ok_send == 1) {
        D("[BLE] Sent %d bytes from UART1 to BLE", inlen);
        sent_ok |= 1; // mark as sent
      } else {
        LOG("[BLE] Failed to send %d bytes from UART1 to BLE", inlen);
      }
    }
  }

  // Check if we have data from BLE to send over UART1
  if(ble_rx_len > 0) {
    if(outlen + ble_rx_len <= REMOTE_BUFFER_SIZE) {
      memcpy(outbuf + outlen, ble_rx_buffer, ble_rx_len);
      D("[BLE] Received %d bytes from BLE to UART1, data: >>%s<<", ble_rx_len, ble_rx_buffer);
      outlen += ble_rx_len;

      // clear BLE buffer
      ble_rx_len = 0;
      memset(ble_rx_buffer, 0, sizeof(ble_rx_buffer));
    } else {
      LOGE("[BLE] Not enough space in outbuf to copy BLE data");
    }
  }
}
#endif

NOINLINE
void wakeup_reason(esp_sleep_wakeup_cause_t wakup_reason) {
  D("[SLEEP] Checking wakeup reason...");
  switch(wakup_reason) {
    case ESP_SLEEP_WAKEUP_UART:
      // woke up due to UART
      D("[SLEEP] Woke up due to UART");
      #ifdef SUPPORT_UART1
      do_uart1_read(); // read UART1 data immediately
      #endif // SUPPORT_UART1
      break;
    case ESP_SLEEP_WAKEUP_BT:
      // woke up due to BT
      D("[SLEEP] Woke up due to BT");
      break;
    case ESP_SLEEP_WAKEUP_WIFI:
      // woke up due to WiFi
      D("[SLEEP] Woke up due to WiFi");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      // woke up due to timer
      D("[SLEEP] Woke up due to timer");
      break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
      // undefined wakeup, should not happen, probably reset
      D("[SLEEP] Woke up due to undefined reason, probably reset or poweron");
      break;
    case ESP_SLEEP_WAKEUP_GPIO:
      // woke up due to GPIO, e.g. button press
      D("[SLEEP] Woke up due to GPIO (button press?)");
      break;
    case ESP_SLEEP_WAKEUP_VBAT_UNDER_VOLT:
      // woke up due to VBAT low voltage
      D("[SLEEP] Woke up due to VBAT under voltage");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      // woke up due to touchpad, should not happen
      D("[SLEEP] Woke up due to touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      // woke up due to ULP, should not happen
      D("[SLEEP] Woke up due to ULP");
      break;
    case ESP_SLEEP_WAKEUP_COCPU:
      // woke up due to COCPU, should not happen
      D("[SLEEP] Woke up due to COCPU");
      break;
    case ESP_SLEEP_WAKEUP_EXT0:
      // woke up due to EXT0, should not happen
      D("[SLEEP] Woke up due to EXT0");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      // woke up due to EXT1, should not happen
      D("[SLEEP] Woke up due to EXT1");
      break;
    default:
      // woke up due to other reason, e.g. button press
      D("[SLEEP] Woke up due to other reason: %d", wakup_reason);
      break;
  }
}

#ifdef LOOP_DELAY
#define USE_LIGHT_SLEEP 1 // allows for button press wakeup
#undef SUPPORT_DEEP_SLEEP
RTC_DATA_ATTR uint8_t sleepy_is_setup = 1;
RTC_DATA_ATTR unsigned long sleep_duration = 0;

NOINLINE
uint8_t super_sleepy(const unsigned long sleep_ms) {
  D("[SLEEP] Entering light sleep for %d ms", sleep_ms);
  esp_err_t err = ESP_OK;

  // Setup light sleep only once
  if(sleepy_is_setup == 0) {
      sleepy_is_setup = 1;
      D("[SLEEP] Setting up light sleep");

      D("[SLEEP] Disabling previous wakeup sources");
      err = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      if(err != ESP_OK) {
        LOG("[SLEEP] Failed to disable previous wakeup sources: %s", esp_err_to_name(err));
      }
  }

  // Wake up on button press: TODO: won't work with GPIO9 isn't a RTC GPIO
  // on esp32c3, we use GPIO3
  bool ok_btn = esp_sleep_is_valid_wakeup_gpio((gpio_num_t)current_button);
  if(ok_btn) {
    // enable wakeup on button pin
    D("[SLEEP] Enabling button wakeup on pin %d, current state: %d", current_button, last_button_state);
    err = gpio_wakeup_enable((gpio_num_t)current_button, (!last_button_state) ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
    if(err != ESP_OK) {
      LOG("[SLEEP] Failed to enable button wakeup LOW on pin %d: %s", current_button, esp_err_to_name(err));
      return 0;
    }
    err = esp_sleep_enable_gpio_wakeup();
    if(err != ESP_OK) {
      LOG("[SLEEP] Failed to enable button wakeup: %s", esp_err_to_name(err));
      return 0;
    }
    D("[SLEEP] Button wakeup enabled on pin %d", current_button);

    // Correct GPIO config for button pin PULLUP/DOWN, as we GND the button
    err = gpio_pullup_en((gpio_num_t)current_button);
    if(err != ESP_OK) {
      D("[SLEEP] Failed to disable pullup on pin %d: %s", current_button, esp_err_to_name(err));
    }
    err = gpio_pulldown_dis((gpio_num_t)current_button);
    if(err != ESP_OK) {
      D("[SLEEP] Failed to enable pulldown on pin %d: %s", current_button, esp_err_to_name(err));
    }
  } else {
    D("[SLEEP] Button wakeup not possible on pin %d", current_button);
    return 0;
  }

  // Wake up on UART activity
  err = esp_sleep_enable_uart_wakeup(UART_NUM_1);
  if(err != ESP_OK) {
    LOG("[SLEEP] Failed to enable UART wakeup: %s", esp_err_to_name(err));
    return 0;
  } else {
    D("[SLEEP] UART wakeup enabled");
  }

  // Wake up n WiFi activity
  err = esp_sleep_enable_wifi_wakeup();
  if(err != ESP_OK) {
    LOG("[SLEEP] Failed to enable WiFi wakeup: %s", esp_err_to_name(err));
    return 0;
  } else {
    D("[SLEEP] WiFi wakeup enabled");
  }

  // Configure timer
  D("[SLEEP] Configuring timer wakeup for %d ms, configured: %d", sleep_ms, cfg.main_loop_delay);
  err = esp_sleep_enable_timer_wakeup((uint64_t)sleep_ms * 1000ULL);
  if(err != ESP_OK) {
    LOG("[SLEEP] Failed to enable timer wakeup: %s", esp_err_to_name(err));
    return 0;
  } else {
    D("[SLEEP] Timer wakeup enabled for %d ms", sleep_ms);
  }

  #if defined(BLUETOOTH_UART_AT) || defined(SUPPORT_BLE_UART1)
  // disable bt controller to save power
  err = esp_bt_controller_disable();
  if(err != ESP_OK) {
    LOG("[SLEEP] Failed to disable BT controller: %s", esp_err_to_name(err));
  } else {
    D("[SLEEP] BT controller disabled to save power");
  }
  #endif

  #ifdef SUPPORT_WIFI
  stop_network_connections();
  /*
  wifi_mode_t current_mode;
  err = esp_wifi_get_mode(&current_mode);
  if(err == ESP_OK) {
    err = esp_wifi_stop();
    if(err != ESP_OK) {
      LOG("[SLEEP] Failed to stop WiFi: %s", esp_err_to_name(err));
    }
  }
  */
  #endif // SUPPORT_WIFI

  // Enable wakeup from UART, WiFi activity, and BUTTON
  uint8_t was_error = 0;
  sleep_duration = millis();
  LOGFLUSH();
  #ifdef SUPPORT_DEEP_SLEEP
  D("[SLEEP] Enabling deep sleep for %d ms", sleep_ms);
  LOGFLUSH();
  esp_deep_sleep_start();
  D("[SLEEP] Deep sleep failed, falling back to light sleep");
  // when we reach here, deep sleep failed and we continue to light sleep
  // when deep sleep is successful, the device resets on wakeup, and this
  // never gets reached
  #endif // SUPPORT_DEEP_SLEEP
  if(USE_LIGHT_SLEEP) {
    D("[SLEEP] Enabling light sleep for %d ms", sleep_ms);
    LOGFLUSH();
    err = esp_light_sleep_start();
    if(err != ESP_OK) {
      LOG("[SLEEP] Failed to enter light sleep: %s", esp_err_to_name(err));
      // A failure can be e.g. the button is still being pressed, in such
      // a case, we exit the sleep correctly, not failure to introduce
      // a delay() sleep
      was_error = 1;
    } else {
      // successful sleep, check duration
      sleep_duration = millis() - sleep_duration;
      wakeup_reason(esp_sleep_get_wakeup_cause());
    }
  } else {
    D("[SLEEP] Enabling regular sleep for %d ms", sleep_ms);
    LOGFLUSH();
    delay(sleep_ms);
  }

  // Re-enable WiFi and BT controller after wakeup
  #ifdef SUPPORT_WIFI
  if(cfg.wifi_enabled && strlen(cfg.wifi_ssid) != 0 && was_error == 0) {
    LOG("[SLEEP] Re-enabling WiFi after wakeup");
    err = esp_wifi_start();
    if(err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED) {
      LOG("[SLEEP] Failed to start WiFi: %s", esp_err_to_name(err));
    }
    LOG("[SLEEP] Waiting for WiFi to be ready");
  }
  #endif // SUPPORT_WIFI

  // Re-enable BT controller
  #if defined(BLUETOOTH_UART_AT) || defined(SUPPORT_BLE_UART1)
  err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if(err != ESP_OK) {
    LOG("[SLEEP] Failed to enable BT controller: %s", esp_err_to_name(err));
  } else {
    D("[SLEEP] BT controller re-enabled after wakeup");
  }
  #endif

  #ifdef SUPPORT_WIFI
  if(cfg.wifi_enabled && strlen(cfg.wifi_ssid) != 0)
    log_wifi_info("[SLEEP]");
  #endif // SUPPORT_WIFI

  D("[SLEEP] Releasing hold on LED pin %d", LED);
  err = gpio_hold_dis(LED);
  if(err != ESP_OK) {
    D("[SLEEP] Failed to release hold on LED pin %d: %s", LED, esp_err_to_name(err));
  } else {
    D("[SLEEP] Released hold on LED pin %d", LED);
  }

  // woke up
  D("[SLEEP] Woke up from light sleep after %lu ms, wanted: %lu", sleep_duration, sleep_ms);

  return 1;
}

#undef LOOP_DELAY_NO_LIGHT_SLEEP
#define LOOP_SLEEP_CUTOFF 100 // sleep smaller: delay(), longer: power save

NOINLINE
void do_sleep(const unsigned long sleep_ms) {

  // Yield at the start
  doYIELD;

  // The esp32c3 is however a single core esp32, so we need yield there as well
  // Use light sleep mode on ESP32 for better battery efficiency
  // Light sleep preserves RAM and allows faster wake-up
  // Light sleep disconnects WiFi/BLE
  #ifndef LOOP_DELAY_NO_LIGHT_SLEEP
  if(sleep_ms >= LOOP_SLEEP_CUTOFF)
    if(super_sleepy(sleep_ms) == 1)
      return; // successfully slept
  #endif // LOOP_DELAY_NO_LIGHT_SLEEP

  uint32_t c_cpu_f = getCpuFrequencyMhz();
  LOG("[SLEEP] Falling back to regular sleep for %d ms", sleep_ms);
  unsigned long start = millis();
  do {
    // We have data in input buffer, break out of sleep
    if(inlen > 0)
      break;

    // If button state changed, break out of sleep early
    #if defined(SUPPORT_BLE_UART1) || defined(BLUETOOTH_UART_AT)
    if((ble_advertising_start != 0 && deviceConnected == 0) || deviceConnected == 1) {
      if(deviceConnected == 1) {
        LOOP_D("[SLEEP] Wake up early due to button BLE:%d, inbuf:%d, %d ms, connected:%d", ble_advertising_start, inlen, sleep_ms - (millis() - start), deviceConnected);
      } else {
        D("[SLEEP] Wake up early due to button BLE:%d, inbuf:%d, %d ms, connected:%d", ble_advertising_start, inlen, sleep_ms - (millis() - start), deviceConnected);
      }
      break;
    }
    #endif

    // Sleep in small increments to allow wake-up, at low CPU freq
    setCpuFrequencyMhz(10);
    delayMicroseconds(100);
    setCpuFrequencyMhz(c_cpu_f);

    // Yield to allow background tasks to run
    doYIELD;
  } while (millis() - start < sleep_ms);

  LOG("[SLEEP] Slept for %d ms (requested %d ms)", millis() - start, sleep_ms);

  // Final yield
  doYIELD;
}

unsigned long loop_start_millis = 0;
NOINLINE
void do_loop_delay() {
  // no delay configured?
  int loop_delay = cfg.main_loop_delay;
  if(loop_delay <= 0)
    return;

  // If button pressed, do not sleep
  if(last_button_state)
    return;

  // WPS running, do not sleep
  #ifdef WIFI_WPS
  if(wps_running)
    return;
  #endif // WIFI_WPS

  // if we have UDP/TCP/TLS connections or config, never sleep
  #ifdef defined(SUPPORT_TCP) || defined(SUPPORT_UDP) || defined(SUPPORT_TLS) || defined(SUPPORT_TCP_SERVER)
  if(
#ifdef SUPPORT_TCP
         tcp_sock != -1
#endif
#ifdef SUPPORT_TLS
      || (cfg.tls_enabled && tls_connected)
#endif
#ifdef SUPPORT_UDP
      || udp_sock != -1
      || udp_out_sock != -1
      || udp_listen_sock != -1
      || udp6_listen_sock != -1
#endif
#ifdef SUPPORT_TCP_SERVER
      || tcp_server_sock != -1
      || tcp6_server_sock != -1
#endif
) {
    LOOP_D("[LOOP] Skipping delay due to active TCP/UDP/TLS connections");
    return;
  }
  #endif

  // BLE connected or advertising, or data in inbuf?
  #if defined(SUPPORT_BLE_UART1) || defined(BLUETOOTH_UART_AT)
  if(deviceConnected == 1 || ble_advertising_start != 0)
    return;
  #endif // SUPPORT_BLE_UART1
  if(inlen != 0)
    return;

  // If WiFi is enabled, but no IP address yet, don't sleep
  #ifdef SUPPORT_WIFI
  if(cfg.wifi_enabled && strlen(cfg.wifi_ssid) != 0) {
    // check if wifi is connected
    if(!wifi_connected(false))
      return;
    // check if we have an IP address, ipv4 or ipv6
    if(!have_ip_address(false))
      return;
    // log that we are connected and can sleep
    D("[LOOP] WiFi enabled, configured and connected and IP address assigned, can sleep");
  }
  #endif // SUPPORT_WIFI


  // delay and yield, check the loop_start_millis on how long we
  // should still sleep
  loop_start_millis = millis() - loop_start_millis;
  long delay_time = (long)loop_delay - (long)loop_start_millis;
  // check other "timeouts" for a possible smaller delay, like the log_esp_info() or timelog
  #ifdef SUPPORT_ESP_LOG_INFO
  if(!(cfg.do_verbose == 0 || cfg.esp_log_interval == 0)) {
    D("[LOOP] ESP info log check, last: %d, now: %d, diff: %d", last_esp_info_log, millis(), millis() - last_esp_info_log);
    delay_time = min(delay_time, (long int)(
        (last_esp_info_log == 0 || (millis() - last_esp_info_log)) > cfg.esp_log_interval
        ? 0
        : cfg.esp_log_interval - (millis() - last_esp_info_log)
    ));
  }
  #endif // SUPPORT_ESP_LOG_INFO
  #ifdef SUPPORT_WIFI
  if(cfg.wifi_enabled && strlen(cfg.wifi_ssid) != 0) {
    D("[LOOP] WiFi check, last: %d, now: %d, diff: %d", last_wifi_check, millis(), millis() - last_wifi_check);
    delay_time = min(delay_time, (long int)(
        (last_wifi_check == 0 || (millis() - last_wifi_check)) > WIFI_LOG_CHECK_INTERVAL
        ? 0
        : WIFI_LOG_CHECK_INTERVAL - (millis() - last_wifi_check)
    ));
    D("[LOOP] WiFi info log check, last: %d, now: %d, diff: %d", last_wifi_info_log, millis(), millis() - last_wifi_info_log);
    delay_time = min(delay_time, (long int)(
        (last_wifi_info_log == 0 || (millis() - last_wifi_info_log)) > WIFI_LOG_INTERVAL
        ? 0
        : WIFI_LOG_INTERVAL - (millis() - last_wifi_info_log)
    ));
  }
  #endif // SUPPORT_WIFI
  #ifdef SUPPORT_NTP
  if(cfg.ntp_host[0] != 0 && esp_sntp_enabled()) {
    D("[LOOP] NTP check, last: %d, now: %d, diff: %d", last_ntp_log, millis(), millis() - last_ntp_log);
    delay_time = min(delay_time, (long int)(
        (last_ntp_log == 0 || (millis() - last_ntp_log)) > NTP_LOG_INTERVAL
        ? 0
        : NTP_LOG_INTERVAL - (millis() - last_ntp_log)
    ));
  }
  #endif // SUPPORT_NTP
  #ifdef TIMELOG
  if(cfg.do_timelog) {
    D("[LOOP] Time log check, last: %d, now: %d, diff: %d", last_time_log, millis(), millis() - last_time_log);
    delay_time = min(delay_time, (long int)(
        (last_time_log == 0 || (millis() - last_time_log)) > TIMELOG_INTERVAL
        ? 0
        : TIMELOG_INTERVAL - (millis() - last_time_log)
    ));
  }
  #endif // TIMELOG
  #ifdef SUPPORT_PLUGINS
  if(PLUGINS::max_sleep_time) {
    long plugin_delay = PLUGINS::max_sleep_time();
    D("[LOOP] Plugins check, current delay time: %d ms, plugin max sleep: %lu", delay_time, plugin_delay);
    delay_time = min(delay_time, (long int)plugin_delay);
  }
  #endif // SUPPORT_PLUGINS

  // add 5ms, so we don't go to sleep for 1,2,.. ms as we woke up too early
  D("[LOOP] Calculated delay time: %d ms", delay_time);
  if(delay_time < 0)
    delay_time = 0;

  LOOP_D("[LOOP] delay for tm: %d, wa: %d, wt: %d, len: %d, ble: %s", loop_start_millis, loop_delay, delay_time, inlen, ble_advertising_start != 0 ? "y" : "n");
  if(delay_time > 0)
    do_sleep((unsigned long)delay_time);
}
#endif // LOOP_DELAY

INLINE
void log_supported_features(){
#ifdef SUPPORT_UDP
  LOG("[SETUP] SUPPORT_UDP enabled");
#else
  LOG("[SETUP] SUPPORT_UDP disabled");
#endif
#ifdef SUPPORT_TCP
  LOG("[SETUP] SUPPORT_TCP enabled");
#else
  LOG("[SETUP] SUPPORT_TCP disabled");
#endif
#ifdef SUPPORT_TLS
  LOG("[SETUP] SUPPORT_TLS enabled");
#else
  LOG("[SETUP] SUPPORT_TLS disabled");
#endif
#ifdef SUPPORT_TCP_SERVER
  LOG("[SETUP] SUPPORT_TCP_SERVER enabled");
#else
  LOG("[SETUP] SUPPORT_TCP_SERVER disabled");
#endif
#ifdef SUPPORT_WIFI
  LOG("[SETUP] SUPPORT_WIFI enabled");
#else
  LOG("[SETUP] SUPPORT_WIFI disabled");
#endif
#ifdef SUPPORT_BLE_UART1
  LOG("[SETUP] SUPPORT_BLE_UART1 enabled");
#else
  LOG("[SETUP] SUPPORT_BLE_UART1 disabled");
#endif
#ifdef SUPPORT_PLUGINS
  LOG("[SETUP] SUPPORT_PLUGINS enabled");
#else
  LOG("[SETUP] SUPPORT_PLUGINS disabled");
#endif
#ifdef UART_AT
  LOG("[SETUP] UART_AT enabled");
#else
  LOG("[SETUP] UART_AT disabled");
#endif
#ifdef SUPPORT_MDNS
  LOG("[SETUP] SUPPORT_MDNS enabled");
#else
  LOG("[SETUP] SUPPORT_MDNS disabled");
#endif
#ifdef SUPPORT_NTP
  LOG("[SETUP] SUPPORT_NTP enabled");
#else
  LOG("[SETUP] SUPPORT_NTP disabled");
#endif
#ifdef SUPPORT_ESP_LOG_INFO
  LOG("[SETUP] SUPPORT_ESP_LOG_INFO enabled");
#else
  LOG("[SETUP] SUPPORT_ESP_LOG_INFO disabled");
#endif
#ifdef LOGUART
  LOG("[SETUP] LOGUART enabled");
#else
  LOG("[SETUP] LOGUART disabled");
#endif
#ifdef TIMELOG
  LOG("[SETUP] TIMELOG enabled");
#else
  LOG("[SETUP] TIMELOG disabled");
#endif
}

void setup() {
  // Serial setup, init at 115200 8N1
  LOGSETUP();

  // check wakeup reason
  esp_sleep_wakeup_cause_t w_reason = esp_sleep_get_wakeup_cause();
  esp_reset_reason_t r_reason = esp_reset_reason();

  // enable all ESP32 core logging
  #ifdef DEBUG
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  #endif

  // was deep sleep?
  LOG("[SETUP] Boot number: %d, wakeup reason: %d, reset: %d", boot_count++, w_reason, r_reason);
  if(boot_count > 1){
    // woke up from deep sleep
    wakeup_reason(w_reason);

    // re-setup after deep sleep
    do_setup();

    // plugins setup
    #ifdef SUPPORT_PLUGINS
    if(PLUGINS::setup){
      LOG("[SETUP] Re-setup plugins after deep sleep");
      PLUGINS::setup();
    }
    #endif // SUPPORT_PLUGINS

    LOG("[SETUP] Re-setup done after deep sleep");
    return;
  }

  // initial setup
  do_setup();

  // log info
  #ifdef SUPPORT_ESP_LOG_INFO
  log_esp_info();
  #endif // SUPPORT_ESP_LOG_INFO

  // log the SUPPORT builtin
  log_supported_features();

  #ifdef SUPPORT_PLUGINS
  if(PLUGINS::setup){
    LOG("[SETUP] Running plugins setup");
    PLUGINS::setup();
  }
  #endif // SUPPORT_PLUGINS

  // plugins initialize
  #ifdef SUPPORT_PLUGINS
  if(PLUGINS::initialize){
    LOG("[SETUP] Initializing plugins");
    PLUGINS::initialize();
  }
  #endif // SUPPORT_PLUGINS

  LOG("[SETUP] Setup done, entering main loop");
}

void loop() {
  LOOP_D("[LOOP] Start main loop");
  sent_ok = 0;

  // Reset the watchdog timer
  esp_err_t ok = esp_task_wdt_reset_user(wdt_user_handle);
  if(ok != ESP_OK) {
    LOG("[WDT] Failed to reset WDT: %s", esp_err_to_name(ok));
  }

  #ifdef LOOP_DELAY
  loop_start_millis = millis();
  #endif // LOOP_DELAY

  #if defined(SUPPORT_WIFI) && defined(WIFI_WPS)
  // Check WPS timeout
  if (wps_running && (millis() - wps_start_time > WPS_TIMEOUT_MS)) {
    LOG("[WPS] WPS timeout reached, stopping WPS");
    if(stop_wps())
      reset_networking();
  }
  #endif // SUPPORT_WIFI && WIFI_WPS

  #ifdef UART_AT
  // Handle Serial AT commands
  while(ATSc.GetSerial()->available() > 0)
    ATSc.ReadSerial();
  #endif

  #ifdef SUPPORT_BLE_UART1
  // Check if BLE advertising should be stopped after timeout
  // Only stop on timeout if no device is connected - once connected,
  // wait for remote disconnect or button press, as BLE UART1 is supported
  // only in AT_MODE, stop advertising
  if(at_mode == AT_MODE){
    if(ble_advertising_start != 0
        && deviceConnected == 0
        && millis() - ble_advertising_start > BLE_ADVERTISING_TIMEOUT) {
      // stop BLE
      stop_advertising_ble();
      // start WIFI if enabled
      #ifdef SUPPORT_WIFI
      if(cfg.wifi_enabled == 1)
        esp_wifi_start();
      #endif
    }
  } else {
    if(ble_advertising_start == 0
        && deviceConnected == 0
        && cfg.ble_uart1_bridge == 1) {
      // stop WIFI if enabled
      #ifdef SUPPORT_WIFI
      if(cfg.wifi_enabled == 1)
        esp_wifi_stop();
      #endif
      // start BLE
      start_advertising_ble();
    }
  }
  #else
  #ifdef BLUETOOTH_UART_AT
  // Check if BLE advertising should be stopped after timeout
  // Only stop on timeout if no device is connected - once connected,
  // wait for remote disconnect or button press.
  //
  // We dont start BLE advertising, as we don't have BLE UART1 bridge support
  // BLE Advertising is started using the button press only
  if (ble_advertising_start != 0
      && deviceConnected == 0
      && millis() - ble_advertising_start > BLE_ADVERTISING_TIMEOUT) {
    stop_advertising_ble();
    // start WIFI if enabled
    #ifdef SUPPORT_WIFI
    if(cfg.wifi_enabled == 1)
      esp_wifi_start();
    #endif
  }
  #endif // BLUETOOTH_UART_AT
  #endif // SUPPORT_BLE_UART1

  #ifdef BLUETOOTH_UART_AT
  handle_ble_commands();
  #endif // BLUETOOTH_UART_AT

  // Plugins pre-loop
  #ifdef SUPPORT_PLUGINS
  if(PLUGINS::loop_pre){
    LOOP_D("[PLUGINS] Running plugins PRE hooks");
    PLUGINS::loop_pre();
  }
  #endif // SUPPORT_PLUGINS

  #ifdef TIMELOG
  // TIMELOG state send, TODO: implement this instead of a stub/dummy
  LOOP_D("[LOOP] Time logging check");
  if(cfg.do_timelog && (last_time_log == 0 || millis() - last_time_log > TIMELOG_INTERVAL)) {
    #if defined(BLUETOOTH_UART_AT) || defined(SUPPORT_BLE_UART1)
    if(ble_advertising_start != 0)
      ble_send(COMMON::PT("🍓 [%H:%M:%S]:📡 ⟹  🖫&💾\n"));
    #endif
    last_time_log = millis();
  }
  #endif // TIMELOG

  #ifdef SUPPORT_UART1
  do_uart1_read();
  #endif // SUPPORT_UART1

  // only do UDP/TCP/TLS checks if WiFi connected
  #ifdef SUPPORT_WIFI
  if(wifi_connected(false)){
    #ifdef SUPPORT_UDP
    do_udp_check();
    #endif // SUPPORT_UDP

    #ifdef SUPPORT_TCP
    do_tcp_check();
    #endif // SUPPORT_TCP

    #ifdef SUPPORT_TLS
    do_tls_check();
    #endif // SUPPORT_TLS

    #ifdef SUPPORT_TCP_SERVER
    do_tcp_server_check();
    #endif // SUPPORT_TCP_SERVER

    #if defined(SUPPORT_WIFI) && (defined(SUPPORT_TCP) || defined(SUPPORT_TLS))
    do_connections_check();
    #endif // SUPPORT_WIFI && (SUPPORT_TCP || SUPPORT_UDP)

  } else {
    // in theory, it's even pointless to do other things too, as we can't send data
    LOOP_D("[LOOP] WiFi not connected, skipping UDP/TCP/TLS checks");
  }
  #endif // SUPPORT_WIFI

  #ifdef SUPPORT_BLE_UART1
  do_ble_uart1_bridge();
  #endif // SUPPORT_BLE_UART1

  #ifdef SUPPORT_WIFI
  do_wifi_check();
  #endif // SUPPORT_WIFI

  #ifdef SUPPORT_ESP_LOG_INFO
  do_esp_log();
  #endif // SUPPORT_ESP_LOG_INFO

  #ifdef LOGUART
  if(cfg.do_log){
    // log the inbuf to UART0, raw
    if(inlen > 0){
      for(size_t i = 0; i < inlen; i++)
        Serial.write((uint8_t)inbuf[i]);
      Serial.flush();
    }
  }
  #endif

  #if defined(SUPPORT_WIFI) && defined(SUPPORT_NTP)
  LOOP_D("[NTP] NTP time sync check");
  do_ntp_check();
  #endif // SUPPORT_WIFI && SUPPORT_NTP

  // copy over the inbuf to outbuf for logging if data received
  if(outlen > 0) {
    // send outbuf to UART1 if data received, in chunks of X bytes
    #ifdef SUPPORT_UART1
    uint8_t *o = (uint8_t *)&outbuf;
    uint8_t *m = (uint8_t *)&outbuf + outlen;
    size_t w = 0;
    while(o < m && (w = UART1.availableForWrite()) > 0) {
      doYIELD;
      w = min((size_t)w, (size_t)UART1_WRITE_SIZE);
      w = min((size_t)w, (size_t)(m - o));
      w = UART1.write(o, w);
      UART1.flush();
      if(w > 0) {
        #ifdef LED
        last_activity = millis(); // Trigger LED activity for UART1 send
        #endif // LED
        D("[UART1]: Written %d bytes, total: %d, data: >>%s<<", w, outlen, outbuf);
        o += w;
      } else {
        // nothing written, just yield
        D("[UART1]: nothing written, yielding...");
      }
    }
    if(o >= m) {
      // all sent
      D("[UART1]: all outbuf data sent");
    } else {
      D("[UART1]: not all outbuf data sent, remaining: %d", m - o);
    }
    outlen = 0;
    #else
    // no UART1 support, just clear the outbuf
    outlen = 0;
    #endif // SUPPORT_UART1
    doYIELD;
  }

  // clear outbuf
  memset(outbuf, 0, sizeof(outbuf));

  // assume the inbuf is sent
  if(inlen && sent_ok) {
    inlen = 0;
    memset(inbuf, 0, sizeof(inbuf));
  }
  if(inlen) {
    LOOP_D("[LOOP] inbuf not sent, clearing buffer, len: %d", inlen);
    inlen = 0;
    memset(inbuf, 0, sizeof(inbuf));
  }

  // Plugins loop
  #ifdef SUPPORT_PLUGINS
  if(PLUGINS::loop_post){
    LOOP_D("[PLUGINS] Running plugins POST hooks");
    PLUGINS::loop_post();
  }
  #endif // SUPPORT_PLUGINS

  // Handle button press
  determine_button_state();

  #ifdef LED
  // LED indicator
  LOOP_D("[LED] Checking LED state and updating if needed");
  set_led_blink(determine_led_state());
  update_led_state();
  #endif // LED

  #ifdef LOOP_DELAY
  // Do delay at the end of the loop, for sleep/power save, this is "smart",
  // when not possible no sleep is done (connections, BLE, WPS,..)
  #ifdef SUPPORT_UART1
  if(!UART1.available())
    do_loop_delay();
  #else
  do_loop_delay();
  #endif // SUPPORT_UART1

  // Handle button press AFTER
  determine_button_state();
  #endif // LOOP_DELAY

  LOOP_D("[LOOP] End main loop");
}
