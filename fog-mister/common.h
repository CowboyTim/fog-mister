/*
 * common.h
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

// Logging setup for esp32c3

#ifndef _COMMON_H
#define _COMMON_H

// DEBUG means all debug messages, so enable VERBOSE as well
#ifdef DEBUG
#warning "DEBUG logging enabled, enabling VERBOSE as well"
#define VERBOSE
#endif

#ifdef VERBOSE
#warning "VERBOSE logging enabled"
#endif

#ifndef UART_LOG_DEV_UART0
#define UART_LOG_DEV_UART0 0
#endif

#if UART_LOG_DEV_UART0 == 1
 #define NO_GLOBAL_INSTANCES
 #define NO_GLOBAL_SERIAL
 #include <HardwareSerial.h>
 extern HardwareSerial Serial0;
 extern HardwareSerial Serial1;
 #define UART0         Serial0
 #define USBSERIAL0    Serial0
 #define Serial        Serial0
 #define UART1         Serial1
#else
 #include <HardwareSerial.h>
 #define UART0         Serial
 #define USBSERIAL0    Serial
 #define UART1         Serial1
#endif // UART_LOG_DEV_UART0

#include <stdint.h>
#include <errno.h>
#include <sys/time.h>
#include "time.h"
#include <nvs_flash.h>
#include <nvs.h>

/*
 * Function attributes, for common functions
 */
#define NOINLINE __attribute__((noinline,noipa))
#define INLINE   __attribute__((always_inline))
#define ALIGN(x) __attribute__((aligned(x)))

/*
 * LED and BUTTON definitions
 */
#ifndef LED
#define LED    GPIO_NUM_8
#endif
#ifndef SUPPORT_LED_BRIGHTNESS
#define SUPPORT_LED_BRIGHTNESS
#endif

#ifndef BUTTON
#ifndef BUTTON_BUILTIN
#define BUTTON_BUILTIN  GPIO_NUM_9
#endif
#define BUTTON GPIO_NUM_3
#endif

/*
 * AT command response helpers
 */
#define AT_R_OK         (const char*)("OK")
#define AT_R(M)         (const char*)(M)
#define AT_R_STR(M)     (const char*)(M)
#define AT_R_S(M)       (const char*)(M).c_str()
#define AT_R_INT(M)     (const char*)(COMMON::_r_int(M))
#define AT_R_DOUBLE(M)  (const char*)(COMMON::_r_double(M))
#define AT_R_F(M)       (const char*)(M)

/* ESP yield, only needed on 1 core ESP (like ESP8266). Multi core ESP32
 * usually runs on CPU1 main arduino sketch and CPU0 for WiFi
 * so yield is not needed there.
 *
 * The esp32c3 is however a single core esp32
 *
 */
#define doYIELD yield();

#ifdef VERBOSE
 #ifdef DEBUG
  #define __FILE__            "esp-at.ino"
  #define LOG_TIME_FORMAT     "[\%H:\%M:\%S][info]: "
  #define DEBUG_TIME_FORMAT   "[\%H:\%M:\%S][debug]: "
  #define DEBUG_FILE_LINE     "[\%hu:\%s:\%d]", millis(), __FILE__, __LINE__
 #else
  #define LOG_TIME_FORMAT     "[\%H:\%M:\%S][info]: "
 #endif

 #define LOG(...)     COMMON::_log_l(__VA_ARGS__);
 #define LOGT(...)    COMMON::_log_t(__VA_ARGS__);
 #define LOGR(...)    COMMON::_log_r(__VA_ARGS__);
 #define LOGE(...)    COMMON::_log_e(__VA_ARGS__);
 #define LOGFLUSH()   COMMON::_log_flush();
 #define LOGSETUP()   COMMON::_log_setup();
 #define DO_VERBOSE(code)  if(COMMON::_do_verbose){code;};
#else
 #define LOG(...)     {}
 #define LOGT(...)    {}
 #define LOGR(...)    {}
 #define LOGE(...)    {}
 #define LOGFLUSH()   {}
 #define LOGSETUP()   {}
 #define DO_VERBOSE(code) {}
 #define D(...)       {}
 #define T(...)       {}
 #define R(...)       {}
 #define E(...)       {}
#endif // VERBOSE

#ifdef DEBUG
 #define D(...)       COMMON::_debug_l(__VA_ARGS__);
 #define T(...)       COMMON::_debug_t(__VA_ARGS__);
 #define R(...)       COMMON::_debug_r(__VA_ARGS__);
 #define E(...)       COMMON::_debug_e(__VA_ARGS__);
#else
 #define D(...)       {}
 #define T(...)       {}
 #define R(...)       {}
 #define E(...)       {}
#endif // DEBUG

namespace COMMON {

NOINLINE
extern char* _r_int(int val);

NOINLINE
extern char* _r_double(double val);

NOINLINE
extern const char* PT(const char* tformat = "[%H:%M:%S]");

// Helper function to get human-readable errno messages
NOINLINE
const char* get_errno_string(int err);

/*
 * VERBOSE logging functions
 */
#ifdef VERBOSE
// flag, VERBOSE on/off
extern uint8_t _do_verbose;

NOINLINE
void do_vprintf(uint8_t t, const char *tf, const char *_fmt, va_list args);

NOINLINE
void do_printf(uint8_t t, const char *tf, const char *_fmt, ...);

NOINLINE
void _log_flush();

NOINLINE
void _log_setup();

NOINLINE
void _log_l(const char *fmt, ...);

NOINLINE
void _log_t(const char *fmt, ...);

NOINLINE
void _log_r(const char *fmt, ...);

NOINLINE
void _log_e(const char *fmt, ...);
#endif // VERBOSE

#ifdef DEBUG
NOINLINE
void _debug_l(const char *fmt, ...);

NOINLINE
void _debug_t(const char *fmt, ...);

NOINLINE
void _debug_r(const char *fmt, ...);

NOINLINE
void _debug_e(const char *fmt, ...);
#endif // DEBUG

/*
 * AT command check helper
 * Returns pointer to the parameter part of the AT command
 * or NULL if not matching
 */
NOINLINE
extern char* at_cmd_check(const char* cmd, const char* at_cmd, unsigned short at_len);

} // namespace COMMON

namespace CFG {

extern NOINLINE bool INIT(const char* pa, const char* ns);
extern NOINLINE void SAVE(const char* pa, const char* ns, const char* st, void* cfg, size_t rs);
extern NOINLINE void CLEAR(const char* pa, const char* ns, const char* st);
extern NOINLINE void LOAD(const char* pa, const char* ns, const char* st, void* cfg, size_t rs);

extern NOINLINE bool INIT_H(nvs_handle_t *, const char* pa, const char* ns);
extern NOINLINE void SAVE_H(nvs_handle_t *, const char* pa, const char* ns, const char* st, void* cfg, size_t rs);
extern NOINLINE void CLEAR_H(nvs_handle_t *, const char* pa, const char* ns, const char* st);
extern NOINLINE void LOAD_H(nvs_handle_t *, const char* pa, const char* ns, const char* st, void* cfg, size_t rs);


} // namespace CFG

#endif // _COMMON_H
