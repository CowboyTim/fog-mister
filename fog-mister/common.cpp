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

#include <stdlib.h>
#include "common.h"

namespace COMMON {

NOINLINE
const char * PT(const char *tformat) {
  ALIGN(4) static char T_buffer[512] = {""};
  static time_t t = 0;
  static struct tm gm_new_tm = {0};
  time(&t);
  if(localtime_r(&t, &gm_new_tm) == NULL) {
    T_buffer[0] = 0;
    return (const char *)&T_buffer;
  }
  memset(T_buffer, 0, sizeof(T_buffer));
  size_t s = strftime(T_buffer, sizeof(T_buffer), tformat, &gm_new_tm);
  if(s == 0)
    T_buffer[0] = 0;
  return (const char *)&T_buffer;
}


NOINLINE
char* at_cmd_check(const char *cmd, const char *at_cmd, unsigned short at_len) {
  unsigned short l = strlen(cmd); /* AT+<cmd>=, or AT, or AT+<cmd>? */
  if(at_len >= l && strncmp(cmd, at_cmd, l) == 0) {
    if(*(cmd+l-1) == '=') {
      return (char *)at_cmd+l;
    } else {
      return (char *)at_cmd;
    }
  }
  return NULL;
}

NOINLINE
char* _r_int(int val) {
  ALIGN(4) static char _obuf[12] = {0};
  // convert integer to string in _obuf
  memset(_obuf, 0, sizeof(_obuf));
  int written = snprintf(_obuf, sizeof(_obuf), "%d", val);
  if(written >= sizeof(_obuf))
    _obuf[sizeof(_obuf) - 1] = 0;
  return _obuf;
}

NOINLINE
char* _r_double(double val) {
  ALIGN(4) static char _obuf[24] = {0};
  // convert double to string in _obuf
  memset(_obuf, 0, sizeof(_obuf));
  int written = snprintf(_obuf, sizeof(_obuf), "%5.5f", val);
  if(written >= sizeof(_obuf))
    _obuf[sizeof(_obuf) - 1] = 0;
  return _obuf;
}

NOINLINE
const char* get_errno_string(int err) {
  switch(err) {
    case EACCES: return "Permission denied";
    case EADDRINUSE: return "Address already in use";
    case EADDRNOTAVAIL: return "Address not available";
    case EAFNOSUPPORT: return "Address family not supported";
    case EAGAIN: return "Resource temporarily unavailable";
    case EALREADY: return "Operation already in progress";
    case EBADF: return "Bad file descriptor";
    case ECONNABORTED: return "Connection aborted";
    case ECONNREFUSED: return "Connection refused";
    case ECONNRESET: return "Connection reset";
    case EFAULT: return "Bad address";
    case EHOSTDOWN: return "Host is down";
    case EHOSTUNREACH: return "Host unreachable";
    case EINPROGRESS: return "Operation in progress";
    case EINTR: return "Interrupted system call";
    case EINVAL: return "Invalid argument";
    case EIO: return "I/O error";
    case EISCONN: return "Already connected";
    case EMFILE: return "Too many open files";
    case EMSGSIZE: return "Message too long";
    case ENETDOWN: return "Network is down";
    case ENETUNREACH: return "Network unreachable";
    case ENOBUFS: return "No buffer space available";
    case ENOMEM: return "Out of memory";
    case ENOTCONN: return "Not connected";
    case ENOTSOCK: return "Not a socket";
    case EPIPE: return "Broken pipe";
    case EPROTONOSUPPORT: return "Protocol not supported";
    case EPROTOTYPE: return "Protocol wrong type for socket";
    case ETIMEDOUT: return "Connection timed out";
    case ENFILE: return "Too many open files in system";
    default: return "Unknown error";
  }
}

#ifdef VERBOSE

NOINLINE
void LOGPRINT(const char *buf){
    UART0.write(buf, strlen(buf));
}

NOINLINE
void do_vprintf(uint8_t t, const char *tf, const char *_fmt, va_list args) {
  ALIGN(4) static char obuf[256] = {0};
  if(_fmt == NULL && tf == NULL)
    return;
  if((t & 2) && tf != NULL)
    LOGPRINT(PT(tf));
  if(_fmt == NULL)
    return;
  static int s = 0;
  s = vsnprintf(obuf, sizeof(obuf), _fmt, args);
  if(s < 0)
    obuf[0] = 0;
  else if(s >= sizeof(obuf))
    obuf[sizeof(obuf) - 1] = 0;
  else
    obuf[s] = 0;

  if(t & 1) {
    LOGPRINT(obuf);
    LOGPRINT("\n");
  } else {
    LOGPRINT(obuf);
  }
}

NOINLINE
void do_printf(uint8_t t, const char *tf, const char *_fmt, ...) {
  va_list args;
  va_start(args, _fmt);
  do_vprintf(t, tf, _fmt, args);
  va_end(args);
}

NOINLINE
void _log_flush() {
    UART0.flush();
}

NOINLINE
void _log_setup() {
    UART0.begin(115200);
    delay(100);
    UART0.setTimeout(0);
    UART0.setTxBufferSize(512);
    UART0.setRxBufferSize(512);
    UART0.println();
}

uint8_t _do_verbose = 1;

NOINLINE
void _log_l(const char *fmt, ...) {
    if(_do_verbose) {
        #ifdef DEBUG
        do_printf(0, NULL, DEBUG_FILE_LINE);
        #endif
        va_list args;
        va_start(args, fmt);
        do_vprintf(3, LOG_TIME_FORMAT, fmt, args);
        va_end(args);
    }
}

NOINLINE
void _log_t(const char *fmt, ...) {
    if(_do_verbose) {
        #ifdef DEBUG
        do_printf(0, NULL, DEBUG_FILE_LINE);
        #endif
        va_list args;
        va_start(args, fmt);
        do_vprintf(2, LOG_TIME_FORMAT, fmt, args);
        va_end(args);
    }
}

NOINLINE
void _log_r(const char *fmt, ...) {
    if(_do_verbose) {
        va_list args;
        va_start(args, fmt);
        do_vprintf(0, NULL, fmt, args);
        va_end(args);
    }
}

NOINLINE
void _log_e(const char *fmt, ...) {
    if(_do_verbose) {
        #ifdef DEBUG
        do_printf(0, NULL, DEBUG_FILE_LINE);
        #endif
        va_list args;
        va_start(args, fmt);
        do_vprintf(2, LOG_TIME_FORMAT, fmt, args);
        va_end(args);
        _log_r(", errno: %d (%s)\n", errno, get_errno_string(errno));
    }
}
#endif

#ifdef DEBUG
NOINLINE
void _debug_l(const char *fmt, ...) {
    do_printf(0, NULL, DEBUG_FILE_LINE);
    va_list args;
    va_start(args, fmt);
    do_vprintf(3, DEBUG_TIME_FORMAT, fmt, args);
    va_end(args);
}

NOINLINE
void _debug_t(const char *fmt, ...) {
    do_printf(0, NULL, DEBUG_FILE_LINE);
    va_list args;
    va_start(args, fmt);
    do_vprintf(2, DEBUG_TIME_FORMAT, fmt, args);
    va_end(args);
}

NOINLINE
void _debug_r(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    do_vprintf(0, NULL, fmt, args);
    va_end(args);
}

NOINLINE
void _debug_e(const char *fmt, ...) {
    do_printf(0, NULL, DEBUG_FILE_LINE);
    va_list args;
    va_start(args, fmt);
    do_vprintf(2, DEBUG_TIME_FORMAT, fmt, args);
    va_end(args);
    _debug_r(", errno: %d (%s)\n", errno, get_errno_string(errno));
}
#endif // DEBUG

} // namespace COMMON

// define a WEAK function for WDT handler to override default behavior
void __attribute__((weak)) WDT(void) {
    // Default empty implementation
}
extern "C" {
  void esp_task_wdt_isr_user_handler(void) {
      WDT();
  }
}


namespace CFG {

nvs_handle_t nvs_handle;

NOINLINE
bool INIT_H(nvs_handle_t * nvs_c, const char *pa, const char *ns) {
  LOG("[NVS] Setting up NVS in partition '%s', namespace '%s'", pa, ns);
  esp_err_t err = nvs_open_from_partition(pa, ns, NVS_READWRITE, nvs_c);
  if (err != ESP_OK) {
    LOG("[NVS] Failed to open NVS partition %s: %s", pa, esp_err_to_name(err));
    LOG("[NVS] Attempting to initialize NVS partition %s", pa);
    err = nvs_flash_init_partition(pa);
    if(err != ESP_OK) {
      LOG("[NVS] Failed to initialize NVS partition %s: %s", pa, esp_err_to_name(err));
      return false;
    } else {
      LOG("[NVS] initialized successfully");
      err = nvs_open_from_partition(pa, ns, NVS_READWRITE, nvs_c);
      if (err) {
        LOG("[NVS] Failed to open NVS partition %s after init: %s", pa, esp_err_to_name(err));
        return false;
      } else {
        LOG("[NVS] opened successfully after init");
      }
    }
  } else {
    LOG("[NVS] NVS partition '%s' opened successfully", pa);
  }
  return true;
}

NOINLINE
void SAVE_H(nvs_handle_t * nvs_c, const char *pa, const char *ns, const char *st, void *cfg, size_t rs) {
  LOG("[NVS] Saving config to NVS, partition: '%s', namespace: '%s', key: '%s', size: %d bytes", pa, ns,  st, rs);
  esp_err_t err;
  err = nvs_open_from_partition(pa, ns, NVS_READWRITE, nvs_c);
  if (err != ESP_OK) {
    LOG("[NVS] Error (%d) opening NVS handle! %s", err, esp_err_to_name(err));
    return;
  }
  err = nvs_set_blob(*nvs_c, st, cfg, rs);
  if (err != ESP_OK) {
    LOG("[NVS] Error (%d) setting blob in NVS! %s", err, esp_err_to_name(err));
  }
  err = nvs_commit(*nvs_c);
  if (err != ESP_OK) {
    LOG("[NVS] Error (%d) committing blob to NVS! %s", err, esp_err_to_name(err));
  }
  LOG("[NVS] Config saved to NVS, partition: '%s', namespace: '%s', key: '%s'", pa, ns, st);
}

NOINLINE
void CLEAR_H(nvs_handle_t *nvs_c, const char *pa, const char *ns, const char *st) {
  LOG("[NVS] Clearing config from NVS for partition %s, %s, %s", pa, ns, st);

  esp_err_t err;

  // Close NVS handle if open
  if(*nvs_c != 0) {
    nvs_close(*nvs_c);
    *nvs_c = NULL;
  }

  // Erase specific key if provided
  if(ns != NULL && st != NULL) {
    LOG("[NVS] Erasing key '%s' from NVS", st);
    err = nvs_open_from_partition(pa, ns, NVS_READWRITE, nvs_c);
    if (err == ESP_OK) {
      err = nvs_erase_key(*nvs_c, st);
      if (err != ESP_OK) {
        LOG("[NVS] Error (%d) erasing key %s in NVS! %s", err, st, esp_err_to_name(err));
      } else {
        LOG("[NVS] Key '%s' erased from NVS", st);
      }
      nvs_close(*nvs_c);
      *nvs_c = NULL;
    }
  }

  // Erase the entire partition
  LOG("[NVS] Erasing entire NVS partition %s", pa);
  err = nvs_flash_erase_partition(pa);
  if (err != ESP_OK) {
    LOG("[NVS] Error (%d) erasing NVS! %s", err, esp_err_to_name(err));
  }

  // Erase the main "nvs" partition as well, to clear any other data
  LOG("[NVS] Erasing main NVS partition");
  err = nvs_flash_erase_partition("nvs");
  if (err != ESP_OK) {
    LOG("[NVS] Error (%d) erasing main NVS! %s", err, esp_err_to_name(err));
  }

  LOG("[NVS] NVS cleared");
}

NOINLINE
void LOAD_H(nvs_handle_t * nvs_c, const char *pa, const char *ns, const char *st, void *cfg, size_t rs) {
  LOG("[NVS] Loading config from NVS partition '%s', namespace '%s', key '%s'", pa, ns, st);
  esp_err_t err;
  err = nvs_open_from_partition(pa, ns, NVS_READONLY, nvs_c);
  if (err != ESP_OK) {
    LOG("[NVS] Error (%d) opening NVS handle! %s", err, esp_err_to_name(err));
    return;
  }
  err = nvs_get_blob(*nvs_c, st, cfg, &rs);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    LOG("[NVS] No config found in NVS, using defaults");
    return;
  } else if (err != ESP_OK) {
    LOG("[NVS] Error (%d) reading config from NVS! %s", err, esp_err_to_name(err));
    return;
  }
  LOG("[NVS] Config loaded from NVS, size: %d bytes", rs);
}

NOINLINE
bool INIT(const char *pa, const char *ns) {
  return INIT_H(&nvs_handle, pa, ns);
}

NOINLINE
void SAVE(const char *pa, const char *ns, const char *st, void *cfg, size_t rs) {
  SAVE_H(&nvs_handle, pa, ns, st, cfg, rs);
  return;
}

NOINLINE
void LOAD(const char *pa, const char *ns, const char *st, void *cfg, size_t rs) {
  LOAD_H(&nvs_handle, pa, ns, st, cfg, rs);
  return;
}

NOINLINE
void CLEAR(const char *pa, const char *ns, const char *st) {
  CLEAR_H(&nvs_handle, pa, ns, st);
  return;
}

} // namespace CFG
