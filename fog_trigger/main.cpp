/*
 * fog_trigger - sits 100-200 m upstream of the fog stretch and tells the
 * fog_mister over ESP-NOW that a group is coming. A marshal presses the
 * button, or an optional PIR sensor does it.
 *
 * Button (GPIO3, or the on-board BOOT button): short press = group coming,
 * hold 2 s = undo last group.
 * LED: short blip every 2 s = link OK, blinking twice a second = no link,
 * flicker = sending, on for 1.5 s = mister confirmed, three long
 * flashes = not confirmed.
 *
 * Board: ESP32-S3 or ESP32-C3 SuperMini, Espressif Arduino core 3.3.x, USB CDC On Boot enabled.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* ---- pins ---- */
#define BUTTON_PIN      3   // push button to GND
#if CONFIG_IDF_TARGET_ESP32S3
#define BOOT_BUTTON_PIN 0   // on-board BOOT button: same function once running (bench testing)
#else
#define BOOT_BUTTON_PIN 9   // on-board BOOT button (C3)
#endif
#define PIR_PIN         4   // AM312 output, powered from 3V3
#if CONFIG_IDF_TARGET_ESP32S3
#define LED_PIN         48  // on-board LED (S3 SuperMini: WS2812 RGB, shown blue)
#define LED_RGB         1
#define LED_RGB_LEVEL   40  // 0-255, it is bright
#else
#define LED_PIN         8   // on-board LED (C3 SuperMini: blue, active low)
#define LED_RGB         0
#endif
#define LED_ACTIVE_LOW  1   // plain LED only

/* ---- options ---- */
#define USE_SENSOR      false
#define PIR_LOCKOUT_MS  45000   // one group per PIR trigger
#define ESPNOW_CHANNEL  1       // must match the mister's AP channel
#ifndef TX_POWER
#define TX_POWER        WIFI_POWER_19_5dBm  // SuperMini boards: try WIFI_POWER_8_5dBm
#endif
#define PING_EVERY_MS   2000
#define LINK_TIMEOUT_MS 6500
#define RETRY_MS        250
#define RETRIES         8
#define LONG_PRESS_MS   2000
/* MISTER_MAC: the mister's soft-AP MAC as bytes, set by fog.py from boards.toml
 * (-DMISTER_MAC=0x..,0x..,...). Without it the trigger finds the mister by
 * broadcast and takes the first one that acks. */

/* ---- ESP-NOW protocol, keep in sync with fog_mister ---- */
#define FOG_MAGIC 0x464F4731UL  // "FOG1"
enum : uint8_t { MSG_PING = 1, MSG_GROUP = 2, MSG_UNDO = 3, MSG_ACK = 0x80 };
struct __attribute__((packed)) FogMsg {
    uint32_t magic;
    uint8_t  type;
    uint8_t  seq;
};

static const uint8_t BROADCAST[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
#ifdef MISTER_MAC
static uint8_t mister[6] = {MISTER_MAC};
static bool have_mister = true;
#else
static uint8_t mister[6];
static bool have_mister = false;
#endif

static volatile uint32_t last_ack_ms = 0;
static volatile bool got_ack = false;
static volatile uint8_t ack_type = 0, ack_seq = 0;
static volatile uint8_t ack_mac[6];

static uint8_t seq = 0;      // pings
static uint8_t cmd_seq = 0;  // group/undo; random start so a reboot is unlikely to repeat the last one

/* pending GROUP/UNDO, resent until acked */
static bool sending = false;
static uint8_t send_type = 0, send_seq = 0;
static int send_tries = 0;
static uint32_t send_at = 0;

enum LedMode { LED_LINK, LED_CONFIRMED, LED_FAILED };
static LedMode led_mode = LED_LINK;
static uint32_t led_mode_since = 0;

static inline bool elapsed(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

static void led(bool on) {
#if LED_RGB
    static int last = -1;  // the WS2812 takes an RMT write; only send changes
    if (on == last)
        return;
    last = on;
    rgbLedWrite(LED_PIN, 0, 0, on ? LED_RGB_LEVEL : 0);
#else
    digitalWrite(LED_PIN, (on ^ LED_ACTIVE_LOW) ? HIGH : LOW);
#endif
}

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len != sizeof(FogMsg))
        return;
    FogMsg m;
    memcpy(&m, data, sizeof m);
    if (m.magic != FOG_MAGIC || !(m.type & MSG_ACK))
        return;
#ifdef MISTER_MAC
    if (memcmp(info->src_addr, mister, 6) != 0)
        return;
#endif
    for (int i = 0; i < 6; i++)
        ack_mac[i] = info->src_addr[i];
    ack_type = m.type & ~MSG_ACK;
    ack_seq = m.seq;
    last_ack_ms = millis();
    got_ack = true;
}

static void add_peer(const uint8_t *mac) {
    if (esp_now_is_peer_exist(mac))
        return;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
}

static void send_msg(uint8_t type, uint8_t s) {
    FogMsg m = {FOG_MAGIC, type, s};
    // unicast once the mister is known: the radio then retries by itself
    esp_now_send(have_mister ? mister : BROADCAST, (const uint8_t *)&m, sizeof m);
}

static void queue_send(uint8_t type, uint32_t now) {
    sending = true;
    send_type = type;
    send_seq = ++cmd_seq;
    send_tries = 0;
    send_at = now;
    Serial.printf("sending %s\n", type == MSG_GROUP ? "group" : "undo");
}

static void handle_acks(uint32_t now) {
    if (!got_ack)
        return;
    got_ack = false;
    if (!have_mister) {
        memcpy(mister, (const void *)ack_mac, 6);
        add_peer(mister);
        have_mister = true;
        Serial.printf("mister %02x:%02x:%02x:%02x:%02x:%02x\n", mister[0], mister[1],
                      mister[2], mister[3], mister[4], mister[5]);
    }
    if (sending && ack_type == send_type && ack_seq == send_seq) {
        sending = false;
        led_mode = LED_CONFIRMED;
        led_mode_since = now;
        Serial.println("confirmed");
    }
}

static void run_send(uint32_t now) {
    static uint32_t next_ping = 0;
    if (sending) {
        if (!elapsed(now, send_at))
            return;
        if (send_tries >= RETRIES) {
            sending = false;
            led_mode = LED_FAILED;
            led_mode_since = now;
            Serial.println("not confirmed");
            return;
        }
        send_tries++;
        send_msg(send_type, send_seq);
        send_at = now + RETRY_MS;
    } else if (elapsed(now, next_ping)) {
        send_msg(MSG_PING, ++seq);
        next_ping = now + PING_EVERY_MS;
    }
}

static void run_led(uint32_t now) {
    uint32_t t = now - led_mode_since;
    if (led_mode == LED_CONFIRMED) {
        if (t < 1500)
            return led(true);
        led_mode = LED_LINK;
    } else if (led_mode == LED_FAILED) {
        if (t < 3 * 800)
            return led(t % 800 < 500);
        led_mode = LED_LINK;
    }
    if (sending)
        led((now / 40) % 2);
    else if (have_mister && now - last_ack_ms < LINK_TIMEOUT_MS)
        led(now % 2000 < 40);
    else
        led(now % 500 < 250);
}

struct Button {
    uint8_t pin;
    bool down = false, long_fired = false;
    uint32_t since = 0, changed = 0;
    // 0 = nothing, 1 = short press (on release), 2 = long press
    int poll(uint32_t now) {
        bool raw = digitalRead(pin) == LOW;
        if (raw != down && now - changed >= 30) {
            changed = now;
            down = raw;
            if (down) {
                since = now;
                long_fired = false;
            } else if (!long_fired) {
                return 1;
            }
        }
        if (down && !long_fired && now - since >= LONG_PRESS_MS) {
            long_fired = true;
            return 2;
        }
        return 0;
    }
};
static Button button{BUTTON_PIN};
static Button boot_button{BOOT_BUTTON_PIN};

void setup() {
    if (!LED_RGB)
        pinMode(LED_PIN, OUTPUT);
    led(false);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    if (USE_SENSOR)
        pinMode(PIR_PIN, INPUT);

    Serial.begin(115200);
    Serial.println("fog_trigger starting");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    WiFi.setTxPower(TX_POWER);

    if (esp_now_init() != ESP_OK)
        Serial.println("ESP-NOW init failed");
    esp_now_register_recv_cb(on_recv);
    add_peer(have_mister ? mister : BROADCAST);
    Serial.printf("mister %s\n", have_mister ? "from build" : "by broadcast");
    cmd_seq = esp_random();
}

void loop() {
    uint32_t now = millis();

    for (Button *b : {&button, &boot_button}) {
        switch (b->poll(now)) {
        case 1: queue_send(MSG_GROUP, now); break;
        case 2: queue_send(MSG_UNDO, now); break;
        }
    }

    if (USE_SENSOR) {
        static bool pir_last = false;
        static uint32_t pir_at = 0;
        static bool pir_fired = false;
        bool pir = digitalRead(PIR_PIN) == HIGH;
        if (pir && !pir_last && (!pir_fired || now - pir_at >= PIR_LOCKOUT_MS)) {
            pir_fired = true;
            pir_at = now;
            Serial.println("PIR");
            queue_send(MSG_GROUP, now);
        }
        pir_last = pir;
    }

    handle_acks(now);
    run_send(now);
    run_led(now);
    delay(5);
}
