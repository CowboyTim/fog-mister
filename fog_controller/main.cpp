/*
 * fog_controller - runs the fogbox: fog machine over DMX, ultrasonic mister
 * via an SSR, and the blower via PWM. Groups are announced by the trigger
 * node over ESP-NOW (or the button / web page); each one gets a timed
 * build-up, hold, tail and purge cycle timed to its arrival.
 *
 * Board: ESP32-C3, Espressif Arduino core 3.3.x, USB CDC On Boot enabled.
 *
 * Web UI: join Wi-Fi "fogbox" and open http://192.168.4.1
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* ---- pins ---- */
#define DMX_TX_PIN      3   // MAX3485 DI
#define BLOWER_PIN      4   // MOSFET module #1 TRIG/PWM
#define MISTER_PIN      5   // MOSFET module #2 TRIG -> SSR input
#define STOP_PIN        7   // toggle to GND, closed = everything off
#define LED_PIN         8   // on-board LED (SuperMini: blue, active low)
#define BUTTON_PIN      10  // push button to GND
#define LED_ACTIVE_LOW  1

/* ---- radio ---- */
#define AP_SSID         "fogbox"
#define AP_PASS         "boo-fog-2026"
#define ESPNOW_CHANNEL  1
#define TX_POWER        WIFI_POWER_19_5dBm  // SuperMini boards: try WIFI_POWER_8_5dBm

/* ---- outputs ---- */
#define DMX_SLOTS       32      // BLAZE uses 8; a short frame is valid DMX
#define DMX_CH_FOG      1
#define FOG_LEVEL       255
#define BLOWER_PWM_HZ   20000
#define BLOWER_PWM_BITS 10
#define BLOWER_RAMP_PCT_PER_S 40.0f

/* ---- cycle timing and safety ---- */
#define WARMUP_MS         (4UL * 60 * 1000)  // no fog after power-on
#define FIRST_BURST_MS    8000
#define BURST_MS          5000
#define BUILD_EVERY_MS    20000
#define HOLD_EVERY_MS     30000
#define MAX_BURST_MS      15000
#define MIN_GAP_MS        10000
#define MANUAL_FOG_MS     5000
#define MANUAL_MIST_MS    15000
#define BLOWER_TEST_MS    10000
#define FALLBACK_MAX      3
#define MAX_GROUPS        8
#define LONG_PRESS_MS     2000

/* ---- ESP-NOW protocol, keep in sync with fog_trigger ---- */
#define FOG_MAGIC 0x464F4731UL  // "FOG1"
enum : uint8_t { MSG_PING = 1, MSG_GROUP = 2, MSG_UNDO = 3, MSG_ACK = 0x80 };
struct __attribute__((packed)) FogMsg {
    uint32_t magic;
    uint8_t  type;
    uint8_t  seq;
};

/* ---- settings, stored in NVS ---- */
struct Settings {
    uint16_t walk_s     = 120;
    uint16_t fallback_s = 390;  // 0 = off
    uint16_t build_s    = 90;
    uint16_t hold_s     = 90;
    uint16_t tail_s     = 30;
    uint16_t purge_s    = 60;
    uint8_t  blow_build = 45;
    uint8_t  blow_hold  = 35;
    uint8_t  blow_tail  = 30;
    uint8_t  blow_purge = 15;
    uint8_t  fog_enabled = 1;
} cfg;

Preferences prefs;

static void load_settings() {
    prefs.begin("fogbox", true);
    cfg.walk_s      = prefs.getUShort("walk", cfg.walk_s);
    cfg.fallback_s  = prefs.getUShort("fallback", cfg.fallback_s);
    cfg.build_s     = prefs.getUShort("build", cfg.build_s);
    cfg.hold_s      = prefs.getUShort("hold", cfg.hold_s);
    cfg.tail_s      = prefs.getUShort("tail", cfg.tail_s);
    cfg.purge_s     = prefs.getUShort("purge", cfg.purge_s);
    cfg.blow_build  = prefs.getUChar("b_build", cfg.blow_build);
    cfg.blow_hold   = prefs.getUChar("b_hold", cfg.blow_hold);
    cfg.blow_tail   = prefs.getUChar("b_tail", cfg.blow_tail);
    cfg.blow_purge  = prefs.getUChar("b_purge", cfg.blow_purge);
    cfg.fog_enabled = prefs.getUChar("fog_en", cfg.fog_enabled);
    prefs.end();
}

static void save_settings() {
    prefs.begin("fogbox", false);
    prefs.putUShort("walk", cfg.walk_s);
    prefs.putUShort("fallback", cfg.fallback_s);
    prefs.putUShort("build", cfg.build_s);
    prefs.putUShort("hold", cfg.hold_s);
    prefs.putUShort("tail", cfg.tail_s);
    prefs.putUShort("purge", cfg.purge_s);
    prefs.putUChar("b_build", cfg.blow_build);
    prefs.putUChar("b_hold", cfg.blow_hold);
    prefs.putUChar("b_tail", cfg.blow_tail);
    prefs.putUChar("b_purge", cfg.blow_purge);
    prefs.putUChar("fog_en", cfg.fog_enabled);
    prefs.end();
}

/* ---- state ---- */
enum Phase : uint8_t { PH_IDLE, PH_WAIT, PH_PURGE, PH_TAIL, PH_HOLD, PH_BUILD };
static const char *phase_name[] = {"idle", "waiting", "purge", "tail", "hold", "build-up"};

struct Group {
    uint32_t trigger_ms;
    bool     fallback;
};
static Group groups[MAX_GROUPS];
static int n_groups = 0;

static Phase phase = PH_IDLE;
static bool soft_stop = false;
static bool hw_stop = false;

static bool fog_on = false;
static uint32_t fog_off_at = 0;
static uint32_t last_burst_end = 0;
static uint32_t next_burst_at = 0;
static bool first_burst = false;
static bool burst_manual = false;
static uint32_t manual_mist_until = 0;
static uint32_t blower_test_until = 0;
static uint8_t blower_test_pct = 0;

static bool mister_on = false;
static float blower_pct = 0;

static bool fallback_armed = false;
static uint8_t fallback_count = 0;
static uint32_t last_trigger_ms = 0;

static uint32_t fog_ms_total = 0, mist_ms_total = 0, blow_ms_total = 0;

static volatile int trig_rssi = 0;
static volatile uint32_t trig_heard_ms = 0;
static volatile bool trig_heard = false;

static volatile uint8_t dmx[DMX_SLOTS + 1];  // [0] = start code 0

static inline bool elapsed(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

/* ---- DMX: own task, the break is sent as a 0x00 at a low baud rate ---- */
static void dmx_task(void *) {
    uint8_t frame[DMX_SLOTS + 1];
    for (;;) {
        for (int i = 0; i <= DMX_SLOTS; i++)
            frame[i] = dmx[i];
        Serial1.flush();
        Serial1.updateBaudRate(90000);  // 9 low bits = 100 us break, stop bits = MAB
        Serial1.write((uint8_t)0);
        Serial1.flush();
        Serial1.updateBaudRate(250000);
        Serial1.write(frame, sizeof frame);
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

/* ---- groups ---- */
static void add_group(uint32_t now, bool fallback) {
    if (n_groups == MAX_GROUPS) {
        memmove(&groups[0], &groups[1], sizeof(Group) * (MAX_GROUPS - 1));
        n_groups--;
    }
    groups[n_groups++] = {now, fallback};
    last_trigger_ms = now;
    if (fallback) {
        if (++fallback_count >= FALLBACK_MAX) {
            fallback_armed = false;
            Serial.println("fallback: limit reached, waiting for a real trigger");
        }
    } else {
        fallback_armed = true;
        fallback_count = 0;
    }
    Serial.printf("group queued (%s), %d in queue\n", fallback ? "fallback" : "trigger", n_groups);
}

static void undo_group() {
    if (n_groups == 0)
        return;
    n_groups--;
    Serial.printf("last group undone, %d in queue\n", n_groups);
}

static Phase group_phase(const Group &g, uint32_t now) {
    int32_t rel = (int32_t)(now - (g.trigger_ms + cfg.walk_s * 1000UL));
    int32_t t;
    if (rel < -(int32_t)(cfg.build_s * 1000UL))
        return PH_WAIT;
    if (rel < 0)
        return PH_BUILD;
    if (rel < (t = cfg.hold_s * 1000L))
        return PH_HOLD;
    if (rel < (t += cfg.tail_s * 1000L))
        return PH_TAIL;
    if (rel < (t += cfg.purge_s * 1000L))
        return PH_PURGE;
    return PH_IDLE;
}

static Phase update_groups(uint32_t now) {
    Phase best = PH_IDLE;
    int j = 0;
    for (int i = 0; i < n_groups; i++) {
        Phase p = group_phase(groups[i], now);
        if (p == PH_IDLE)
            continue;  // done, drop it
        groups[j++] = groups[i];
        if (p > best)
            best = p;
    }
    n_groups = j;
    return best;
}

/* ---- fog machine ---- */
static bool fog_allowed(uint32_t now) {
    return cfg.fog_enabled && !hw_stop && !soft_stop && now >= WARMUP_MS;
}

static bool start_burst(uint32_t now, uint32_t len, bool manual) {
    if (fog_on || !fog_allowed(now))
        return false;
    if (last_burst_end && now - last_burst_end < MIN_GAP_MS)
        return false;
    if (len > MAX_BURST_MS)
        len = MAX_BURST_MS;
    fog_on = true;
    burst_manual = manual;
    fog_off_at = now + len;
    Serial.printf("fog burst %lu ms%s\n", (unsigned long)len, manual ? " (manual)" : "");
    return true;
}

static void stop_burst(uint32_t now) {
    if (!fog_on)
        return;
    fog_on = false;
    last_burst_end = now;
}

static void run_fog(uint32_t now, Phase prev) {
    if (fog_on && (elapsed(now, fog_off_at) || !fog_allowed(now)))
        stop_burst(now);
    // tail and later: the fog machine is off, a running cycle burst is cut
    if (fog_on && !burst_manual && phase != PH_BUILD && phase != PH_HOLD)
        stop_burst(now);

    if (phase == PH_BUILD && prev != PH_BUILD && prev != PH_HOLD) {
        next_burst_at = now;
        first_burst = true;
    }
    if ((phase == PH_BUILD || phase == PH_HOLD) && elapsed(now, next_burst_at)) {
        uint32_t len = first_burst ? FIRST_BURST_MS : BURST_MS;
        if (start_burst(now, len, false)) {
            first_burst = false;
            next_burst_at = now + (phase == PH_BUILD ? BUILD_EVERY_MS : HOLD_EVERY_MS);
        }
    }
    dmx[DMX_CH_FOG] = fog_on ? FOG_LEVEL : 0;
}

/* ---- blower and mister ---- */
static uint8_t phase_blower(Phase p) {
    switch (p) {
    case PH_BUILD: return cfg.blow_build;
    case PH_HOLD:  return cfg.blow_hold;
    case PH_TAIL:  return cfg.blow_tail;
    case PH_PURGE: return cfg.blow_purge;
    default:       return 0;
    }
}

static void run_outputs(uint32_t now, uint32_t dt) {
    bool stopped = hw_stop || soft_stop;

    bool mist = phase == PH_BUILD || phase == PH_HOLD || phase == PH_TAIL;
    if (!elapsed(now, manual_mist_until))
        mist = true;
    mister_on = mist && !stopped;
    digitalWrite(MISTER_PIN, mister_on ? HIGH : LOW);

    uint8_t target = phase_blower(phase);
    if (burst_manual && fog_on && target < cfg.blow_build)
        target = cfg.blow_build;  // push a test burst through the pipe
    if (!elapsed(now, blower_test_until))
        target = blower_test_pct;
    if (stopped)
        target = 0;
    if (target > 100)
        target = 100;

    if (stopped || target <= blower_pct) {
        blower_pct = target;
    } else {
        blower_pct += BLOWER_RAMP_PCT_PER_S * dt / 1000.0f;
        if (blower_pct > target)
            blower_pct = target;
    }
    uint32_t max_duty = (1UL << BLOWER_PWM_BITS) - 1;
    ledcWrite(BLOWER_PIN, (uint32_t)(blower_pct * max_duty / 100.0f));

    if (fog_on)
        fog_ms_total += dt;
    if (mister_on)
        mist_ms_total += dt;
    if (blower_pct > 0)
        blow_ms_total += dt;
}

static void stop_all(uint32_t now) {
    stop_burst(now);
    dmx[DMX_CH_FOG] = 0;
    manual_mist_until = now;
    blower_test_until = now;
    mister_on = false;
    digitalWrite(MISTER_PIN, LOW);
    blower_pct = 0;
    ledcWrite(BLOWER_PIN, 0);
}

/* ---- button and LED ---- */
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

static void led(bool on) { digitalWrite(LED_PIN, (on ^ LED_ACTIVE_LOW) ? HIGH : LOW); }

static void run_led(uint32_t now) {
    if (hw_stop || soft_stop)
        led(true);
    else if (phase >= PH_PURGE)
        led((now / 100) % 2);
    else
        led(now % 2000 < 40);
}

/* ---- ESP-NOW ---- */
struct RxEvent {
    uint8_t mac[6];
    FogMsg msg;
};
static QueueHandle_t rx_queue;

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len != sizeof(FogMsg))
        return;
    RxEvent ev;
    memcpy(ev.mac, info->src_addr, 6);
    memcpy(&ev.msg, data, sizeof(FogMsg));
    if (ev.msg.magic != FOG_MAGIC)
        return;
    trig_rssi = info->rx_ctrl->rssi;
    trig_heard_ms = millis();
    trig_heard = true;
    xQueueSend(rx_queue, &ev, 0);
}

static void reply(const uint8_t *mac, uint8_t type, uint8_t seq) {
    if (!esp_now_is_peer_exist(mac)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, mac, 6);
        peer.channel = 0;
        peer.ifidx = WIFI_IF_AP;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }
    FogMsg ack = {FOG_MAGIC, (uint8_t)(MSG_ACK | type), seq};
    esp_now_send(mac, (const uint8_t *)&ack, sizeof ack);
}

static void handle_radio(uint32_t now) {
    static uint8_t last_seq = 0;
    static bool have_seq = false;
    RxEvent ev;
    while (xQueueReceive(rx_queue, &ev, 0) == pdTRUE) {
        uint8_t type = ev.msg.type;
        if (type == MSG_GROUP || type == MSG_UNDO) {
            // the trigger retries until acked: act once per sequence number
            if (!have_seq || ev.msg.seq != last_seq) {
                have_seq = true;
                last_seq = ev.msg.seq;
                if (type == MSG_GROUP)
                    add_group(now, false);
                else
                    undo_group();
            }
        }
        if (type == MSG_PING || type == MSG_GROUP || type == MSG_UNDO)
            reply(ev.mac, type, ev.msg.seq);
    }
}

/* ---- web UI ---- */
WebServer server(80);

static const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Fogbox</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;padding:16px;background:#111;color:#eee;max-width:520px}
h1{font-size:20px;margin:0 0 12px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:6px 12px;margin-bottom:16px}
.grid div:nth-child(odd){color:#999}
button{font-size:16px;padding:12px;margin:4px 0;width:100%;border:0;border-radius:8px;background:#335;color:#fff}
button.go{background:#2a6a3a}button.stop{background:#a22}
.row{display:grid;grid-template-columns:1fr 1fr;gap:8px}
label{display:grid;grid-template-columns:1fr 90px;align-items:center;margin:6px 0}
input{font-size:16px;padding:6px;width:80px}
#phase{font-weight:bold}
details{margin-top:16px}
</style></head><body>
<h1>Fogbox</h1>
<div class="grid">
<div>Phase</div><div id="phase">-</div>
<div>Groups queued</div><div id="groups">-</div>
<div>Next arrival</div><div id="next">-</div>
<div>Fog / mister</div><div id="out">-</div>
<div>Blower</div><div id="blower">-</div>
<div>Warm-up</div><div id="warm">-</div>
<div>Fallback</div><div id="fb">-</div>
<div>Trigger node</div><div id="trig">-</div>
<div>Run time fog / mist / blower</div><div id="tot">-</div>
</div>
<button class="go" onclick="cmd('group')">Group coming</button>
<div class="row">
<button onclick="cmd('undo')">Undo last group</button>
<button onclick="cmd('clear')">Clear all groups</button>
<button onclick="cmd('fog')">Fog burst 5 s</button>
<button onclick="cmd('mist')">Mister 15 s</button>
<button onclick="cmd('blower&pct='+document.getElementById('bt').value)">Blower test</button>
<label style="grid-template-columns:1fr 70px">at %<input id="bt" type="number" value="40" min="0" max="100"></label>
<button onclick="if(confirm('End show?'))cmd('end')">End show</button>
<button id="stopb" class="stop" onclick="cmd(stopped?'resume':'stop')">STOP</button>
</div>
<details><summary>Settings</summary>
<form id="f" onsubmit="save(event)">
<label>Walk time (s)<input name="walk_s"></label>
<label>Fallback interval (s, 0 = off)<input name="fallback_s"></label>
<label>Build-up (s)<input name="build_s"></label>
<label>Hold (s)<input name="hold_s"></label>
<label>Tail (s)<input name="tail_s"></label>
<label>Purge (s)<input name="purge_s"></label>
<label>Blower build-up %<input name="blow_build"></label>
<label>Blower hold %<input name="blow_hold"></label>
<label>Blower tail %<input name="blow_tail"></label>
<label>Blower purge %<input name="blow_purge"></label>
<label>Fog machine enabled (0/1)<input name="fog_enabled"></label>
<button type="submit">Save</button>
</form></details>
<script>
let stopped=false;
const $=id=>document.getElementById(id);
function mmss(s){s=Math.max(0,Math.round(s));return Math.floor(s/60)+':'+String(s%60).padStart(2,'0')}
function cmd(c){fetch('/cmd?c='+c,{method:'POST'}).then(r=>{if(!r.ok)r.text().then(alert);poll()})}
function poll(){fetch('/status').then(r=>r.json()).then(s=>{
 stopped=s.stop||s.sstop;
 $('phase').textContent=s.stop?'STOPPED (switch)':s.sstop?'STOPPED':s.phase;
 $('groups').textContent=s.groups;
 $('next').textContent=s.groups?(s.next_s>=0?'in '+mmss(s.next_s):mmss(-s.next_s)+' ago'):'-';
 $('out').textContent=(s.fog?'FOG':'fog off')+' / '+(s.mist?'MIST':'mist off');
 $('blower').textContent=s.blower+' %';
 $('warm').textContent=s.warm_s>0?mmss(s.warm_s)+' left':(s.fog_en?'ready':'fog disabled');
 $('fb').textContent=s.fb_armed?(s.fb_off?'off':'in '+mmss(s.fb_s)+' ('+s.fb_n+'/3 used)'):'waiting for trigger';
 $('trig').textContent=s.trig_age<0?'never heard':s.trig_rssi+' dBm, '+s.trig_age+' s ago';
 $('tot').textContent=mmss(s.fog_s)+' / '+mmss(s.mist_s)+' / '+mmss(s.blow_s);
 $('stopb').textContent=stopped?'Resume':'STOP';
}).catch(()=>{$('phase').textContent='no connection'})}
function load(){fetch('/settings').then(r=>r.json()).then(c=>{for(const k in c){const e=document.f[k];if(e)e.value=c[k]}})}
function save(e){e.preventDefault();fetch('/settings',{method:'POST',body:new URLSearchParams(new FormData($('f')))}).then(load)}
document.f=$('f');load();poll();setInterval(poll,1000);
</script></body></html>)HTML";

static void send_status() {
    uint32_t now = millis();
    long next_s = 0;
    if (n_groups) {
        uint32_t first = groups[0].trigger_ms;
        for (int i = 0; i < n_groups; i++) {
            // the next arrival still ahead, else the latest one passed
            int32_t rel = (int32_t)(groups[i].trigger_ms + cfg.walk_s * 1000UL - now);
            if (rel >= 0) {
                first = groups[i].trigger_ms;
                break;
            }
            first = groups[i].trigger_ms;
        }
        next_s = (long)((int32_t)(first + cfg.walk_s * 1000UL - now) / 1000);
    }
    long fb_s = cfg.fallback_s - (long)((now - last_trigger_ms) / 1000);
    char buf[512];
    snprintf(buf, sizeof buf,
             "{\"phase\":\"%s\",\"groups\":%d,\"next_s\":%ld,\"fog\":%d,\"mist\":%d,\"blower\":%d,"
             "\"warm_s\":%ld,\"fog_en\":%d,\"stop\":%d,\"sstop\":%d,\"fb_armed\":%d,\"fb_off\":%d,"
             "\"fb_s\":%ld,\"fb_n\":%d,\"trig_rssi\":%d,\"trig_age\":%ld,"
             "\"fog_s\":%lu,\"mist_s\":%lu,\"blow_s\":%lu,\"uptime_s\":%lu}",
             phase == PH_WAIT ? "idle (group on its way)" : phase_name[phase], n_groups, next_s,
             fog_on, mister_on, (int)(blower_pct + 0.5f),
             now < WARMUP_MS ? (long)((WARMUP_MS - now) / 1000) : 0L, cfg.fog_enabled, hw_stop, soft_stop,
             fallback_armed, cfg.fallback_s == 0, fb_s < 0 ? 0 : fb_s, fallback_count, (int)trig_rssi,
             trig_heard ? (long)((now - trig_heard_ms) / 1000) : -1L,
             (unsigned long)(fog_ms_total / 1000), (unsigned long)(mist_ms_total / 1000),
             (unsigned long)(blow_ms_total / 1000), (unsigned long)(now / 1000));
    server.send(200, "application/json", buf);
}

static void send_settings() {
    char buf[320];
    snprintf(buf, sizeof buf,
             "{\"walk_s\":%u,\"fallback_s\":%u,\"build_s\":%u,\"hold_s\":%u,\"tail_s\":%u,\"purge_s\":%u,"
             "\"blow_build\":%u,\"blow_hold\":%u,\"blow_tail\":%u,\"blow_purge\":%u,\"fog_enabled\":%u}",
             cfg.walk_s, cfg.fallback_s, cfg.build_s, cfg.hold_s, cfg.tail_s, cfg.purge_s, cfg.blow_build,
             cfg.blow_hold, cfg.blow_tail, cfg.blow_purge, cfg.fog_enabled);
    server.send(200, "application/json", buf);
}

static void arg_u16(const char *name, uint16_t &v, long lo, long hi) {
    if (server.hasArg(name) && server.arg(name).length())
        v = constrain(server.arg(name).toInt(), lo, hi);
}
static void arg_u8(const char *name, uint8_t &v, long lo, long hi) {
    if (server.hasArg(name) && server.arg(name).length())
        v = constrain(server.arg(name).toInt(), lo, hi);
}

static void post_settings() {
    arg_u16("walk_s", cfg.walk_s, 0, 1800);
    arg_u16("fallback_s", cfg.fallback_s, 0, 3600);
    arg_u16("build_s", cfg.build_s, 10, 600);
    arg_u16("hold_s", cfg.hold_s, 0, 600);
    arg_u16("tail_s", cfg.tail_s, 0, 600);
    arg_u16("purge_s", cfg.purge_s, 0, 600);
    arg_u8("blow_build", cfg.blow_build, 0, 100);
    arg_u8("blow_hold", cfg.blow_hold, 0, 100);
    arg_u8("blow_tail", cfg.blow_tail, 0, 100);
    arg_u8("blow_purge", cfg.blow_purge, 0, 100);
    arg_u8("fog_enabled", cfg.fog_enabled, 0, 1);
    save_settings();
    Serial.println("settings saved");
    send_settings();
}

static void post_cmd() {
    uint32_t now = millis();
    String c = server.arg("c");
    if (c == "group") {
        add_group(now, false);
    } else if (c == "undo") {
        undo_group();
    } else if (c == "clear") {
        n_groups = 0;
    } else if (c == "fog") {
        if (!start_burst(now, MANUAL_FOG_MS, true)) {
            server.send(409, "text/plain", "fog not allowed now (warm-up, gap, stop or disabled)");
            return;
        }
    } else if (c == "mist") {
        if (!soft_stop && !hw_stop)
            manual_mist_until = now + MANUAL_MIST_MS;
    } else if (c == "blower") {
        blower_test_pct = constrain(server.arg("pct").toInt(), 0, 100);
        blower_test_until = now + BLOWER_TEST_MS;
    } else if (c == "end") {
        fallback_armed = false;
        Serial.println("end of show: fallback off");
    } else if (c == "stop") {
        soft_stop = true;
        stop_all(now);
    } else if (c == "resume") {
        soft_stop = false;
    } else {
        server.send(400, "text/plain", "unknown command");
        return;
    }
    server.send(200, "text/plain", "ok");
}

/* ---- main ---- */
void setup() {
    pinMode(MISTER_PIN, OUTPUT);
    digitalWrite(MISTER_PIN, LOW);
    ledcAttach(BLOWER_PIN, BLOWER_PWM_HZ, BLOWER_PWM_BITS);
    ledcWrite(BLOWER_PIN, 0);
    pinMode(LED_PIN, OUTPUT);
    led(false);
    pinMode(STOP_PIN, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    Serial.println("fog_controller starting");
    load_settings();

    Serial1.setTxBufferSize(256);
    Serial1.begin(250000, SERIAL_8N2, -1, DMX_TX_PIN);
    xTaskCreate(dmx_task, "dmx", 2048, nullptr, 5, nullptr);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, ESPNOW_CHANNEL);
    WiFi.setSleep(false);
    WiFi.setTxPower(TX_POWER);
    Serial.printf("AP %s on %s, MAC %s\n", AP_SSID, WiFi.softAPIP().toString().c_str(),
                  WiFi.softAPmacAddress().c_str());

    rx_queue = xQueueCreate(16, sizeof(RxEvent));
    if (esp_now_init() != ESP_OK)
        Serial.println("ESP-NOW init failed");
    esp_now_register_recv_cb(on_recv);

    server.on("/", HTTP_GET, [] { server.send_P(200, "text/html", PAGE); });
    server.on("/status", HTTP_GET, send_status);
    server.on("/settings", HTTP_GET, send_settings);
    server.on("/settings", HTTP_POST, post_settings);
    server.on("/cmd", HTTP_POST, post_cmd);
    server.begin();
}

void loop() {
    static uint32_t last = millis();
    uint32_t now = millis();
    uint32_t dt = now - last;
    last = now;

    bool was_stop = hw_stop;
    hw_stop = digitalRead(STOP_PIN) == LOW;
    if (hw_stop && !was_stop) {
        Serial.println("STOP switch closed");
        stop_all(now);
    }

    switch (button.poll(now)) {
    case 1: add_group(now, false); break;
    case 2: undo_group(); break;
    }
    handle_radio(now);
    server.handleClient();

    if (fallback_armed && cfg.fallback_s && now - last_trigger_ms >= cfg.fallback_s * 1000UL)
        add_group(now, true);

    Phase prev = phase;
    phase = update_groups(now);
    if (phase != prev)
        Serial.printf("phase: %s\n", phase_name[phase]);
    run_fog(now, prev);
    run_outputs(now, dt);
    run_led(now);
    delay(5);
}
