#!/usr/bin/env python3
"""Generates fog_mister.svg and fog_trigger.svg.

Usage: python3 docs/gen_schematics.py docs
"""
import sys

STYLE = """
<style>
text{font-family:Helvetica,Arial,sans-serif;fill:#1d1d1f}
.t{font-size:15px;font-weight:bold}
.s{font-size:11.5px;fill:#555}
.p{font-size:11px;fill:#333}
.n{font-size:11px;fill:#666;font-style:italic}
.h{font-size:22px;font-weight:bold}
.box{fill:#f6f7f9;stroke:#333;stroke-width:1.5}
.mcu{fill:#e9f1fb;stroke:#2b5aa0;stroke-width:2}
.dev{fill:#fff6e8;stroke:#a86a10;stroke-width:1.5}
.w{fill:none;stroke-width:2;stroke-linejoin:round;stroke-linecap:round}
.mains{stroke:#8b4513;stroke-width:3.5;stroke-dasharray:10 5}
.v12{stroke:#d22}
.v5{stroke:#e07b00}
.v33{stroke:#1a7fd4}
.sig{stroke:#1d9a4a}
.dmx{stroke:#7b3fbf}
.iso{stroke:#c2187a}
.gnd{stroke:#222}
.c{fill:none;stroke:#222;stroke-width:1.6}
.cable{fill:none;stroke:#7b3fbf;stroke-width:1;stroke-dasharray:4 3}
</style>"""


class Svg:
    def __init__(s, w, h, title):
        s.w, s.h, s.o = w, h, []
        s.o.append(f'<rect width="{w}" height="{h}" fill="#fff"/>')
        s.text(24, 36, title, "h")

    def add(s, x):
        s.o.append(x)

    def text(s, x, y, t, cls="p", anchor="start"):
        t = t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        s.add(f'<text x="{x}" y="{y}" class="{cls}" text-anchor="{anchor}">{t}</text>')

    def box(s, x, y, w, h, title, subs=(), cls="box"):
        s.add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="6" class="{cls}"/>')
        s.text(x + w / 2, y + 22, title, "t", "middle")
        for i, l in enumerate(subs):
            s.text(x + w / 2, y + 39 + i * 15, l, "s", "middle")

    def wire(s, pts, cls):
        d = " ".join(f"{x},{y}" for x, y in pts)
        s.add(f'<polyline points="{d}" class="w {cls}"/>')

    def dot(s, x, y):
        s.add(f'<circle cx="{x}" cy="{y}" r="3.5" fill="#222"/>')

    def pin(s, x, y, label, side):
        # label inside the box next to the pin
        s.add(f'<circle cx="{x}" cy="{y}" r="2.8" fill="#fff" stroke="#333" stroke-width="1.3"/>')
        if side == "l":
            s.text(x + 7, y + 4, label, "p")
        elif side == "r":
            s.text(x - 7, y + 4, label, "p", "end")
        elif side == "t":
            s.text(x + 7, y - 5, label, "p")
        else:
            s.text(x, y - 7, label, "p", "middle")

    def gnd(s, x, y):
        s.add(f'<path d="M{x},{y} v10 M{x-10},{y+10} h20 M{x-6},{y+15} h12 M{x-2.5},{y+20} h5" class="c"/>')

    def tag(s, x, y, label, cls="v12", direction="up"):
        # supply net label: a short stub and a flag
        col = {"v12": "#d22", "v5": "#e07b00", "v33": "#1a7fd4"}[cls]
        if direction == "up":
            s.wire([(x, y), (x, y - 14)], cls)
            s.add(f'<path d="M{x-6},{y-14} h12 l-6,-8 z" fill="{col}"/>')
            s.add(f'<text x="{x}" y="{y-27}" class="p" text-anchor="middle" style="fill:{col};font-weight:bold">{label}</text>')
        elif direction == "down":
            s.wire([(x, y), (x, y + 14)], cls)
            s.add(f'<path d="M{x-6},{y+14} h12 l-6,8 z" fill="{col}"/>')
            s.add(f'<text x="{x}" y="{y+36}" class="p" text-anchor="middle" style="fill:{col};font-weight:bold">{label}</text>')
        else:  # left
            s.wire([(x, y), (x - 16, y)], cls)
            s.add(f'<path d="M{x-16},{y-6} v12 l-8,-6 z" fill="{col}"/>')
            s.add(f'<text x="{x-28}" y="{y+4}" class="p" text-anchor="end" style="fill:{col};font-weight:bold">{label}</text>')

    def resistor_v(s, x, y1, y2, label):
        m = (y1 + y2) / 2
        pts = [(x, y1), (x, m - 14)]
        for i in range(7):
            pts.append((x + (6 if i % 2 == 0 else -6), m - 14 + 4 * i + 2))
        pts += [(x, m + 14), (x, y2)]
        d = " ".join(f"{a},{b}" for a, b in pts)
        s.add(f'<polyline points="{d}" class="c"/>')
        s.text(x + 10, m + 4, label, "p")

    def resistor_h(s, x1, x2, y, label):
        m = (x1 + x2) / 2
        pts = [(x1, y), (m - 14, y)]
        for i in range(7):
            pts.append((m - 14 + 4 * i + 2, y + (6 if i % 2 == 0 else -6)))
        pts += [(m + 14, y), (x2, y)]
        d = " ".join(f"{a},{b}" for a, b in pts)
        s.add(f'<polyline points="{d}" class="c"/>')
        s.text(m, y + 20, label, "p", "middle")

    def diode_v(s, x, y_cathode, y_anode, label):
        # anode at the bottom, cathode (bar) at the top: current flows up
        m = (y_cathode + y_anode) / 2
        s.add(f'<path d="M{x},{y_anode} V{m+7} M{x},{y_cathode} V{m-7} M{x-9},{m-7} h18" class="c"/>')
        s.add(f'<path d="M{x-9},{m+7} h18 l-9,-14 z" fill="#222"/>')
        s.text(x + 13, m + 4, label, "p")

    def switch_h(s, x1, x2, y, label, push=False):
        a, b = x1 + (x2 - x1) * 0.3, x1 + (x2 - x1) * 0.7
        s.add(f'<path d="M{x1},{y} H{a} M{b},{y} H{x2}" class="c"/>')
        s.add(f'<circle cx="{a}" cy="{y}" r="2.5" fill="#222"/><circle cx="{b}" cy="{y}" r="2.5" fill="#222"/>')
        if push:
            s.add(f'<path d="M{a-2},{y-9} H{b+2} M{(a+b)/2},{y-9} V{y-17} M{(a+b)/2-6},{y-17} h12" class="c"/>')
        else:
            s.add(f'<path d="M{a},{y} L{b-2},{y-13}" class="c"/>')
        for i, l in enumerate(label if isinstance(label, list) else [label]):
            s.text((x1 + x2) / 2, y + 22 + i * 14, l, "p", "middle")

    def legend(s, x, y, items):
        for i, (cls, label) in enumerate(items):
            yy = y + i * 20
            s.wire([(x, yy), (x + 34, yy)], cls)
            s.text(x + 44, yy + 4, label, "p")

    def save(s, path):
        with open(path, "w") as f:
            f.write(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {s.w} {s.h}" width="{s.w}" height="{s.h}">')
            f.write(STYLE + "\n" + "\n".join(s.o) + "\n</svg>\n")


def mister(path):
    s = Svg(1400, 1010, "Fogbox: mister box (fog_mister)")
    s.add('<g transform="translate(0,40)">')

    # mains
    s.box(40, 40, 160, 60, "Fossibot F2400", ["230 V AC out"], "dev")
    s.box(250, 40, 150, 60, "Portable RCD", ["30 mA PRCD"])
    s.wire([(200, 70), (250, 70)], "mains")
    s.wire([(400, 70), (1350, 70)], "mains")
    s.text(700, 62, "230 V AC: outdoor IP44 extensions, connectors off the ground, drip loops", "n")
    for x, y in ((520, 130), (1245, 130), (1350, 660)):
        s.dot(x, 70)
        s.wire([(x, 70), (x, y)], "mains")

    # 12 V supply
    s.box(440, 130, 160, 70, "12 V 60 W supply", ["BRIMETI, IP67"])
    s.pin(480, 200, "+12V", "b")
    s.pin(560, 200, "−", "b")
    s.wire([(480, 200), (480, 222)], "v12")
    s.add('<path d="M474,222 h12 l-6,8 z" fill="#d22"/>')
    s.add('<text x="480" y="246" class="p" text-anchor="middle" style="fill:#d22;font-weight:bold">+12V</text>')
    s.gnd(560, 200)

    # buck
    s.box(100, 280, 160, 90, "Buck 12 → 5 V", [])
    s.pin(140, 280, "IN+", "t")
    s.tag(140, 280, "+12V")
    s.pin(180, 370, "GND", "b")
    s.gnd(180, 370)
    s.pin(260, 335, "5V out", "r")
    s.wire([(260, 335), (295, 335), (295, 305), (330, 305)], "v5")

    # ESP32-S3 SuperMini
    s.box(330, 270, 220, 460, "ESP32-S3", ["SuperMini", "fog_mister"], "mcu")
    s.text(440, 440, "Wi-Fi AP “fogbox”, ch 1", "s", "middle")
    s.text(440, 456, "ESP-NOW ← trigger node", "s", "middle")
    s.text(440, 472, "web UI 192.168.4.1", "s", "middle")
    s.text(440, 500, "GPIO48: on-board RGB LED", "s", "middle")
    s.pin(330, 305, "5V", "l")
    s.pin(330, 360, "GND", "l")
    s.wire([(330, 360), (300, 360)], "gnd")
    s.gnd(300, 360)
    s.pin(330, 580, "GPIO7", "l")
    s.pin(330, 660, "GPIO10", "l")
    s.pin(550, 380, "GPIO4", "r")
    s.pin(550, 520, "GPIO5", "r")
    s.pin(550, 660, "GPIO3", "r")
    s.pin(500, 730, "3V3", "b")

    # STOP and button
    s.wire([(330, 580), (260, 580)], "sig")
    s.switch_h(160, 260, 580, ["STOP toggle", "closed = all off"])
    s.wire([(160, 580), (100, 580)], "gnd")
    s.gnd(100, 580)
    s.wire([(330, 660), (260, 660)], "sig")
    s.switch_h(160, 260, 660, ["Push button: group,", "hold 2 s = undo"], push=True)
    s.wire([(160, 660), (100, 660)], "gnd")
    s.gnd(100, 660)

    # MOSFET #1 -> blower
    s.box(700, 330, 180, 100, "MOSFET #1", ["blower PWM"])
    s.wire([(550, 380), (580, 380)], "sig")
    s.resistor_h(580, 620, 380, "220 Ω")
    s.wire([(620, 380), (700, 380)], "sig")
    s.text(600, 372, "PWM 20 kHz", "n")
    s.pin(700, 380, "TRIG", "l")
    s.dot(640, 380)
    s.resistor_v(640, 380, 440, "10 kΩ")
    s.gnd(640, 440)
    s.pin(790, 330, "VIN+", "t")
    s.tag(790, 330, "+12V")
    s.pin(790, 430, "VIN−", "b")
    s.gnd(790, 430)
    s.pin(880, 380, "OUT+", "r")
    s.pin(880, 410, "OUT−", "r")
    s.box(1060, 330, 120, 100, "Blower", [], "dev")
    s.text(1120, 425, "3\" 12 V bilge", "s", "middle")
    s.add('<circle cx="1130" cy="395" r="14" class="c"/><text x="1130" y="400" class="t" text-anchor="middle">M</text>')
    s.pin(1060, 380, "+", "l")
    s.pin(1060, 410, "−", "l")
    s.wire([(880, 380), (1060, 380)], "v12")
    s.wire([(880, 410), (1060, 410)], "gnd")
    s.dot(990, 380); s.dot(990, 410)
    s.diode_v(990, 380, 410, "SB540")
    s.text(905, 372, "red", "n"); s.text(905, 428, "black", "n")

    # MOSFET #2 -> SSR
    s.box(700, 500, 180, 100, "MOSFET #2", ["mister on/off"])
    s.wire([(550, 520), (580, 520)], "sig")
    s.resistor_h(580, 620, 520, "220 Ω")
    s.wire([(620, 520), (640, 520), (640, 550), (700, 550)], "sig")
    s.pin(700, 550, "TRIG", "l")
    s.dot(640, 550)
    s.resistor_v(640, 550, 610, "10 kΩ")
    s.gnd(640, 610)
    s.pin(790, 500, "VIN+", "t")
    s.tag(790, 500, "+12V")
    s.pin(790, 600, "VIN−", "b")
    s.gnd(790, 600)
    s.pin(880, 550, "OUT+", "r")
    s.pin(880, 580, "OUT−", "r")
    s.box(1000, 500, 150, 110, "SSR-40DD", [])
    s.text(1075, 630, "heatsink, outside the cooler", "n", "middle")
    s.pin(1000, 545, "ctrl +", "l")
    # OUT+ is the module's 12 V pass-through; the SSR control + hangs on it
    s.wire([(880, 550), (940, 550), (940, 545), (1000, 545)], "v12")
    s.pin(1000, 580, "ctrl −", "l")
    s.wire([(880, 580), (1000, 580)], "sig")
    s.pin(1150, 545, "DC in", "r")
    s.pin(1150, 580, "DC out", "r")

    # mister supply (isolated)
    s.box(1170, 130, 150, 70, "Mister brick", ["230 V AC → DC, 350 W"], "dev")
    s.pin(1200, 200, "DC+", "b")
    s.pin(1290, 200, "DC−", "b")
    s.wire([(1200, 200), (1200, 545), (1150, 545)], "iso")
    s.box(1230, 500, 100, 110, "Mister", ["8-head", "ultrasonic"], "dev")
    s.pin(1230, 580, "+", "l")
    s.pin(1290, 500, "−", "t")
    s.wire([(1150, 580), (1230, 580)], "iso")
    s.wire([(1290, 200), (1290, 500)], "iso")
    s.text(1190, 280, "isolated DC: never tied to GND", "n", "end")
    s.text(1190, 294, "check the brick's output is DC!", "n", "end")

    # DMX
    s.box(700, 650, 180, 150, "MAX485 module", ["5 V RS-485"])
    s.wire([(550, 660), (620, 660), (620, 700), (700, 700)], "dmx")
    s.text(565, 652, "DMX 250 kbaud", "n")
    s.pin(700, 700, "DI", "l")
    s.pin(730, 800, "VCC", "b")
    s.pin(775, 800, "DE", "b")
    s.pin(820, 800, "/RE", "b")
    s.pin(860, 800, "GND", "b")
    s.gnd(860, 800)
    s.wire([(500, 730), (500, 845), (820, 845)], "v33")
    s.tag(730, 800, "+5V", cls="v5", direction="down")
    for x in (775, 820):
        s.wire([(x, 845), (x, 800)], "v33")
    s.dot(775, 845)
    s.text(560, 863, "VCC to +5V; DE and /RE to 3V3 (transmit only)", "n")
    # MAX485: B is the non-inverting line, so B goes to XLR pin 3 (Data+)
    s.pin(880, 720, "B", "r")
    s.pin(880, 745, "A", "r")
    s.pin(880, 770, "GND", "r")
    s.box(1170, 660, 200, 130, "BeamZ BLAZE1200", ["DMX mode, addr 1, ch 1 = fog"], "dev")
    s.pin(1170, 720, "pin 3 Data+", "l")
    s.pin(1170, 745, "pin 2 Data−", "l")
    s.pin(1170, 770, "pin 1 GND", "l")
    s.wire([(880, 720), (1170, 720)], "dmx")
    s.wire([(880, 745), (1170, 745)], "dmx")
    s.wire([(880, 770), (1170, 770)], "gnd")
    s.add('<rect x="960" y="705" width="140" height="80" rx="8" class="cable"/>')
    s.text(1030, 803, "3-pin XLR DMX cable; 120 Ω across", "n", "middle")
    s.text(1030, 817, "pins 2–3 at the machine only if", "n", "middle")
    s.text(1030, 831, "it acts erratic", "n", "middle")

    s.add("</g>")
    s.legend(40, 920, [("mains", "230 V AC"), ("v12", "+12 V"), ("v5", "+5 V"), ("v33", "+3.3 V")])
    s.legend(300, 920, [("sig", "logic signal"), ("dmx", "DMX / RS-485"), ("iso", "isolated ultrasonic mister DC"), ("gnd", "ground")])
    s.text(560, 924, "All ground symbols are one common ground: 12 V supply −, buck, ESP32, MOSFET VIN−, MAX485.", "p")
    s.text(560, 944, "+12V flags connect to the 12 V supply's + output, +5V flags to the buck's 5 V output. The ultrasonic mister's DC side stays isolated behind the SSR.", "p")
    s.text(560, 964, "10 kΩ pull-downs keep the MOSFETs off while the ESP32 boots. Diode band (cathode) to blower +.", "p")
    s.text(560, 984, "Fog machine stand: horizontal, 50 cm clear around vents, 5 cm gap from nozzle to the duct.", "p")
    s.save(path)


def trigger(path):
    s = Svg(920, 540, "Fogbox trigger node (fog_trigger)")
    s.add('<g transform="translate(0,30)">')
    s.box(40, 60, 150, 80, "USB power bank", ["no auto-off at ~80 mA"], "dev")
    s.box(320, 60, 220, 330, "ESP32-S3", ["SuperMini"], "mcu")
    s.text(430, 130, "Wi-Fi STA on ch 1, not joined", "s", "middle")
    s.text(430, 146, "ESP-NOW → mister", "s", "middle")
    s.text(430, 162, "100–200 m in woods", "s", "middle")
    s.text(430, 190, "GPIO48: on-board RGB LED", "s", "middle")
    s.text(430, 206, "blip = link OK, 2 Hz = no link", "s", "middle")
    s.text(430, 222, "on 1.5 s = confirmed", "s", "middle")
    s.text(430, 238, "3 long flashes = not confirmed", "s", "middle")
    s.pin(320, 100, "USB", "l")
    s.wire([(190, 100), (320, 100)], "v5")
    s.text(255, 92, "USB 5 V", "n", "middle")
    # antenna
    s.add('<path d="M540,90 H580 V60 M568,48 L580,60 L592,48" class="c"/>')
    s.text(600, 58, "))) ESP-NOW", "s")
    s.pin(320, 310, "GPIO3", "l")
    s.wire([(320, 310), (260, 310)], "sig")
    s.switch_h(160, 260, 310, ["Push button: group,", "hold 2 s = undo"], push=True)
    s.wire([(160, 310), (100, 310)], "gnd")
    s.gnd(100, 310)
    s.pin(320, 370, "GND", "l")
    s.wire([(320, 370), (290, 370), (290, 380)], "gnd")
    s.gnd(290, 380)

    s.pin(540, 290, "3V3", "r")
    s.pin(540, 330, "GPIO4", "r")
    s.pin(540, 370, "GND", "r")
    s.box(680, 230, 200, 160, "PIR sensor", ["AM312 (optional)", "USE_SENSOR = true"], "dev")
    s.pin(680, 290, "VCC", "l")
    s.pin(680, 330, "OUT", "l")
    s.pin(680, 370, "GND", "l")
    s.wire([(540, 290), (680, 290)], "v33")
    s.wire([(540, 330), (680, 330)], "sig")
    s.wire([(540, 370), (680, 370)], "gnd")
    s.add("</g>")
    s.legend(40, 460, [("v5", "USB 5 V"), ("v33", "+3.3 V"), ("sig", "logic signal"), ("gnd", "ground")])
    s.text(300, 464, "Mount where the LED still blips reliably; RSSI above about −85 dBm on the page is safe.", "p")
    s.text(300, 484, "Board with an external antenna connector helps range. SuperMini boards:", "p")
    s.text(300, 504, "set TX_POWER to WIFI_POWER_8_5dBm on both nodes.", "p")
    s.save(path)


out = sys.argv[1]
mister(f"{out}/fog_mister.svg")
trigger(f"{out}/fog_trigger.svg")
