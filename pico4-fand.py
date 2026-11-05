#!/usr/bin/env python3
"""
pico4-fand: PICO 4 fan control.

The fan is not on an SoC PWM: the GD32 MCU (the one that also drives the IPD
lens motor) runs it and measures its speed. Stock talks to it from
gd32ipdservice over UART (stock /dev/ttyHS1, here /dev/ttyHS0 = SE1),
115200 8N1. Frames (reverse engineered from libgd32ipdservice.so,
Gd32ipdService::SetFanSpeed/GetFanSpeed/GetFanRPM/GetHallTempADC):

  request  a0 <cmd> 00 <lo> <hi> <xor of the 5 bytes before>
  reply    79 <lo> <hi> <xor of the 3 bytes before>     (set: just 79)

  0x0e set speed (0..100 %), 0x0f get speed, 0x10 get RPM,
  0x23 hall/temperature ADC (raw)

The daemon picks the hottest CPU/GPU/memory thermal zone, maps it on a fan
curve (with hysteresis for switching off and a slow ramp down), and
publishes its state in /run/pico4-fand/status. Writing a number (0-100) to
/run/pico4-fand/override pins the speed, "auto" returns to the curve.

  pico4-fand              run the daemon
  pico4-fand --status     print the MCU's view (speed, RPM, ADC)
  pico4-fand --set N      set N % once (for testing without the daemon;
                          the daemon overrides it on its next round)
"""
import glob
import json
import os
import re
import select
import signal
import sys
import termios
import time

DEVICE = os.environ.get("PICO4_FAN_TTY", "/dev/ttyHS0")
RUN_DIR = "/run/pico4-fand"
CONFIG = "/etc/pico4-fand.conf"

# Fan curve: (degrees C, percent), linear in between.
CURVE = [(45.0, 20), (55.0, 35), (65.0, 60), (75.0, 100)]
OFF_BELOW = 42.0        # switch off below this (on again at CURVE[0])
RAMP_DOWN = 3           # max % decrease per interval
INTERVAL = 2.0          # seconds
EXIT_MIN = 40           # never leave a hot SoC with the fan off
ZONES = r"^(cpu|cluster|gpu|mem)"

CMD_SET, CMD_SPEED, CMD_RPM, CMD_ADC = 0x0E, 0x0F, 0x10, 0x23
ACK = 0x79


class Gd32Fan:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        a = termios.tcgetattr(self.fd)
        a[0] = 0
        a[1] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[3] = 0
        a[4] = a[5] = termios.B115200
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def _xfer(self, cmd, value, reply_len):
        frame = [0xA0, cmd, 0x00, value & 0xFF, (value >> 8) & 0xFF]
        x = 0
        for b in frame:
            x ^= b
        for _ in range(3):
            termios.tcflush(self.fd, termios.TCIFLUSH)
            os.write(self.fd, bytes(frame + [x]))
            out = b""
            end = time.monotonic() + 0.2
            while len(out) < reply_len and time.monotonic() < end:
                r, _, _ = select.select([self.fd], [], [], 0.05)
                if r:
                    out += os.read(self.fd, 64)
            if len(out) >= reply_len and out[0] == ACK:
                if reply_len == 1 or out[3] == out[0] ^ out[1] ^ out[2]:
                    return out[:reply_len]
        raise IOError("GD32: no valid reply to command 0x%02x" % cmd)

    def _get16(self, cmd):
        r = self._xfer(cmd, 0, 4)
        return r[1] | r[2] << 8

    def set_speed(self, percent):
        self._xfer(CMD_SET, max(0, min(100, int(percent))), 1)

    def speed(self):
        return self._get16(CMD_SPEED)

    def rpm(self):
        return self._get16(CMD_RPM)

    def adc(self):
        return self._get16(CMD_ADC)


def load_config():
    global CURVE, OFF_BELOW, RAMP_DOWN, INTERVAL, ZONES
    try:
        with open(CONFIG) as f:
            cfg = json.load(f)
    except FileNotFoundError:
        return
    CURVE = [tuple(p) for p in cfg.get("curve", CURVE)]
    OFF_BELOW = cfg.get("off_below", OFF_BELOW)
    RAMP_DOWN = cfg.get("ramp_down", RAMP_DOWN)
    INTERVAL = cfg.get("interval", INTERVAL)
    ZONES = cfg.get("zones", ZONES)


def thermal_zones():
    zones = []
    for z in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        try:
            with open(z + "/type") as f:
                t = f.read().strip()
        except OSError:
            continue
        if re.search(ZONES, t):
            zones.append((t, z + "/temp"))
    return zones


def hottest(zones):
    best = None
    for name, path in zones:
        try:
            with open(path) as f:
                v = int(f.read()) / 1000.0
        except (OSError, ValueError):
            continue
        if best is None or v > best[1]:
            best = (name, v)
    return best


def curve_percent(temp, running):
    if temp < OFF_BELOW or (not running and temp < CURVE[0][0]):
        return 0
    if temp <= CURVE[0][0]:
        return CURVE[0][1]
    for (t0, p0), (t1, p1) in zip(CURVE, CURVE[1:]):
        if temp <= t1:
            return round(p0 + (p1 - p0) * (temp - t0) / (t1 - t0))
    return CURVE[-1][1]


def read_override():
    try:
        with open(RUN_DIR + "/override") as f:
            v = f.read().strip()
    except OSError:
        return None
    if v.isdigit():
        return max(0, min(100, int(v)))
    return None


def write_status(state):
    tmp = RUN_DIR + "/status.tmp"
    with open(tmp, "w") as f:
        json.dump(state, f)
        f.write("\n")
    os.replace(tmp, RUN_DIR + "/status")


def daemon():
    load_config()
    fan = Gd32Fan(DEVICE)
    zones = thermal_zones()
    if not zones:
        sys.exit("no thermal zones match %r" % ZONES)
    os.makedirs(RUN_DIR, exist_ok=True)
    print("pico4-fand: %s, %d zones, curve %s, off below %.0f C" %
          (DEVICE, len(zones), CURVE, OFF_BELOW), flush=True)

    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))

    current, stalled, last_log = None, 0, None
    while not stop:
        name, temp = hottest(zones)
        override = read_override()
        target = override if override is not None else curve_percent(temp, bool(current))
        # Up at once, down slowly: avoids the fan pumping on short spikes.
        if current is not None and target < current and override is None:
            target = max(target, current - RAMP_DOWN)
        try:
            if target != current:
                fan.set_speed(target)
                current = target
            rpm = fan.rpm()
            adc = fan.adc()
        except IOError as e:
            print("pico4-fand: %s" % e, flush=True)
            current = None  # resend next round
            time.sleep(INTERVAL)
            continue

        stalled = stalled + 1 if current and rpm == 0 else 0
        if stalled == 3:
            print("pico4-fand: fan reports 0 RPM at %d %%, resending" % current, flush=True)
            current = None
        state = {"temp_c": temp, "zone": name, "speed_pct": current, "rpm": rpm,
                 "hall_adc": adc, "override": override}
        write_status(state)
        # Log overrides and steps of 5 % or more, not every ramp step.
        log = (current, override)
        if (last_log is None or log[1] != last_log[1] or
                abs((current or 0) - (last_log[0] or 0)) >= 5 or (current == 0) != (last_log[0] == 0)):
            print("pico4-fand: %.1f C (%s) -> %s %%%s, %d rpm" %
                  (temp, name, current, " (override)" if override is not None else "", rpm),
                  flush=True)
            last_log = log
        for _ in range(int(INTERVAL * 10)):
            if stop:
                break
            time.sleep(0.1)

    # Never leave a hot SoC without airflow.
    try:
        fan.set_speed(max(current or 0, EXIT_MIN))
    except IOError:
        pass
    print("pico4-fand: exit, fan left at %d %%" % max(current or 0, EXIT_MIN), flush=True)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--status":
        fan = Gd32Fan(DEVICE)
        print("speed %d %%, %d rpm, hall ADC %d" % (fan.speed(), fan.rpm(), fan.adc()))
    elif len(sys.argv) > 2 and sys.argv[1] == "--set":
        Gd32Fan(DEVICE).set_speed(int(sys.argv[2]))
    else:
        daemon()


if __name__ == "__main__":
    main()
