#!/usr/bin/env python3
"""
FAKE ECU for the Truck Black Box (ESP32-S3)
-------------------------------------------
Runs on your PC. Pretends to be a truck ECU and sends one data line every 0.1 s
over the USB serial port. Also shows everything the ESP32 prints (lines starting with '#').

Install once:   pip install pyserial
Run:            python fake_ecu.py                 (finds the ESP32 port itself)
                python fake_ecu.py --port COM6
                python fake_ecu.py --auto          (hands-free demo, faults happen by themselves)

Line sent to the ESP32 (10 per second):
    D,speed,rpm,coolant,oil,battery,fuel,load,throttle,mil,derate
and every 10 s the PC time, so the device can show real date and time:
    T,<local epoch seconds>

KEYS while running
    g  go (cruise 80 km/h)        s  stop the vehicle (mechanic mode works only when stopped)
    1  coolant overheating        2  oil pressure drops       3  alternator / battery failure
    4  coolant sensor glitch      5  limp mode (power reduced) 6  low fuel
    7  engine over-rev (2 s)      l  ECU link lost for 8 s    0  clear all faults
    q  quit
"""
import argparse
import random
import sys
import threading
import time


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class Truck:
    def __init__(self):
        self.speed = 0.0
        self.prev_speed = 0.0
        self.target = 0.0
        self.cruise = 80.0
        self.coolant = 88.0
        self.oil = 230.0
        self.bat = 13.9
        self.fuel = 62.0
        self.fault_cool = False
        self.fault_oil = False
        self.fault_alt = False
        self.fault_derate = False
        self.low_fuel = False
        self.overrev_left = 0.0
        self.glitch_left = 0
        self.rpm = 800.0
        self.load = 10.0
        self.thr = 5.0

    # ---- things the keys do ----
    def go(self):
        self.target = self.cruise

    def stop(self):
        self.target = 0.0

    def clear(self):
        self.fault_cool = self.fault_oil = self.fault_alt = self.fault_derate = False
        self.low_fuel = False
        self.overrev_left = 0.0

    def any_fault(self):
        return (self.fault_cool or self.fault_oil or self.fault_alt or self.fault_derate or
                self.low_fuel or self.coolant >= 100.0)

    def derate(self):
        return self.fault_derate or self.coolant >= 112.0

    # ---- physics, called every 0.1 s ----
    def step(self, dt):
        tgt = self.target
        if self.derate():
            tgt = min(tgt, 40.0)                       # limp mode: ECU cuts power
        if tgt > self.speed:
            self.speed = min(tgt, self.speed + 6.0 * dt)
        else:
            self.speed = max(tgt, self.speed - 12.0 * dt)
        accel = (self.speed - self.prev_speed) / dt
        self.prev_speed = self.speed

        rpm = 800.0 + self.speed * 17.0 + random.uniform(-25, 25)
        if self.derate():
            rpm = min(rpm, 1800.0)
        if self.overrev_left > 0:
            rpm = 4500.0 + random.uniform(-80, 80)
            self.overrev_left -= dt
        self.rpm = rpm

        self.load = clamp(12.0 + self.speed * 0.45 + accel * 2.5 + random.uniform(-2, 2), 5, 100)
        self.thr = clamp(self.load * 0.9 + random.uniform(-2, 2), 0, 100)

        cool_target = 88.0 + self.load * 0.06
        if self.fault_cool:
            self.coolant = min(118.0, self.coolant + 0.9 * dt)       # about 30 s from 90 to 118
        elif self.coolant > cool_target:
            self.coolant = max(cool_target, self.coolant - 0.6 * dt)
        else:
            self.coolant = min(cool_target, self.coolant + 0.1 * dt)

        oil_target = 60.0 if self.fault_oil else 150.0 + self.rpm * 0.09
        self.oil += (oil_target - self.oil) * min(1.0, dt * 1.5) + random.uniform(-3, 3)

        if self.fault_alt:
            self.bat = max(10.6, self.bat - 0.1 * dt)
        else:
            self.bat += (13.9 - self.bat) * dt * 0.5
        self.bat += random.uniform(-0.01, 0.01)

        if self.low_fuel:
            self.fuel = min(self.fuel, 8.0)
        else:
            self.fuel = max(0.0, self.fuel - 0.004 * dt * (1.0 + self.speed / 60.0))

    def line(self):
        clt = self.coolant
        if self.glitch_left > 0:                        # one bad reading, then fine again
            clt = 140.0
            self.glitch_left -= 1
        mil = 1 if self.any_fault() else 0
        return "D,%.1f,%.0f,%.1f,%.0f,%.2f,%.0f,%.0f,%.0f,%d,%d\n" % (
            self.speed, self.rpm, clt, self.oil, self.bat, self.fuel, self.load, self.thr,
            mil, 1 if self.derate() else 0)


# hands-free demo: (seconds after start, key)
AUTO_SCRIPT = [
    (0, 'g'), (45, '4'), (90, '1'), (140, '0'), (180, '2'), (215, '0'),
    (250, '3'), (285, '0'), (320, 's'),
]


def apply_key(truck, k, state):
    k = k.lower()
    if k == 'g': truck.go(); return "driving"
    if k == 's': truck.stop(); return "stopping"
    if k == '1': truck.fault_cool = True; return "COOLANT OVERHEAT started"
    if k == '2': truck.fault_oil = True; return "OIL PRESSURE DROP started"
    if k == '3': truck.fault_alt = True; return "ALTERNATOR FAILURE started"
    if k == '4': truck.glitch_left = 2; return "coolant SENSOR GLITCH"
    if k == '5': truck.fault_derate = True; return "LIMP MODE (derate)"
    if k == '6': truck.low_fuel = True; return "LOW FUEL"
    if k == '7': truck.overrev_left = 2.0; return "OVER-REV"
    if k == 'l': state['mute_until'] = time.time() + 8.0; return "ECU LINK LOST for 8 s"
    if k == '0': truck.clear(); return "all faults cleared"
    if k == 'q': state['quit'] = True; return "bye"
    return None


class Keys:
    """Reads single key presses without Enter (Windows, Linux, Mac)."""
    def __init__(self):
        self.win = sys.platform.startswith('win')
        self.ok = sys.stdin is not None and sys.stdin.isatty()
        self.old = None
        if self.ok and not self.win:
            import termios, tty
            self.termios = termios
            self.old = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())

    def get(self):
        if not self.ok:
            return None
        if self.win:
            import msvcrt
            if msvcrt.kbhit():
                return msvcrt.getwch()
            return None
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def close(self):
        if self.old is not None:
            self.termios.tcsetattr(sys.stdin.fileno(), self.termios.TCSADRAIN, self.old)


def find_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    best = []
    for p in ports:
        text = ((p.description or '') + ' ' + (p.manufacturer or '')).lower()
        if p.vid == 0x303A or any(w in text for w in ('cp210', 'ch340', 'ch910', 'silicon labs', 'usb serial', 'usb jtag', 'espressif')):
            best.append(p)
    if len(best) == 1:
        return best[0].device
    if not best:
        best = ports
    if not best:
        sys.exit("No serial port found. Plug in the ESP32-S3 (or use --port COMx).")
    print("Several ports found:")
    for i, p in enumerate(best):
        print("  %d) %s  %s" % (i + 1, p.device, p.description))
    return best[int(input("Choose number: ")) - 1].device


def reader_thread(ser, stop):
    buf = b''
    while not stop.is_set():
        try:
            data = ser.read(256)
        except Exception:
            break
        if not data:
            continue
        buf += data
        while b'\n' in buf:
            raw, buf = buf.split(b'\n', 1)
            text = raw.decode('utf-8', 'replace').strip()
            if text:
                print("\n  ESP32> " + text)


def main():
    ap = argparse.ArgumentParser(description="Fake ECU for Truck Black Box")
    ap.add_argument('--port', help="serial port, e.g. COM6 or /dev/ttyACM0")
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--auto', action='store_true', help="run the hands-free demo script")
    ap.add_argument('--dry', action='store_true', help="no serial port, print lines (testing)")
    ap.add_argument('--fast', action='store_true', help="with --dry: do not wait between lines")
    ap.add_argument('--seconds', type=float, default=0, help="stop after this many seconds (0 = never)")
    args = ap.parse_args()

    ser = None
    stop = threading.Event()
    if not args.dry:
        try:
            import serial
        except ImportError:
            sys.exit("pyserial missing.  Run:  pip install pyserial")
        port = args.port or find_port()
        print("Opening %s ..." % port)
        ser = serial.Serial(port, args.baud, timeout=0.1)
        time.sleep(2.5)                                  # board may reset when the port opens
        threading.Thread(target=reader_thread, args=(ser, stop), daemon=True).start()

    truck = Truck()
    state = {'quit': False, 'mute_until': 0.0}
    keys = Keys()
    print(__doc__.split('KEYS while running')[1] if not args.auto else "AUTO demo running. Press q to quit.")

    def send(text):
        if ser:
            ser.write(text.encode())
        elif args.dry and not args.fast:
            sys.stdout.write(text)

    dt = 0.1
    t0 = time.time()
    next_tick = time.monotonic()
    next_sync = 0.0
    script = list(AUTO_SCRIPT) if args.auto else []
    ticks = 0
    last_msg = ""
    try:
        while not state['quit']:
            now = time.time()
            elapsed = ticks * dt if args.fast else now - t0
            if args.seconds and elapsed >= args.seconds:
                break
            while script and elapsed >= script[0][0]:
                msg = apply_key(truck, script.pop(0)[1], state)
                last_msg = msg or last_msg
                print("\n[auto %3.0fs] %s" % (elapsed, last_msg))
            k = keys.get()
            if k:
                msg = apply_key(truck, k, state)
                if msg:
                    last_msg = msg
                    print("\n[key %s] %s" % (k, msg))
            truck.step(dt)
            if now >= next_sync or args.fast and ticks % 100 == 0:
                send("T,%d\n" % int(now + time.localtime().tm_gmtoff))
                next_sync = now + 10.0
            if now >= state['mute_until'] or args.fast:
                send(truck.line())
            ticks += 1
            if ticks % 10 == 0 and not (args.dry and not args.fast and False):
                sys.stdout.write("\r spd %5.1f  rpm %4.0f  cool %5.1f  oil %3.0f  bat %4.1f  fuel %2.0f  %s   " % (
                    truck.speed, truck.rpm, truck.coolant, truck.oil, truck.bat, truck.fuel,
                    "LINK CUT" if now < state['mute_until'] else ""))
                sys.stdout.flush()
            if not args.fast:
                next_tick += dt
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        keys.close()
        if ser:
            ser.close()
        print("\nstopped.")


if __name__ == '__main__':
    main()
