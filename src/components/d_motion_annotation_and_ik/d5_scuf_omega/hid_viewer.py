"""Live Omega HID monitor; never changes button mappings.

PC mode temporarily enables touch acquisition and subscribes to IMU reports.
Normal shutdown restores the original touch setting and closes the session.

Run with the repository's .venv/bin/python. On macOS grant Input Monitoring
to the host application. Switch device mode before launching the viewer.
"""

import binascii
import ctypes
import os
from pathlib import Path
import queue
import struct
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[4]
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".tools/matplotlib-cache"))

import hid
import libusb_package
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
import usb.core
import usb.util


BG, PANEL, IDLE = "#101722", "#192435", "#324358"
ACTIVE, TEXT, UNKNOWN = "#59e3a5", "#e3edf7", "#ffcf70"
EXTRAS = ["L outer", "R outer", "L inner", "R inner", "L SAX", "R SAX"]
G_KEYS = [f"G{i}" for i in range(1, 6)]
# User's SCUF App profile; HID Keyboard/Keypad usage IDs, independent of OS layout.
EXTRA_KEYBOARD = {
    "L inner": ("P", 0x13), "L outer": ("L", 0x0F),
    "R inner": ("O", 0x12), "R outer": ("K", 0x0E),
    "L SAX": ("I", 0x0C), "R SAX": ("J", 0x0D),
    "G1": ("U", 0x18), "G2": ("H", 0x0B), "G3": ("V", 0x19),
    "G4": ("B", 0x05), "G5": ("N", 0x11),
}


class VendorSession:
    """Omega FF42 protocol, verified on USB PC PID 3A28.

    Recovered from SCUF's official pairing-tool firmware (2.18.431):
    https://customercare.scufgaming.com/hc/en-us/article_attachments/49489215970193
    Command 59 subscribes to notification 1B (accel then gyro, six int16).
    Commands 77/78 read/write the touch acquisition switch.
    """

    def __init__(self, device):
        self.device = device
        self.session = 0
        self.imu_enabled = False
        self.restore_touch = None
        self.pending = []

    @staticmethod
    def encode(data):
        out = bytearray([0])
        index, count = 0, 1
        for value in data:
            if value == 0:
                out[index] = count
                index, count = len(out), 1
                out.append(0)
            else:
                out.append(value)
                count += 1
                if count == 255:
                    out[index] = count
                    index, count = len(out), 1
                    out.append(0)
        out[index] = count
        return bytes(out)

    @staticmethod
    def unpack(report):
        if len(report) < 7 or report[0] & 3:
            raise RuntimeError(f"Unsupported vendor packet: {report.hex()}")
        data = report[2:report.index(0, 2)]
        out = bytearray(report[:2])
        index = 0
        while index < len(data):
            count = data[index]
            if not count or index + count > len(data):
                raise RuntimeError(f"Invalid vendor COBS: {report.hex()}")
            out.extend(data[index+1:index+count])
            index += count
            if count < 255 and index < len(data):
                out.append(0)
        if len(out) < 6 or binascii.crc_hqx(out[:-2], 65535) != int.from_bytes(out[-2:], "little"):
            raise RuntimeError(f"Vendor CRC failure: {report.hex()}")
        return bytes(out[:-2])

    def request(self, command, payload=b""):
        if command not in (0, 1, 0x59, 0x77, 0x78):
            raise RuntimeError(f"Unapproved vendor command: {command:02x}")
        header = bytes([0x88, 1 | self.session << 4])
        data = header + struct.pack("<H", command) + payload
        crc = struct.pack("<H", binascii.crc_hqx(data, 65535))
        wire = header + self.encode(data[2:] + crc) + b"\0"
        if len(wire) > 64:
            raise RuntimeError("Vendor command exceeds HID packet size")
        # macOS requires the full 64-byte report plus HIDAPI's zero report ID.
        if self.device.write(b"\0" + wire.ljust(64, b"\0")) != 65:
            raise RuntimeError("Short vendor HID write")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            raw = bytes(self.device.read(64))
            if not raw:
                time.sleep(.002)
                continue
            packet = self.unpack(raw)
            if packet[1] & 15 == 3:
                self.pending.append(raw)
                continue
            if (len(packet) < 5 or packet[1] & 15 != 2
                    or int.from_bytes(packet[2:4], "little") != command):
                raise RuntimeError(f"Unexpected vendor response: {packet.hex()}")
            if packet[4]:
                raise RuntimeError(f"Command {command:02x} rejected: status {packet[4]:02x}")
            return packet[5:]
        raise RuntimeError(f"Vendor command {command:02x} timed out")

    def start(self):
        response = self.request(0, bytes.fromhex("78563412"))
        if len(response) != 5 or not 1 <= response[0] <= 15:
            raise RuntimeError(f"Invalid session response: {response.hex()}")
        self.session = response[0]
        original = self.request(0x77)
        if original not in (b"\0", b"\1"):
            raise RuntimeError(f"Invalid touch setting: {original.hex()}")
        if original == b"\0":
            self.restore_touch = original
            self.request(0x78, b"\1")
        if self.request(0x77) != b"\1":
            raise RuntimeError("Touch enable readback failed")
        self.imu_enabled = True
        self.request(0x59, b"\1")
        print("PC IMU subscribed; touch enabled and verified", flush=True)

    def close(self):
        errors = []
        for name in ("imu", "touch", "session"):
            try:
                if name == "imu" and self.imu_enabled:
                    self.request(0x59, b"\0")
                    self.imu_enabled = False
                elif name == "touch" and self.restore_touch is not None:
                    self.request(0x78, self.restore_touch)
                    if self.request(0x77) != self.restore_touch:
                        raise RuntimeError("Touch restoration readback failed")
                    self.restore_touch = None
                elif name == "session" and self.session:
                    self.request(1, bytes([self.session]))
                    self.session = 0
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        if errors:
            raise RuntimeError("Vendor cleanup failed: " + "; ".join(errors))
        print("PC subscriptions closed; original touch setting restored", flush=True)


class Monitor:
    def __init__(self):
        self.devices = []
        self.counts, self.latest = {}, {}
        self.keys, self.patches, self.labels = {}, {}, {}
        self.failed = False
        self.usb_device = None
        self.usb_thread = None
        self.usb_stop = threading.Event()
        self.usb_reports = queue.Queue(maxsize=4096)
        self.usb_error = None
        self.vendor = None
        self.vendor_notices = set()
        self.start = time.monotonic()
        # HIDAPI defaults to exclusive access on macOS. Shared access is
        # intentional: keep the system's keyboard/touch interface available.
        if sys.platform == "darwin":
            lib = ctypes.CDLL(hid.__file__)
            lib.hid_init.restype = ctypes.c_int
            if lib.hid_init() != 0:
                raise RuntimeError("HIDAPI initialization failed")
            lib.hid_darwin_set_open_exclusive.argtypes = [ctypes.c_int]
            lib.hid_darwin_set_open_exclusive(0)
        entries = {d["path"]: d for d in hid.enumerate(0x1B1C)
                   if d["product_id"] in (0x3A27, 0x3A28)}
        if len(entries) != 2:
            raise RuntimeError(f"Expected two wired Omega HID paths; found {len(entries)}")
        pids = {d["product_id"] for d in entries.values()}
        if len(pids) != 1:
            raise RuntimeError("Multiple Omega modes/devices detected")
        self.ps = pids == {0x3A27}
        try:
            for path, d in entries.items():
                device = hid.device()
                device.open_path(path)
                self.devices.append((d["interface_number"], device))
                device.set_nonblocking(True)
                self.counts[d["interface_number"]] = 0
            if not self.ps:
                vendor_device = next(device for interface, device in self.devices if interface == 0)
                self.vendor = VendorSession(vendor_device)
                self.vendor.start()
                self.usb_device = libusb_package.find(idVendor=0x1B1C, idProduct=0x3A28)
                if self.usb_device is None:
                    raise RuntimeError("Omega PC USB device is missing")
                interface = self.usb_device.get_active_configuration()[(5, 0)]
                if (interface.bInterfaceClass, interface.bInterfaceSubClass,
                    interface.bInterfaceProtocol) != (0xFF, 0x5D, 1):
                    raise RuntimeError("Unexpected gamepad USB interface descriptor")
                if not any(ep.bEndpointAddress == 0x84 for ep in interface):
                    raise RuntimeError("Gamepad input endpoint 0x84 is missing")
                usb.util.claim_interface(self.usb_device, 5)
                self.counts[5] = 0
                self.usb_thread = threading.Thread(target=self.read_gamepad, daemon=True)
                self.usb_thread.start()
            self.build_ui()
        except Exception:
            self.close()
            raise
        self.timer = self.fig.canvas.new_timer(interval=30)
        self.timer.add_callback(self.tick)
        self.fig.canvas.mpl_connect("close_event", self.close)
        self.timer.start()
        print(f"READY: {'PS' if self.ps else 'PC'} mode; shared HID access; "
              f"interfaces {list(self.counts)}", flush=True)

    def read_gamepad(self):
        """Read the non-HID Xbox 360 interface without blocking Matplotlib."""
        try:
            while not self.usb_stop.is_set():
                try:
                    report = bytes(self.usb_device.read(0x84, 32, timeout=100))
                except usb.core.USBTimeoutError:
                    continue  # No packet during this interval; no fabricated state.
                self.usb_reports.put_nowait(report)
        except Exception as exc:
            self.usb_error = exc

    def button(self, name, x, y, w=1.15, unknown=False):
        patch = FancyBboxPatch((x, y), w, .52, boxstyle="round,pad=0.035",
                              facecolor=PANEL if unknown else IDLE,
                              edgecolor=UNKNOWN if unknown else "#546780",
                              linewidth=1.1)
        self.ax.add_patch(patch)
        label = self.ax.text(x+w/2, y+.26, name + (" ?" if unknown else ""),
                             ha="center", va="center", fontsize=10,
                             color=UNKNOWN if unknown else TEXT)
        self.patches[name], self.labels[name] = patch, label

    def build_ui(self):
        self.fig, self.ax = plt.subplots(figsize=(14, 9), facecolor=BG)
        self.fig.canvas.manager.set_window_title("Omega - raw HID monitor")
        self.fig.subplots_adjust(left=.025, right=.985, top=.98, bottom=.025)
        self.ax.set(xlim=(0, 14), ylim=(0, 10), facecolor=BG)
        self.ax.axis("off")
        mode = "PS / 1B1C:3A27" if self.ps else "PC / 1B1C:3A28"
        self.ax.text(.2, 9.65, f"OMEGA    {mode}", color=TEXT, fontsize=20, weight="bold")
        self.status = self.ax.text(.2, 9.25, "Opening inputs...", color=TEXT, fontsize=11)
        self.ax.text(.2, 8.88, "Green = observed pressed state    Amber ? = independent input not decoded",
                     color=UNKNOWN, fontsize=11)
        base = [("L2", .5, 8.1), ("L1", .5, 7.4), ("R2", 11.9, 8.1), ("R1", 11.9, 7.4),
                ("Up", 2, 7.5), ("Left", .7, 6.85), ("Right", 3.3, 6.85), ("Down", 2, 6.2),
                ("Triangle", 9.5, 7.5), ("Square", 8.2, 6.85), ("Circle", 10.8, 6.85),
                ("Cross", 9.5, 6.2), ("Create", 4.9, 8.1), ("Options", 7.2, 8.1),
                ("PS", 6.1, 6.15), ("L3", 3.9, 5.05), ("R3", 8.9, 5.05)]
        for name, x, y in base:
            self.button(name, x, y)
        self.trigger_bars = {}
        for name, x in (("L2", 1.9), ("R2", 9.1)):
            self.ax.add_patch(Rectangle((x, 8.12), 2.45, .18,
                                        facecolor=PANEL, edgecolor="#546780"))
            fill = Rectangle((x, 8.12), 0, .18, facecolor=ACTIVE)
            self.ax.add_patch(fill)
            label = self.ax.text(x+1.225, 8.48,
                                 "Waiting for report",
                                 ha="center", va="center", color=TEXT,
                                 fontsize=10, family="monospace")
            self.trigger_bars[name] = (fill, label)
        self.button("Touch click", 5.7, 7.15, 2)
        self.sticks = []
        for x in (4.45, 9.45):
            self.ax.add_patch(Circle((x, 5.95), .46, facecolor=PANEL, edgecolor="#546780"))
            dot = Circle((x, 5.95), .09, color=ACTIVE)
            self.ax.add_patch(dot)
            self.sticks.append((x, dot))
        self.ax.add_patch(Rectangle((5.5, 7.85), 2.5, .7, facecolor=PANEL, edgecolor="#546780"))
        self.touch_dots = []
        for color in (ACTIVE, "#72bbff"):
            dot = Circle((0, 0), .07, color=color, visible=False)
            self.ax.add_patch(dot)
            self.touch_dots.append(dot)
        for j, name in enumerate(EXTRAS):
            self.button(name, .5+j*2.2, 4.15, 1.8, unknown=self.ps)
        for j, name in enumerate(G_KEYS):
            self.button(name, .5+j*1.5, 3.35, unknown=self.ps)
        if not self.ps:
            for name, (letter, _) in EXTRA_KEYBOARD.items():
                self.labels[name].set_text(f"{name} [{letter}]")
        self.motion = self.ax.text(.3, 2.85, "IMU: waiting for sensor report",
                                   color=TEXT, fontsize=10, family="monospace")
        self.touch_status = self.ax.text(5.2, 5.62, "Touch: waiting", color=TEXT,
                                         fontsize=9, ha="center")
        self.keyboard = self.ax.text(.3, 2.5, "Keyboard usages: --", color=TEXT, fontsize=10)
        self.raw = self.ax.text(.3, 2.15, "", va="top", color="#a9bbd1", fontsize=9,
                                family="monospace", linespacing=1.5)
        self.ax.text(.3, .22,
                     ("PS: extra-key aliases light their reported standard button; physical origin unknown.\n"
                      if self.ps else "PC: extra buttons follow your SCUF App keyboard mappings shown in brackets.\n")
                     + "PC: temporary IMU/touch enable; restored on close. Switch mode only after closing.",
                     color=UNKNOWN, fontsize=10)

    def set_key(self, key, down):
        self.patches[key].set_facecolor(ACTIVE if down else IDLE)
        self.labels[key].set_color(BG if down else TEXT)

    def touch(self, points):
        for dot, (active, x, y, xmax, ymax) in zip(self.touch_dots, points):
            dot.set_visible(active)
            dot.center = (5.5+2.5*x/xmax, 7.85+.7*(1-y/ymax))

    def triggers(self, left, right):
        for name, value in (("L2", left), ("R2", right)):
            fill, label = self.trigger_bars[name]
            fill.set_width(2.45 * value / 255)
            label.set_text(f"{value:3d}/255   {value / 255:6.1%}")

    def decode_gamepad(self, r):
        # Xbox 360 wired report layout, as documented by Linux xpad:
        # https://github.com/torvalds/linux/blob/master/drivers/input/joystick/xpad.c
        if len(r) != 20 or r[:2] != b"\x00\x14":
            raise RuntimeError(f"Unexpected PC gamepad report: {r.hex()}")
        for bit, key in enumerate(("Up", "Down", "Left", "Right", "Options", "Create", "L3", "R3")):
            self.set_key(key, bool(r[2] & (1 << bit)))
        for bit, key in ((0, "L1"), (1, "R1"), (2, "PS"), (4, "Cross"),
                         (5, "Circle"), (6, "Square"), (7, "Triangle")):
            self.set_key(key, bool(r[3] & (1 << bit)))
        self.triggers(r[4], r[5])
        self.set_key("L2", r[4] > 0)
        self.set_key("R2", r[5] > 0)
        axes = struct.unpack_from("<4h", r, 6)
        for j, (x, dot) in enumerate(self.sticks):
            sx, sy = axes[2*j:2*j+2]
            dot.center = (x+.36*sx/(32767 if sx >= 0 else 32768),
                          5.95+.36*sy/(32767 if sy >= 0 else 32768))

    def decode(self, interface, r):
        if self.ps and interface == 0:
            if len(r) != 64 or r[0] != 1:
                raise RuntimeError(f"Unexpected PS input: {r.hex()}")
            self.triggers(r[5], r[6])
            names = ("Square", "Cross", "Circle", "Triangle")
            for bit, key in enumerate(names, 4):
                self.set_key(key, bool(r[8] & (1 << bit)))
            for bit, key in enumerate(("L1", "R1", "L2", "R2", "Create", "Options", "L3", "R3")):
                down = bool(r[9] & (1 << bit))
                if key in ("L2", "R2"):
                    down |= r[5 if key == "L2" else 6] > 0
                self.set_key(key, down)
            hat = r[8] & 15
            for key, positions in (("Up", (7, 0, 1)), ("Right", (1, 2, 3)),
                                   ("Down", (3, 4, 5)), ("Left", (5, 6, 7))):
                self.set_key(key, hat in positions)
            self.set_key("PS", bool(r[10] & 1))
            self.set_key("Touch click", bool(r[10] & 2))
            for j, (x, dot) in enumerate(self.sticks):
                dot.center = (x+.36*(r[1+j*2]-127.5)/127.5,
                              5.95-.36*(r[2+j*2]-127.5)/127.5)
            points = []
            for p in (32, 36):
                points.append((not bool(r[p] & 128), r[p+1] | ((r[p+2] & 15) << 8),
                               (r[p+2] >> 4) | (r[p+3] << 4), 1919, 1079))
            self.touch(points)
            self.touch_status.set_text(f"Touch: {sum(p[0] for p in points)} contacts")
            values = struct.unpack_from("<6h", r, 16)
            self.motion.set_text(f"IMU raw int16   gyro {values[:3]}   accel {values[3:]}")
        elif not self.ps and interface == 0:
            packet = self.vendor.unpack(r)
            if packet[1] & 15 != 3 or packet[1] >> 4 != self.vendor.session:
                raise RuntimeError(f"Unexpected vendor notification: {packet.hex()}")
            ident = int.from_bytes(packet[2:4], "little")
            if ident == 0x1B:
                if len(packet) != 16:
                    raise RuntimeError(f"Invalid IMU payload: {packet.hex()}")
                values = struct.unpack_from("<6h", packet, 4)
                self.motion.set_text(f"IMU raw int16   gyro {values[3:]}   accel {values[:3]}")
            elif ident not in self.vendor_notices:
                self.vendor_notices.add(ident)
                print(f"Other vendor notification 0x{ident:02x}, raw: {packet.hex()}", flush=True)
        elif not self.ps and interface == 1:
            if r[0] == 2 and len(r) == 15:
                # Actual firmware coordinates reach 1079 despite descriptor Y=900.
                points = [(bool(r[p] & 2), *struct.unpack_from("<HH", r, p+1), 1919, 1079)
                          for p in (1, 6)]
                self.touch(points)
                self.set_key("Touch click", bool(r[14] & 1))
                self.touch_status.set_text(f"Touch: {r[13]} contacts; XY {points[0][1:3]}")
            elif r[0] == 1 and len(r) == 17:
                # Descriptor: 8 modifier bits, then 15 one-byte keyboard usages.
                pressed = set(r[2:]) - {0}
                if pressed & {1, 2, 3}:
                    raise RuntimeError(f"Keyboard rollover/error report: {r.hex()}")
                for name, (_, usage) in EXTRA_KEYBOARD.items():
                    self.set_key(name, usage in pressed)
                self.keyboard.set_text(f"Keyboard modifier=0x{r[1]:02x}  usages="
                                       + str([f"0x{x:02x}" for x in sorted(pressed)]))
            elif r[0] == 3 and len(r) == 2:
                # Consumer volume reports remain in the raw display only.
                return
            else:
                raise RuntimeError(f"Unknown PC input report: {r.hex()}")
        # Vendor-specific reports remain visible verbatim, with no guessed mapping.

    def tick(self):
        try:
            if self.vendor is not None:
                pending, self.vendor.pending = self.vendor.pending, []
                for report in pending:
                    self.counts[0] += 1
                    self.latest[(0, report[0])] = report
                    self.decode(0, report)
            if self.usb_error is not None:
                raise RuntimeError(f"PC USB reader failed: {self.usb_error}") from self.usb_error
            while not self.usb_reports.empty():
                report = self.usb_reports.get_nowait()
                self.counts[5] += 1
                self.latest[(5, report[0])] = report
                self.decode_gamepad(report)
            for interface, device in self.devices:
                for _ in range(128):
                    report = device.read(256)
                    if not report:
                        break
                    r = bytes(report)
                    self.counts[interface] += 1
                    self.latest[(interface, r[0])] = r
                    self.decode(interface, r)
            elapsed = time.monotonic()-self.start
            self.status.set_text("Raw input  |  " + "  ".join(
                f"IF {i}: {n} reports ({n/max(elapsed,.001):.0f}/s)" for i,n in self.counts.items()))
            lines = []
            for interface in self.counts:
                reports = [(rid, r) for (i, rid), r in sorted(self.latest.items()) if i == interface]
                if not reports:
                    lines.append(f"IF {interface}: no input reports received")
                for rid, r in reports:
                    lines.append(f"IF {interface} report 0x{rid:02x} [{len(r)} bytes]: {r[:32].hex(' ')}")
                    if len(r) > 32:
                        lines.append(" "*28+r[32:].hex(' '))
            self.raw.set_text("\n".join(lines))
            self.fig.canvas.draw_idle()
        except Exception as exc:
            self.failed = True
            traceback.print_exc()
            self.timer.stop()
            try:
                self.close()
            except Exception as cleanup_error:
                traceback.print_exc()
                exc = RuntimeError(f"{exc}; cleanup: {cleanup_error}")
            self.status.set_text(f"FATAL: {exc} -- capture stopped; close window and restart")
            self.status.set_color("#ff6262")
            self.fig.canvas.draw_idle()
            return False
        return True

    def close(self, _event=None):
        if hasattr(self, "timer"):
            self.timer.stop()
        self.usb_stop.set()
        if self.usb_thread is not None:
            self.usb_thread.join(timeout=1)
            if self.usb_thread.is_alive():
                raise RuntimeError("PC USB reader did not stop; device was not released")
            self.usb_thread = None
        if self.usb_device is not None:
            usb.util.dispose_resources(self.usb_device)
            self.usb_device = None
        try:
            if self.vendor is not None:
                vendor, self.vendor = self.vendor, None
                vendor.close()
        finally:
            for _, device in self.devices:
                device.close()
            self.devices.clear()


if __name__ == "__main__":
    # SCUF keyboard mappings must not trigger Matplotlib navigation or scaling.
    # This is process-local; mouse interactions and user configuration stay intact.
    for key in plt.rcParams:
        if key.startswith("keymap."):
            plt.rcParams[key] = []
    monitor = Monitor()
    try:
        plt.show()
    finally:
        monitor.close()
    if monitor.failed:
        raise SystemExit(1)
