#!/usr/bin/env python3
"""Direct teleop for an ESP32 OpenBot - no phone required.

Speaks the serial protocol implemented by firmware/openbot/openbot.ino, over USB or BLE:

    c<left>,<right>\\n   drive; both values in [-255, 255], negative = reverse, 0 = stop
    h<ms>\\n             heartbeat; the firmware stops the motors when it is not refreshed
    f\\n                 report robot type and feature list
    w<ms>\\n             interval for wheel odometry messages
    v<ms>\\n             interval for voltage messages

Telemetry sent by the bot: v<volts>, w<left_rpm>,<right_rpm>, s<cm>, b<collision>.

    python teleop.py --port COM5   # USB serial
    python teleop.py --ble         # Bluetooth, advertises as "OpenBot: DIY_ESP32"
    python teleop.py --list        # show serial ports

Keys: w/s forward/back, a/d spin left/right, q/e forward arc, z/c backward arc,
space stop, +/- speed, x quit (also stops the motors).
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import time

TX_PERIOD = 0.05     # control frames per second -> 20 Hz
HEARTBEAT_PERIOD = 0.2
HEARTBEAT_MS = 500   # firmware stops the motors 500 ms after the last heartbeat
WHEEL_MS = 200       # ask for wheel odometry 5x per second
SPEED_STEP = 16
MAX_SPEED = 255
SERVICE_UUID = "61653dc3-4021-4d1e-ba83-8b4eec61d613"
RX_UUID = "06386c14-86ea-4d71-811c-48f97c58f8c9"  # host -> robot (write without response)
TX_UUID = "9bf1103b-834c-47cf-b149-c9e4bcf778a7"  # robot -> host (notify)

# Wheel setpoints as a fraction of the current speed: (left, right).
PATTERNS = {
    "forward": (1, 1),
    "back": (-1, -1),
    "left": (-1, 1),          # spin in place
    "right": (1, -1),
    "arc_left": (0.5, 1),
    "arc_right": (1, 0.5),
    "arc_back_left": (-0.5, -1),
    "arc_back_right": (-1, -0.5),
    "stop": (0, 0),
}


def _clamp(value: float) -> int:
    return max(-MAX_SPEED, min(int(round(value)), MAX_SPEED))


class Vehicle:
    """Differential drive state: key names in, c<left>,<right> frames out."""

    def __init__(self, speed: int = 128):
        self.speed = max(1, min(speed, MAX_SPEED))
        self.pattern = PATTERNS["stop"]

    def apply(self, key: str) -> None:
        """Update the drive setpoints for a key name (unknown names are ignored)."""
        if key == "faster":
            self.speed = min(self.speed + SPEED_STEP, MAX_SPEED)
        elif key == "slower":
            self.speed = max(self.speed - SPEED_STEP, 1)
        elif key in PATTERNS:
            self.pattern = PATTERNS[key]

    @property
    def left(self) -> int:
        return _clamp(self.pattern[0] * self.speed)

    @property
    def right(self) -> int:
        return _clamp(self.pattern[1] * self.speed)

    @property
    def frame(self) -> str:
        return f"c{self.left},{self.right}\n"


class SerialLink:
    """USB serial transport."""

    def __init__(self, port: str, baud: int):
        import serial  # pyserial

        self.ser = serial.Serial(port, baud, timeout=0)
        self._buffer = b""

    async def start(self) -> None:
        # Opening the port toggles DTR/RTS and resets the ESP32; wait for it to boot.
        await asyncio.sleep(2.0)
        self.ser.reset_input_buffer()

    async def send(self, text: str) -> None:
        self.ser.write(text.encode("ascii"))

    def read_lines(self) -> list[str]:
        self._buffer += self.ser.read(4096) or b""
        lines = self._buffer.split(b"\n")
        self._buffer = lines.pop()
        return [line.decode("ascii", "replace").strip() for line in lines]

    async def close(self) -> None:
        self.ser.close()


class BleLink:
    """Bluetooth LE transport using the UART service of the firmware."""

    def __init__(self, name: str):
        self.name = name
        self.client = None
        self._lines: list[str] = []
        self._buffer = ""

    async def start(self) -> None:
        from bleak import BleakClient, BleakScanner

        def matches(device, adv) -> bool:
            return (adv.local_name or device.name or "").startswith(self.name) or SERVICE_UUID in (adv.service_uuids or [])

        device = await BleakScanner.find_device_by_filter(matches, timeout=10.0)
        if device is None:
            raise RuntimeError(f'no BLE device advertising "{self.name}" found')
        self.client = BleakClient(device)
        await self.client.connect()
        await self.client.start_notify(TX_UUID, self._on_notify)
        print(f"connected to {device.name}")

    def _on_notify(self, _sender, data: bytearray) -> None:
        self._buffer += data.decode("ascii", "replace")
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            self._lines.append(line.strip())

    async def send(self, text: str) -> None:
        await self.client.write_gatt_char(RX_UUID, text.encode("ascii"), response=False)

    def read_lines(self) -> list[str]:
        lines, self._lines = self._lines, []
        return lines

    async def close(self) -> None:
        if self.client is not None and self.client.is_connected:
            await self.client.stop_notify(TX_UUID)
            await self.client.disconnect()


class Keys:
    """Single key presses from the terminal, without echo and without Enter."""

    def __init__(self):
        self.windows = sys.platform.startswith("win")
        self._saved = None

    def __enter__(self):
        if not self.windows:
            import termios
            import tty

            self._termios, self._tty = termios, tty
            self._saved = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *_exc):
        if not self.windows and self._saved is not None:
            self._termios.tcsetattr(sys.stdin.fileno(), self._termios.TCSADRAIN, self._saved)

    def poll(self) -> list[str]:
        if self.windows:
            import msvcrt

            keys = []
            while msvcrt.kbhit():
                char = msvcrt.getch()
                if char in (b"\x00", b"\xe0"):  # function/arrow key prefix
                    arrow = msvcrt.getch().decode("ascii", "replace")
                    keys.append({"H": "w", "P": "s", "K": "a", "M": "d"}.get(arrow, ""))
                else:
                    keys.append(char.decode("ascii", "replace"))
            return [key for key in keys if key]
        import select

        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            keys.append(sys.stdin.read(1))
        return [key for key in keys if key]


KEYMAP = {
    "w": "forward", "s": "back", "a": "left", "d": "right",
    "q": "arc_left", "e": "arc_right", "z": "arc_back_left", "c": "arc_back_right",
    " ": "stop", "+": "faster", "=": "faster", "-": "slower",
}
QUIT_KEYS = ("x", "\x03", "\x1b")  # x, Ctrl-C, Esc


async def run(link, args) -> int:
    vehicle = Vehicle(args.speed)
    await link.start()
    await link.send("f\n")
    await link.send(f"w{WHEEL_MS}\n")
    await link.send(f"h{HEARTBEAT_MS}\n")

    print(f"speed {vehicle.speed}  |  w/a/s/d drive, q/e/z/c arc, space stop, +/- speed, x quit")
    last_tx = last_hb = 0.0
    with Keys() as keys:
        while True:
            for line in link.read_lines():
                if line and line[0] in "vwsb":
                    print(f"\r{line:<40}", end="", flush=True)
            now = time.monotonic()
            for key in keys.poll():
                if key in QUIT_KEYS:
                    await link.send("c0,0\n")
                    print("\nstopped")
                    return 0
                before = vehicle.frame
                vehicle.apply(KEYMAP.get(key, ""))
                if vehicle.frame != before:
                    last_tx = 0.0  # send the change immediately
            if now - last_tx >= TX_PERIOD:
                await link.send(vehicle.frame)
                last_tx = now
            if now - last_hb >= HEARTBEAT_PERIOD:
                await link.send(f"h{HEARTBEAT_MS}\n")
                last_hb = now
            await asyncio.sleep(0.005)


def list_ports() -> int:
    from serial.tools import list_ports as lp

    ports = list(lp.comports())
    if not ports:
        print("no serial ports found")
    for port in ports:
        print(f"{port.device}  {port.description}")
    return 0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Teleop an ESP32 OpenBot over USB or BLE.")
    parser.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate (default 115200)")
    parser.add_argument("--ble", action="store_true", help="use Bluetooth LE instead of USB serial")
    parser.add_argument("--name", default="OpenBot", help='BLE name prefix to connect to (default "OpenBot")')
    parser.add_argument("--speed", type=int, default=128, help="initial speed 1-255 (default 128)")
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.list:
        return list_ports()
    if args.ble:
        link = BleLink(args.name)
    else:
        port = args.port
        if not port:
            from serial.tools import list_ports as lp

            found = [p.device for p in lp.comports()]
            if len(found) != 1:
                print(f"specify --port; found: {found or 'nothing'}", file=sys.stderr)
                return 2
            port = found[0]
        link = SerialLink(port, args.baud)
    try:
        return asyncio.run(_run_and_close(link, args))
    except KeyboardInterrupt:
        return 0


async def _run_and_close(link, args) -> int:
    try:
        return await run(link, args)
    except BaseException:
        # Best effort: keep the robot from rolling on if the link dies or we are interrupted.
        try:
            await link.send("c0,0\n")
        except Exception:
            pass
        raise
    finally:
        await link.close()


if __name__ == "__main__":
    sys.exit(main())
