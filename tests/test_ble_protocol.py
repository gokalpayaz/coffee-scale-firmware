import importlib
import os
import sys
import types
import unittest


LIB_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "firmware", "lib")
)
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)

from app_contracts import (
    EVENT_TARE,
    EVENT_TIMER_RESET,
    EVENT_TIMER_TOGGLE,
    EventQueue,
)
from ble_protocol import make_weight_packet, parse_bookoo_command


class BookooProtocolTests(unittest.TestCase):
    def test_parses_supported_commands_to_normalized_events(self):
        self.assertEqual((EVENT_TARE,), parse_bookoo_command(b"\x03\x0a\x01"))
        self.assertEqual(
            (EVENT_TIMER_TOGGLE,), parse_bookoo_command(b"\x03\x0a\x04")
        )
        self.assertEqual(
            (EVENT_TIMER_TOGGLE,), parse_bookoo_command(b"\x03\x0a\x05")
        )
        self.assertEqual(
            (EVENT_TIMER_RESET,), parse_bookoo_command(b"\x03\x0a\x06")
        )
        self.assertEqual(
            (EVENT_TARE, EVENT_TIMER_TOGGLE),
            parse_bookoo_command(b"\x03\x0a\x07"),
        )

    def test_rejects_invalid_or_unsupported_commands(self):
        self.assertEqual((), parse_bookoo_command(None))
        self.assertEqual((), parse_bookoo_command(b"\x03\x0a"))
        self.assertEqual((), parse_bookoo_command(b"\x00\x0a\x01"))
        self.assertEqual((), parse_bookoo_command(b"\x03\x0a\x08"))

    def test_packet_encodes_passed_weight_flow_battery_and_checksum(self):
        packet = make_weight_packet(-12.34, -1.23, 67, 0x123456)

        self.assertEqual(20, len(packet))
        self.assertEqual(b"\x03\x0b", bytes(packet[:2]))
        self.assertEqual(b"\x12\x34\x56", bytes(packet[2:5]))
        self.assertEqual(0x2D, packet[6])
        self.assertEqual(b"\x00\x04\xd2", bytes(packet[7:10]))
        self.assertEqual(0x2D, packet[10])
        self.assertEqual(b"\x00\x7b", bytes(packet[11:13]))
        self.assertEqual(67, packet[13])
        self.assertEqual(0, sum(packet[14:19]))

        checksum = 0
        for value in packet[:19]:
            checksum ^= value
        self.assertEqual(checksum, packet[19])

    def test_packet_clamps_flow_and_battery(self):
        packet = make_weight_packet(1.0, 1000.0, 120, 0)

        self.assertEqual(b"\x7f\xff", bytes(packet[11:13]))
        self.assertEqual(100, packet[13])


class FakeBluetoothModule(types.ModuleType):
    FLAG_READ = 1
    FLAG_NOTIFY = 2
    FLAG_WRITE = 4
    FLAG_WRITE_NO_RESPONSE = 8

    @staticmethod
    def UUID(value):
        return value


class FakeBLE:
    def __init__(self):
        self.irq_handler = None
        self.command = b""
        self.writes = []
        self.notifications = []
        self.advertisements = []

    def active(self, value):
        self.active_value = value

    def irq(self, handler):
        self.irq_handler = handler

    def gatts_register_services(self, services):
        self.services = services
        return ((11, 12),)

    def gap_advertise(self, interval_us, adv_data):
        self.advertisements.append((interval_us, adv_data))

    def gatts_read(self, handle):
        return self.command

    def gatts_write(self, handle, packet):
        self.writes.append((handle, bytes(packet)))

    def gatts_notify(self, conn_handle, handle, packet):
        self.notifications.append((conn_handle, handle, bytes(packet)))


class BLEScalesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original_bluetooth = sys.modules.get("bluetooth")
        cls.original_micropython = sys.modules.get("micropython")

        bluetooth = FakeBluetoothModule("bluetooth")
        micropython = types.ModuleType("micropython")
        micropython.const = lambda value: value
        sys.modules["bluetooth"] = bluetooth
        sys.modules["micropython"] = micropython
        sys.modules.pop("ble_scales", None)
        cls.ble_scales = importlib.import_module("ble_scales")

    @classmethod
    def tearDownClass(cls):
        sys.modules.pop("ble_scales", None)
        if cls.original_bluetooth is None:
            sys.modules.pop("bluetooth", None)
        else:
            sys.modules["bluetooth"] = cls.original_bluetooth
        if cls.original_micropython is None:
            sys.modules.pop("micropython", None)
        else:
            sys.modules["micropython"] = cls.original_micropython

    def test_irq_enqueues_commands_in_order_and_ignores_invalid_handle(self):
        ble = FakeBLE()
        queue = EventQueue()
        scales = self.ble_scales.BLEScales(ble, command_sink=queue)
        ble.command = b"\x03\x0a\x07"

        scales._irq(self.ble_scales._IRQ_GATTS_WRITE, (1, 99))
        self.assertEqual(0, len(queue))

        scales._irq(self.ble_scales._IRQ_GATTS_WRITE, (1, 12))
        self.assertEqual(EVENT_TARE, queue.pop())
        self.assertEqual(EVENT_TIMER_TOGGLE, queue.pop())

    def test_set_measurement_uses_controller_flow_without_recalculation(self):
        ble = FakeBLE()
        scales = self.ble_scales.BLEScales(ble)

        scales.set_measurement(10.0, -2.5)
        first_packet = ble.writes[-1][1]
        scales.set_measurement(20.0, -2.5)
        second_packet = ble.writes[-1][1]

        self.assertEqual(0x2D, first_packet[10])
        self.assertEqual(b"\x00\xfa", first_packet[11:13])
        self.assertEqual(first_packet[10:13], second_packet[10:13])

    def test_advertising_and_notifications_are_preserved(self):
        ble = FakeBLE()
        scales = self.ble_scales.BLEScales(ble, name="BOOKOO_SC")

        self.assertEqual(500000, ble.advertisements[-1][0])
        self.assertIn(b"BOOKOO_SC", bytes(ble.advertisements[-1][1]))

        scales.set_measurement(3.5, 0.4)
        scales._irq(self.ble_scales._IRQ_CENTRAL_CONNECT, (7, None, None))
        self.assertEqual(7, ble.notifications[-1][0])


if __name__ == "__main__":
    unittest.main()
