import bluetooth
import time
from micropython import const

from ble_protocol import make_weight_packet, parse_bookoo_command

_IRQ_CENTRAL_CONNECT = const(1 << 0)
_IRQ_CENTRAL_DISCONNECT = const(1 << 1)
_IRQ_GATTS_WRITE = const(1 << 2)

# BOOKOO THEMIS MINI
_BOOKOO_SERVICE_UUID = bluetooth.UUID(0x0FFE)
_BOOKOO_WEIGHT_UUID = bluetooth.UUID(0xFF11)
_BOOKOO_COMMAND_UUID = bluetooth.UUID(0xFF12)

_BOOKOO_WEIGHT_FLAGS = bluetooth.FLAG_READ | bluetooth.FLAG_NOTIFY
_BOOKOO_COMMAND_FLAGS = bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE

_BOOKOO_SERVICE = (
    _BOOKOO_SERVICE_UUID,
    (
        (_BOOKOO_WEIGHT_UUID, _BOOKOO_WEIGHT_FLAGS),
        (_BOOKOO_COMMAND_UUID, _BOOKOO_COMMAND_FLAGS),
    ),
)


class BLEScales:

    def __init__(self, ble, name="BOOKOO_SC", command_sink=None):
        self._ble = ble
        self._command_sink = command_sink
        self._ble.active(True)

        print("bt activated")

        self._ble.irq(self._irq)

        (
            (self._weight_handle, self._command_handle),
        ) = self._ble.gatts_register_services(
            (_BOOKOO_SERVICE,)
        )

        self._connections = set()

        self._weight = 0.0
        self._flow = 0.0
        self._battery = 100

        self._payload = self._advertising_payload(
            name
        )

        self._advertise()

        print("BOOKOO BLE ready")
        print("name:", name)
        print("service: 0x0FFE")
        print("weight: 0xFF11")
        print("command: 0xFF12")

    # ---------------------------------------------------------
    # BLE EVENTS
    # ---------------------------------------------------------

    def _irq(self, event, data):

        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)

            print("BOOKOO connected:", conn_handle)

            # Send current weight immediately
            self._notify_weight()

        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data

            if conn_handle in self._connections:
                self._connections.remove(conn_handle)

            print("BOOKOO disconnected:", conn_handle)

            self._advertise()

        elif event == _IRQ_GATTS_WRITE:
            _, value_handle = data
            if value_handle == self._command_handle:
                command = self._ble.gatts_read(self._command_handle)
                self._handle_command(command)

    # ---------------------------------------------------------
    # BOOKOO COMMANDS
    # ---------------------------------------------------------

    def _handle_command(self, command):
        for normalized_event in parse_bookoo_command(command):
            self._enqueue_command(normalized_event)

    def _enqueue_command(self, event):
        if self._command_sink is None:
            return

        push = getattr(self._command_sink, "push", None)
        if push is not None:
            push(event)
        else:
            self._command_sink(event)

    # ---------------------------------------------------------
    # WEIGHT
    # ---------------------------------------------------------

    def set_measurement(self, weight_g, flow_gps, notify=False):
        """Publish controller-owned weight and flow without recalculation."""

        self._weight = float(weight_g)
        self._flow = float(flow_gps)

        packet = self._make_weight_packet()
        self._ble.gatts_write(self._weight_handle, packet)

        if notify:
            self._notify_weight()

    def set_weight(self, weight, notify=False):
        """Compatibility wrapper that preserves the last controller flow."""

        self.set_measurement(weight, self._flow, notify=notify)

    def set_battery(self, battery):
        self._battery = max(0, min(100, int(battery)))

    def _notify_weight(self):

        packet = self._make_weight_packet()

        self._ble.gatts_write(
            self._weight_handle,
            packet
        )

        for conn_handle in self._connections:
            try:
                self._ble.gatts_notify(
                    conn_handle,
                    self._weight_handle,
                    packet
                )
            except Exception as e:
                print("BOOKOO notify error:", e)

    # ---------------------------------------------------------
    # BOOKOO 20-BYTE WEIGHT PACKET
    # ---------------------------------------------------------

    def _make_weight_packet(self):
        ticks_ms = getattr(time, "ticks_ms", None)
        if ticks_ms is not None:
            now_ms = ticks_ms()
        else:
            now_ms = int(time.monotonic() * 1000)
        return make_weight_packet(
            self._weight, self._flow, self._battery, now_ms
        )

    # ---------------------------------------------------------
    # ADVERTISING
    # ---------------------------------------------------------

    def _advertising_payload(self, name):

        # Flags
        payload = bytearray([
            0x02,
            0x01,
            0x06
        ])

        # Complete local name
        name_bytes = name.encode()

        payload += bytes([
            len(name_bytes) + 1,
            0x09
        ])

        payload += name_bytes

        # Complete 16-bit service UUID list
        payload += bytes([
            0x03,
            0x03,
            0xFE,
            0x0F
        ])

        return payload

    def _advertise(self, interval_us=500000):

        self._ble.gap_advertise(
            interval_us,
            adv_data=self._payload
        )

        print("BOOKOO advertising...")
