import struct
import bluetooth
import time
from micropython import const

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

    def __init__(self, ble, name="BOOKOO_SC"):
        self._ble = ble
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
        self._last_weight = 0.0
        self._last_weight_time = time.ticks_ms()
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
            conn_handle, value_handle = data
            
            print("BOOKOO CMD:", " ".join("{:02X}".format(b) for b in data))
            
            if value_handle == self._command_handle:
                command = self._ble.gatts_read(self._command_handle)

                print(
                    "BOOKOO command:",
                    " ".join("{:02X}".format(x) for x in command)
                )

                self._handle_command(command)

    # ---------------------------------------------------------
    # BOOKOO COMMANDS
    # ---------------------------------------------------------

    def _handle_command(self, command):

        if len(command) < 3:
            return

        # 03 0A xx ...
        if command[0] != 0x03 or command[1] != 0x0A:
            return

        command_type = command[2]

        if command_type == 0x01:
            # TARE
            print("BOOKOO: tare")

        elif command_type == 0x04:
            # START TIMER
            print("BOOKOO: start timer")

        elif command_type == 0x05:
            # STOP TIMER
            print("BOOKOO: stop timer")

        elif command_type == 0x06:
            # RESET TIMER
            print("BOOKOO: reset timer")

        elif command_type == 0x07:
            # TARE + START TIMER
            print("BOOKOO: tare + start timer")

        elif command_type == 0x08:
            # FLOW SMOOTHING
            print("BOOKOO: flow smoothing command")

    # ---------------------------------------------------------
    # WEIGHT
    # ---------------------------------------------------------

    def set_weight(self, weight, notify=False):

        self._weight = float(weight)

        # Store the value in the characteristic too
        packet = self._make_weight_packet()

        self._ble.gatts_write(
            self._weight_handle,
            packet
        )

        if notify:
            self._notify_weight()
    
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

        now = time.ticks_ms() & 0xFFFFFF

        weight = self._weight

        # Bookoo uses weight * 100
        weight_int = int(round(abs(weight) * 100))

        if weight < 0:
            sign = 0x2D       # '-'
        else:
            sign = 0x2B       # '+'

        # Calculate flow rate
        current_time = time.ticks_ms()

        dt = time.ticks_diff(
            current_time,
            self._last_weight_time
        )

        if dt > 0:
            flow = (
                (weight - self._last_weight)
                / (dt / 1000.0)
            )
        else:
            flow = 0.0

        self._last_weight = weight
        self._last_weight_time = current_time

        flow_int = int(round(abs(flow) * 100))

        if flow_int > 32767:
            flow_int = 32767

        # 24-bit weight
        weight_int &= 0xFFFFFF

        # 16-bit flow
        flow_int &= 0xFFFF

        packet = bytearray(20)

        # Header
        packet[0] = 0x03
        packet[1] = 0x0B

        # milliseconds, 24-bit
        packet[2] = (now >> 16) & 0xFF
        packet[3] = (now >> 8) & 0xFF
        packet[4] = now & 0xFF

        # Unit
        # Bookoo currently supports grams
        packet[5] = 0x00

        # Weight sign
        packet[6] = sign

        # Weight * 100, 24-bit
        packet[7] = (weight_int >> 16) & 0xFF
        packet[8] = (weight_int >> 8) & 0xFF
        packet[9] = weight_int & 0xFF

        # Flow sign
        packet[10] = 0x2B if flow >= 0 else 0x2D

        # Flow * 100
        packet[11] = (flow_int >> 8) & 0xFF
        packet[12] = flow_int & 0xFF

        # Battery percentage
        packet[13] = self._battery

        # Standby time * 10
        packet[14] = 0
        packet[15] = 0

        # Buzzer
        packet[16] = 0

        # Flow smoothing
        packet[17] = 0

        # Reserved
        packet[18] = 0

        # XOR checksum
        checksum = 0

        for i in range(19):
            checksum ^= packet[i]

        packet[19] = checksum

        return packet

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