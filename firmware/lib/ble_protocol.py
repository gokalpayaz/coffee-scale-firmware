"""Pure helpers for the Bookoo-compatible BLE protocol."""

from app_contracts import (
    EVENT_TARE,
    EVENT_TIMER_RESET,
    EVENT_TIMER_START,
    EVENT_TIMER_STOP,
)


def parse_bookoo_command(command):
    """Return normalized events for a Bookoo command packet."""

    if command is None or len(command) < 3:
        return ()
    if command[0] != 0x03 or command[1] != 0x0A:
        return ()

    command_type = command[2]
    if command_type == 0x01:
        return (EVENT_TARE,)
    if command_type == 0x04:
        return (EVENT_TIMER_START,)
    if command_type == 0x05:
        return (EVENT_TIMER_STOP,)
    if command_type == 0x06:
        return (EVENT_TIMER_RESET,)
    if command_type == 0x07:
        return (EVENT_TARE, EVENT_TIMER_START)
    return ()


def make_weight_packet(weight_g, flow_gps, battery_percent, now_ms):
    """Build the 20-byte Bookoo weight notification packet."""

    weight_g = float(weight_g)
    flow_gps = float(flow_gps)
    battery_percent = max(0, min(100, int(battery_percent)))

    weight_int = int(round(abs(weight_g) * 100)) & 0xFFFFFF
    flow_int = int(round(abs(flow_gps) * 100))
    if flow_int > 32767:
        flow_int = 32767

    packet = bytearray(20)
    timestamp = int(now_ms) & 0xFFFFFF

    packet[0] = 0x03
    packet[1] = 0x0B
    packet[2] = (timestamp >> 16) & 0xFF
    packet[3] = (timestamp >> 8) & 0xFF
    packet[4] = timestamp & 0xFF
    packet[5] = 0x00
    packet[6] = 0x2D if weight_g < 0 else 0x2B
    packet[7] = (weight_int >> 16) & 0xFF
    packet[8] = (weight_int >> 8) & 0xFF
    packet[9] = weight_int & 0xFF
    packet[10] = 0x2D if flow_gps < 0 else 0x2B
    packet[11] = (flow_int >> 8) & 0xFF
    packet[12] = flow_int & 0xFF
    packet[13] = battery_percent
    # Standby, buzzer, smoothing, and reserved bytes remain zero.

    checksum = 0
    for value in packet[:19]:
        checksum ^= value
    packet[19] = checksum

    return packet
