# robot_uart.py
# UART Protokoll-Library fuer den Pololu 3pi+ 2040
# Sendet und empfaengt strukturierte Pakete mit Magic Bytes (0xAA 0x55) und CRC-8
# Enthaelt einen internen Debug-Screen der vom ESP32 dynamisch befuellt wird

from machine import UART
import time
import struct

# Pakettypen (applikation)
MSG_SPEED    = 0x01
MSG_DEBUG    = 0x02
MSG_METRICS  = 0x03
MSG_ROBOT_ID = 0x04

# Pakettypen (debug-protokoll, bidirektional)
MSG_PING        = 0x20
MSG_DEBUG_REG   = 0x22
MSG_DEBUG_DATA  = 0x23

# DEBUG_REG display_type
DBG_METRIC  = 0x01
DBG_GRAPH   = 0x02
DBG_LOG     = 0x03

# DEBUG_DATA value_type
DBG_FLOAT32 = 0x01
DBG_INT8    = 0x02
DBG_INT16   = 0x03
DBG_STRING  = 0x04

# Heartbeat-Timeout
_HEARTBEAT_TIMEOUT_MS = 5000

# Frame-Konstanten
_MAGIC_0 = 0xAA
_MAGIC_1 = 0x55


def _make_crc8_table():
    t = bytearray(256)
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
        t[i] = crc
    return bytes(t)

_CRC8_TABLE = _make_crc8_table()

def _crc8(data) -> int:
    """CRC-8 via Lookup-Table (Polynom 0x07)."""
    crc = 0
    tbl = _CRC8_TABLE
    for b in data:
        crc = tbl[crc ^ b]
    return crc

def _crc8_buf(buf, start: int, length: int) -> int:
    """CRC-8 direkt auf bytearray mit Offset – keine Kopie noetig."""
    crc = 0
    tbl = _CRC8_TABLE
    end = start + length
    for i in range(start, end):
        crc = tbl[crc ^ buf[i]]
    return crc


def build_packet(msg_type: int, payload: bytes) -> bytearray:
    """
    Verpackt Nutzdaten in einen vollstaendigen Frame.
    Frame: [0xAA][0x55][type][len][payload...][crc8]
    """
    header = bytes([_MAGIC_0, _MAGIC_1, msg_type, len(payload)])
    crc    = _crc8(bytes([msg_type, len(payload)]) + payload)
    return bytearray(header + payload + bytes([crc]))


class Packet:
    """Repraesentiert ein empfangenes Paket."""
    __slots__ = ('msg_type', 'payload', 'valid')

    def __init__(self, msg_type: int, payload: bytes, valid: bool):
        self.msg_type = msg_type
        self.payload  = payload
        self.valid    = valid

    def __repr__(self):
        return f"Packet(type=0x{self.msg_type:02x}, len={len(self.payload)}, {'OK' if self.valid else 'CRC ERR'})"


class UARTProtocol:
    """
    Verwaltet das Senden und Empfangen von strukturierten UART-Paketen.

    MSG_SPEED Payload (2 Bytes):
        [left_signed_int8][right_signed_int8]
        Wertebereich: -127..+127, Vorzeichen = Richtung

    Verwendung:
        proto = UARTProtocol(uart, screen_manager=mgr,
                             on_packet=my_callback, on_raw=my_raw_callback)
        while True:
            proto.loop()
    """

    def __init__(self, uart: UART, screen_manager=None, on_packet=None, on_raw=None, on_robot_id=None):
        self._uart           = uart
        self._on_packet      = on_packet
        self._on_raw         = on_raw
        self._on_robot_id    = on_robot_id
        self._buf            = bytearray()
        self._screen_manager = screen_manager
        self._debug_screen   = None
        self._debug_fields   = {}
        self._last_heartbeat = time.ticks_ms()

        if screen_manager is not None:
            self._debug_screen = self._create_debug_screen(screen_manager)

        self._send_ping()

    # Senden

    def send(self, msg_type: int, payload: bytes):
        """Sendet ein Paket ueber UART."""
        self._uart.write(build_packet(msg_type, payload))

    def send_speed(self, left: int, right: int):
        """
        Sendet ein kombiniertes SPEED Paket (MSG_SPEED).

        :param left:  Geschwindigkeit linker Motor (-127..+127)
        :param right: Geschwindigkeit rechter Motor (-127..+127)
        """
        left  = max(-127, min(127, left))
        right = max(-127, min(127, right))
        # struct.pack mit signed bytes
        payload = struct.pack('bb', left, right)
        self.send(MSG_SPEED, payload)

    def send_metrics(self, battery_byte: int):
        """
        Sendet die Batteriespannung an den ESP32 (MSG_METRICS).

        :param battery_byte: Spannung, 40mV/LSB (0..255 -> 0-10.2V), wird vom
                             ESP32 1:1 in den MSG_TELEMETRY-Batterie-Byte übernommen
        """
        self.send(MSG_METRICS, bytes([battery_byte & 0xFF]))

    def send_debug(self, text, field_id: int = 0):
        """
        Sendet eine Debug-Log-Zeile an den PC (MSG_DEBUG).

        Der ESP32 stellt die robot_id voran und reicht die Zeile an den Dongle
        weiter; das swarm_terminal zeigt sie im DEBUG-LOG-Bereich an. Payload:
        [field_id][value_type][data...] – gleiches Format wie MSG_DEBUG_DATA,
        nur in Gegenrichtung (Robot -> PC).

        :param text:     Beliebiger String (wird auf 32 Bytes UTF-8 gekuerzt,
                         um im UART/ESP-NOW-Frame-Budget zu bleiben)
        :param field_id: Optionaler Kanal (0..255) zur Unterscheidung mehrerer Quellen
        """
        data    = text.encode("utf-8")[:32]
        payload = bytes([field_id & 0xFF, DBG_STRING]) + data
        self.send(MSG_DEBUG, payload)

    def _send_ping(self):
        self.send(MSG_PING, bytes())
        self._last_heartbeat = time.ticks_ms()

    # Empfangen

    def loop(self):
        """Muss regelmaessig in der Hauptschleife aufgerufen werden."""
        if self._uart.any():
            self._buf.extend(self._uart.read(self._uart.any()))
        self._parse()
        self._check_heartbeat()

    def _check_heartbeat(self):
        """Ueberprueft den Heartbeat-Timeout und sendet ggf. einen Ping."""
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, self._last_heartbeat)

        # Retry quickly until fields are registered
        if not self._debug_fields and elapsed >= 500:
            self._send_ping()
            return

        if elapsed >= _HEARTBEAT_TIMEOUT_MS:
            self._debug_fields.clear()
            if self._debug_screen:
                self._debug_screen.clear()
                self._debug_screen.clear_primitives()
                self._debug_screen.clear_diagram()
            self._send_ping()

    # Debug-Screen

    def _create_debug_screen(self, mgr) -> object:
        try:
            from screen_manager import Screen
        except ImportError:
            return None
        screen = Screen("ESP DEBUG")
        mgr.register(screen)
        return screen

    def _handle_debug_reg(self, payload: bytes):
        if len(payload) < 5:
            return
        field_id     = payload[0]
        display_type = payload[1]
        x            = payload[2]
        y            = payload[3]
        label        = bytes(payload[4:]).decode("utf-8", "ignore")

        field = {'type': display_type, 'label': label, 'x': x, 'y': y, 'graph': None}

        if self._debug_screen:
            if display_type == DBG_GRAPH:
                try:
                    from screen_manager import LineGraph
                    graph = LineGraph(label=label, max_val=100, min_val=-100, history=64, auto_scale=True)
                    self._debug_screen.set_diagram(graph)
                    field['graph'] = graph
                except ImportError:
                    pass
            elif display_type == DBG_METRIC:
                current = ['---']
                field['current'] = current
                self._debug_screen.set_metric(label, lambda c=current: c[0], x=x, y=y)

        self._debug_fields[field_id] = field
        if self._screen_manager:
            self._screen_manager.mark_dirty()

    def _handle_debug_data(self, payload: bytes):
        if len(payload) < 3:
            return
        field_id   = payload[0]
        value_type = payload[1]
        data       = payload[2:]

        if field_id not in self._debug_fields:
            return
        field = self._debug_fields[field_id]

        try:
            if value_type == DBG_FLOAT32 and len(data) >= 4:
                value = struct.unpack('<f', bytes(data[:4]))[0]
                value_str = f"{value:.1f}"
            elif value_type == DBG_INT8 and len(data) >= 1:
                value = data[0] if data[0] < 128 else data[0] - 256
                value_str = str(value)
            elif value_type == DBG_INT16 and len(data) >= 2:
                value = struct.unpack('<h', bytes(data[:2]))[0]
                value_str = str(value)
            elif value_type == DBG_STRING:
                value = bytes(data).decode("utf-8", "ignore")
                value_str = value
            else:
                return
        except Exception:
            return

        if not self._debug_screen:
            if self._screen_manager:
                self._screen_manager.mark_dirty()
            return

        display_type = field['type']
        if display_type == DBG_GRAPH and field.get('graph'):
            field['graph'].push(float(value))
        elif display_type == DBG_METRIC and 'current' in field:
            field['current'][0] = value_str
        elif display_type == DBG_LOG:
            self._debug_screen.log(f"{field['label']}:{value_str}")

        if self._screen_manager:
            self._screen_manager.mark_dirty()

    # Parser – nutzt Positions-Index statt wiederholter bytearray-Slices

    def _parse(self):
        buf = self._buf
        n   = len(buf)
        pos = 0

        while pos < n:
            # find() direkt auf bytearray – kein bytes()-Wrapper noetig
            idx = buf.find(b'\xAA\x55', pos)

            if idx == -1:
                if self._on_raw and pos < n:
                    raw_end = (n - 1) if buf[n - 1] == _MAGIC_0 else n
                    if raw_end > pos:
                        self._on_raw(bytes(buf[pos:raw_end]))
                pos = (n - 1) if (n > 0 and buf[n - 1] == _MAGIC_0) else n
                break

            if idx > pos and self._on_raw:
                self._on_raw(bytes(buf[pos:idx]))
            pos = idx

            if n - pos < 4:
                break

            msg_type = buf[pos + 2]
            length   = buf[pos + 3]
            pkt_end  = pos + 4 + length + 1

            if n < pkt_end:
                break

            # CRC direkt auf buf mit Offset – kein payload-bytes() noetig
            crc_recv = buf[pkt_end - 1]
            crc_calc = _crc8_buf(buf, pos + 2, 2 + length)

            if crc_recv == crc_calc:
                if msg_type == MSG_PING:
                    self._last_heartbeat = time.ticks_ms()
                elif msg_type == MSG_ROBOT_ID:
                    self._last_heartbeat = time.ticks_ms()
                    if length >= 1 and self._on_robot_id:
                        self._on_robot_id(buf[pos + 4])
                elif msg_type == MSG_DEBUG_REG:
                    self._last_heartbeat = time.ticks_ms()
                    self._handle_debug_reg(bytes(buf[pos + 4:pos + 4 + length]))
                elif msg_type == MSG_DEBUG_DATA:
                    self._last_heartbeat = time.ticks_ms()
                    self._handle_debug_data(bytes(buf[pos + 4:pos + 4 + length]))
                elif self._on_packet:
                    self._on_packet(Packet(msg_type, bytes(buf[pos + 4:pos + 4 + length]), True))

            pos = pkt_end

        # Einmalig trimmen – keine wiederholten Slices in der Schleife
        if pos >= n:
            self._buf = bytearray()
        elif pos > 0:
            self._buf = bytearray(buf[pos:])