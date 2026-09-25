"""MSPv2 client for an INAV SITL TCP port, plus the text CLI on the same port."""
import socket
import struct
import time


class MspError(IOError):
    pass


class CliError(RuntimeError):
    pass


def _crc_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return bytes(table)


_CRC = _crc_table()
_HEADER = struct.Struct("<BHH")


def _crc8_dvb_s2(data, crc=0):
    for b in data:
        crc = _CRC[crc ^ b]
    return crc


class Msp:
    def __init__(self, port=5760, host="127.0.0.1", timeout=3.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._rx = bytearray()

    def close(self):
        self.sock.close()

    @staticmethod
    def _frame(cmd, payload):
        body = _HEADER.pack(0, cmd, len(payload)) + payload
        return b"$X<" + body + bytes([_crc8_dvb_s2(body)])

    def request(self, cmd, payload=b""):
        return self.request_many(((cmd, payload),))[0]

    def request_many(self, requests):
        """Send several (cmd, payload) requests in one write, then collect the replies in order.
        Saves a round trip per extra request."""
        self.sock.sendall(b"".join(self._frame(cmd, payload) for cmd, payload in requests))
        replies = []
        for cmd, payload in requests:
            while True:
                rcmd, data, is_error = self._read_frame()
                if rcmd == cmd:
                    break
            if is_error:
                raise MspError(f"FC rejected MSP command 0x{cmd:04X} (payload {payload.hex() or 'empty'})")
            replies.append(data)
        return replies

    def _read_frame(self):
        rx = self._rx
        while True:
            start = rx.find(b"$X")
            if start < 0:
                del rx[:max(0, len(rx) - 1)]         # keep a trailing "$" in case "X" follows
            elif len(rx) >= start + 8:
                _, cmd, size = _HEADER.unpack_from(rx, start + 3)
                end = start + 8 + size
                if len(rx) > end:
                    header_and_data = rx[start + 3:end]
                    if rx[end] != _crc8_dvb_s2(header_and_data):
                        raise MspError(f"CRC mismatch on reply to 0x{cmd:04X}")
                    is_error = rx[start + 2] == 0x21     # "!"
                    data = bytes(rx[start + 8:end])
                    del rx[:end + 1]
                    return cmd, data, is_error
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("SITL closed the MSP connection")
            rx += chunk

    def cli(self, lines, timeout=5.0):
        """Run CLI commands. Raises CliError on the first line INAV rejects.

        Leaves the port in CLI mode: finish with cli_save() (which reboots the
        SITL) or cli_exit()."""
        self.sock.sendall(b"#")
        self._read_prompt(timeout)
        output = []
        for line in lines:
            self.sock.sendall(line.encode() + b"\n")
            text = self._read_prompt(timeout)
            if "### ERROR" in text or "Unknown command" in text:
                raise CliError(f"INAV rejected {line!r}: {text.strip()}")
            output.append(text)
        return "".join(output)

    def cli_save(self, timeout=10.0):
        """'save' writes the eeprom and reboots the SITL. Returns once the reboot
        has closed this connection, so a new one reaches the rebooted FC."""
        self.sock.sendall(b"save\n")
        self.sock.settimeout(timeout)
        try:
            while self.sock.recv(65536):
                pass
        finally:
            self.close()

    def cli_exit(self):
        self.sock.sendall(b"exit\n")
        time.sleep(0.5)
        self.close()

    def _read_prompt(self, timeout):
        deadline = time.monotonic() + timeout
        while not self._rx.endswith(b"\r\n# "):
            if time.monotonic() > deadline:
                raise CliError(f"no CLI prompt within {timeout} s, got {bytes(self._rx[-200:])!r}")
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("SITL closed the connection during CLI")
            self._rx += chunk
        text = self._rx.decode(errors="replace")
        self._rx.clear()
        return text
