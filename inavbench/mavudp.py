"""Relay MAVLink from SITL TCP ports to UDP, so a GCS sees every vehicle on one UDP port.

    python3 -m inavbench.mavudp 5762 5772 5782 5792            # to QGroundControl on udp 14550
    python3 -m inavbench.mavudp 5762 --to 127.0.0.1:14550 --to 127.0.0.1:14551
    python3 -m inavbench.mavudp 5762 5772 --each 14560        # also sysid n alone to udp 14560+n

SITL only offers TCP, and a GCS listening on UDP expects whole MAVLink packets,
so each TCP stream is framed into messages before it is forwarded. Every vehicle
goes to every `--to` address; with `--each`, vehicle sysid n also goes alone to
its own port, for clients that handle one vehicle each (a MAVSDK System). Whatever the
GCS sends back goes to the vehicle its target_system names, or to all of them.
Links that drop (SITL rebooting after its CLI save) are reconnected.
"""
import select
import socket
import sys
import time

from pymavlink.dialects.v20 import common as mavlink


def _parser():
    p = mavlink.MAVLink(None)
    p.robust_parsing = True
    return p


class _Link:
    def __init__(self, host, port):
        self.addr = (host, port)
        self.sock = None
        self.parser = None
        self.sysid = None
        self.next_try = 0.0

    def connect(self):
        if self.sock or time.monotonic() < self.next_try:
            return
        try:
            self.sock = socket.create_connection(self.addr, timeout=0.5)
            self.sock.setblocking(False)
            self.parser = _parser()
        except OSError:
            self.sock = None
            self.next_try = time.monotonic() + 0.5

    def drop(self):
        self.sock.close()
        self.sock = None
        self.next_try = time.monotonic() + 0.5


class Relay:
    def __init__(self, tcp_ports, to=(("127.0.0.1", 14550),), each=None, host="127.0.0.1"):
        """each: base UDP port; vehicle sysid n is also sent alone to host:each+n."""
        self.links = [_Link(host, p) for p in tcp_ports]
        self.to = list(to)
        self.each = each
        self.host = host
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("0.0.0.0", 0))          # a GCS replies to this port
        self.udp.setblocking(False)
        self._gcs_parsers = {}

    def run(self, stop=None):
        """Forward until stop (a threading.Event) is set, or forever."""
        while stop is None or not stop.is_set():
            for link in self.links:
                link.connect()
            socks = [l.sock for l in self.links if l.sock] + [self.udp]
            readable, _, _ = select.select(socks, [], [], 0.1)
            for s in readable:
                if s is self.udp:
                    self._from_gcs()
                else:
                    self._from_vehicle(next(l for l in self.links if l.sock is s))

    def _from_vehicle(self, link):
        try:
            data = link.sock.recv(65536)
        except OSError:
            data = b""
        if not data:
            link.drop()
            return
        for msg in link.parser.parse_buffer(data) or ():
            link.sysid = msg.get_srcSystem()
            raw = bytes(msg.get_msgbuf())
            for dest in self.to:
                self.udp.sendto(raw, dest)
            if self.each is not None:
                self.udp.sendto(raw, (self.host, self.each + link.sysid))

    def _from_gcs(self):
        data, src = self.udp.recvfrom(65536)
        parser = self._gcs_parsers.setdefault(src, _parser())
        for msg in parser.parse_buffer(data) or ():
            target = getattr(msg, "target_system", 0)
            raw = bytes(msg.get_msgbuf())
            for link in self.links:
                if link.sock and (target == 0 or link.sysid == target):
                    try:
                        link.sock.sendall(raw)
                    except OSError:
                        link.drop()


def main():
    args = sys.argv[1:]
    ports, to, each = [], [], None
    while args:
        a = args.pop(0)
        if a == "--to":
            host, port = args.pop(0).rsplit(":", 1)
            to.append((host, int(port)))
        elif a == "--each":
            each = int(args.pop(0))
        else:
            ports.append(int(a))
    if not ports:
        sys.exit(__doc__)
    to = to or [("127.0.0.1", 14550)]
    print(f"relaying MAVLink from tcp {ports} to udp {to}" + (f", and sysid n alone to {each}+n" if each else ""))
    Relay(ports, to, each).run()


if __name__ == "__main__":
    main()
