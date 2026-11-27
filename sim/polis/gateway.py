"""Gateway: Polis speaks ordinary UDP and TCP.

Edge tiles host a conventional network stack. They translate between
standard sockets and port events of a running Machine. Inside the chip
only short IDs travel; full IP/UDP/TCP headers exist only here.

Wire format, NEP v1 (Nomos Event Protocol), big-endian:

  header   : magic "NM" | version u8 = 1 | type u8
  RUN  (1) : ticks u16 | n u16 | n x (port u16, start u16, end u16)
             input `port` is held on during ticks [start, end)
  RESULT(2): n u16 | n x (port u16, spikes u16, last_tick u16)
             one entry per output port; last_tick = 0xFFFF if silent
  PORTS(3) : request is header only; reply is
             n u16 | n x (port u16, kind u8 [0 in, 1 out], len u8, name)
  ERROR(4) : len u16 | utf-8 message

UDP: one message per datagram. TCP: each message prefixed by a u32 length.
Port numbers: inputs first, then outputs, in declaration order.
"""
import socket
import socketserver
import struct
import threading

MAGIC, VER = b"NM", 1
RUN, RESULT, PORTS, ERROR = 1, 2, 3, 4
SILENT = 0xFFFF


def header(t):
    return MAGIC + bytes([VER, t])


class Gateway:
    def __init__(self, machine, max_ticks=4096):
        self.m = machine
        self.lock = threading.Lock()
        self.max_ticks = max_ticks
        self.in_names = list(machine.inputs)
        self.out_names = list(machine.outputs)

    # ---- message handling (transport independent) ----------------------
    def handle(self, msg):
        try:
            if len(msg) < 4 or msg[:2] != MAGIC or msg[2] != VER:
                raise ValueError("bad header")
            t = msg[3]
            if t == PORTS:
                return self._ports()
            if t == RUN:
                return self._run(msg[4:])
            raise ValueError(f"unknown type {t}")
        except (ValueError, struct.error, IndexError) as e:
            b = str(e).encode()
            return header(ERROR) + struct.pack(">H", len(b)) + b

    def _ports(self):
        out = header(PORTS) + struct.pack(">H", len(self.in_names) + len(self.out_names))
        for i, name in enumerate(self.in_names + self.out_names):
            b = name.encode()
            out += struct.pack(">HBB", i, 0 if i < len(self.in_names) else 1, len(b)) + b
        return out

    def _run(self, body):
        ticks, n = struct.unpack_from(">HH", body)
        if not 0 < ticks <= self.max_ticks:
            raise ValueError("ticks out of range")
        events = {name: set() for name in self.in_names}
        for k in range(n):
            port, start, end = struct.unpack_from(">HHH", body, 4 + 6 * k)
            if port >= len(self.in_names):
                raise ValueError(f"port {port} is not an input")
            events[self.in_names[port]].update(range(start, min(end, ticks)))
        with self.lock:
            res = self.m.run(events, ticks)
        out = header(RESULT) + struct.pack(">H", len(self.out_names))
        base = len(self.in_names)
        for j, name in enumerate(self.out_names):
            fired = res[name]
            out += struct.pack(">HHH", base + j, len(fired), fired[-1] if fired else SILENT)
        return out

    # ---- transports ------------------------------------------------------
    def serve_udp(self, host="127.0.0.1", port=0):
        gw = self

        class H(socketserver.BaseRequestHandler):
            def handle(self):
                data, sock = self.request
                sock.sendto(gw.handle(data), self.client_address)

        return _start(socketserver.ThreadingUDPServer((host, port), H))

    def serve_tcp(self, host="127.0.0.1", port=0):
        gw = self

        class H(socketserver.BaseRequestHandler):
            def handle(self):
                while True:
                    msg = _recv_frame(self.request)
                    if msg is None:
                        return
                    reply = gw.handle(msg)
                    self.request.sendall(struct.pack(">I", len(reply)) + reply)

        socketserver.ThreadingTCPServer.allow_reuse_address = True
        return _start(socketserver.ThreadingTCPServer((host, port), H))


def _start(server):
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def _recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def _recv_frame(sock):
    h = _recv_exact(sock, 4)
    if h is None:
        return None
    return _recv_exact(sock, struct.unpack(">I", h)[0])


# ---- a minimal client, as any third-party host would write it ----------

def encode_run(ticks, events):
    """events: list of (port, start, end)."""
    return header(RUN) + struct.pack(">HH", ticks, len(events)) + b"".join(
        struct.pack(">HHH", *e) for e in events)


def decode_result(msg):
    if msg[3] == ERROR:
        raise RuntimeError(msg[6:].decode())
    n = struct.unpack_from(">H", msg, 4)[0]
    return [struct.unpack_from(">HHH", msg, 6 + 6 * k) for k in range(n)]


def udp_call(addr, msg, timeout=5.0):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.settimeout(timeout)
        s.sendto(msg, addr)
        return s.recv(65535)


def tcp_call(addr, msg, timeout=5.0):
    with socket.create_connection(addr, timeout=timeout) as s:
        s.sendall(struct.pack(">I", len(msg)) + msg)
        return _recv_frame(s)
