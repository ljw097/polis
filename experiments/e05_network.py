"""e05: an ordinary computer adds two numbers on Polis over UDP and TCP.

A compiled 8-bit adder is exposed through a gateway. The client below
uses nothing but standard sockets and the NEP v1 wire format, the way any
third-party program would.

Run: python experiments/e05_network.py 17 25
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from polis.gateway import (Gateway, PORTS, decode_result, encode_run, header,  # noqa: E402
                           tcp_call, udp_call)
from polis.program import Program, ripple_adder  # noqa: E402

BITS, TICKS = 8, 40


def request(x, y):
    # ports 0..7 = a0..a7, 8..15 = b0..b7 (see the PORTS reply)
    ev = [(i, 0, TICKS) for i in range(BITS) if x >> i & 1]
    ev += [(BITS + i, 0, TICKS) for i in range(BITS) if y >> i & 1]
    return encode_run(TICKS, ev)


def answer(reply):
    outs = decode_result(reply)  # (port, spikes, last_tick), s0..s8 in order
    return sum(1 << k for k, (_, _, last) in enumerate(outs) if last == TICKS - 1)


def main():
    x, y = (int(v) for v in sys.argv[1:3]) if len(sys.argv) > 2 else (17, 25)
    p = Program()
    ripple_adder(p, BITS)
    gw = Gateway(p.load())
    udp, tcp = gw.serve_udp(), gw.serve_tcp()
    ua, ta = udp.server_address, tcp.server_address

    names = udp_call(ua, header(PORTS))
    print(f"UDP {ua[0]}:{ua[1]}  TCP {ta[0]}:{ta[1]}  PORTS reply {len(names)} bytes")
    req = request(x, y)
    print(f"request: {len(req)} bytes")
    print(f"UDP: {x} + {y} = {answer(udp_call(ua, req))}")
    print(f"TCP: {x} + {y} = {answer(tcp_call(ta, req))}")
    udp.shutdown()
    tcp.shutdown()


if __name__ == "__main__":
    main()
