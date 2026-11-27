import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "experiments"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from e05_network import answer, request  # noqa: E402
from polis.gateway import Gateway, decode_result, encode_run, tcp_call, udp_call  # noqa: E402
from polis.program import Program, ripple_adder  # noqa: E402
import pytest  # noqa: E402


@pytest.fixture(scope="module")
def servers():
    p = Program()
    ripple_adder(p, 8)
    gw = Gateway(p.load())
    u, t = gw.serve_udp(), gw.serve_tcp()
    yield u.server_address, t.server_address
    u.shutdown()
    t.shutdown()


def test_add_over_udp_and_tcp(servers):
    ua, ta = servers
    rnd = random.Random(0)
    for _ in range(30):
        x, y = rnd.randrange(256), rnd.randrange(256)
        assert answer(udp_call(ua, request(x, y))) == x + y
        assert answer(tcp_call(ta, request(x, y))) == x + y


def test_bad_requests_get_an_error_not_a_crash(servers):
    ua, ta = servers
    with pytest.raises(RuntimeError):
        decode_result(udp_call(ua, encode_run(10, [(99, 0, 5)])))  # 99 is not an input
    with pytest.raises(RuntimeError):
        decode_result(tcp_call(ta, b"hello"))
    assert answer(udp_call(ua, request(1, 2))) == 3          # still alive
