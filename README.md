# Polis

A brain-inspired computer architecture with no fixed circuits — it rewires and organizes itself as it runs.

> 정해진 회로 없이, 쓰면서 스스로 자라고 조직되는 뇌 모방 컴퓨터 아키텍처.

Polis is the first architecture of the **Nomos** paradigm. Where RISC and CISC
describe how to design an instruction set, Nomos machines have no
user-facing instructions at all: every node obeys the same small set of
local rules (the *constitution*), and the program is whatever structure
the connections grow into (the *customs*).

## Two layers

| Layer | What it is |
|---|---|
| **Virtual field** | Any node may connect to any node. Connections are addresses with weights, so they can appear and vanish freely. Inputs and outputs exist only at the edge. |
| **Physical substrate** | A grid of small tiles, each hosting hundreds of virtual nodes. A spike becomes a packet routed between tiles. Nodes *migrate* so that nodes that talk a lot live close together. |

Migration is governed federally: tiles form regions that decide moves
inside themselves; moves between regions are negotiated by elected
representatives. See [docs/architecture.md](docs/architecture.md).

## Status: v0 simulator

```
pip install numpy
python experiments/e01_migration.py
```

First result (1024 nodes, 64 tiles, 4 regions, 3000 ticks, 3 seeds),
same traffic fed to a static and a federal substrate:

| | static | federal |
|---|---|---|
| mean hops per packet | 5.2–5.8 | **~1.0** |
| packets staying on-tile | ~1% | **34–43%** |
| region purity (chance ≈ 0.29) | 0.33–0.34 | **0.60–0.81** |

Regions start specializing in one input pattern without anyone
assigning them — the first sign of emergent functional areas.

Known limit: only ~17% of nodes become active; most of the interior
stays silent. Homeostasis and inhibition (experiment e02) wake the
interior but one pattern then wins everywhere — see
[docs/architecture.md](docs/architecture.md#e02-조용한-내부-깨우기-항상성-억제).

## Programming model

A program is a connection graph, made in one of three ways on the same
hardware: **compile** (write the links directly — deterministic),
**learn** (links grow from reward), or **mixed** (a compiled skeleton
with some links left open to learning). In compiled mode a field builds
gates, a half adder and a 1-bit latch from a few nodes, so Polis is a
general-purpose machine, not only a learning accelerator.
See [docs/programming-model.md](docs/programming-model.md).

## RTL v1: everything in (verified)

`rtl/v1/` adds leak, refractory, homeostasis, the learning unit (Art. 2,
3, 7b + credit), reward sweeps (Art. 8), synchronous SRAMs, a config
read/write port for the management-core firmware (migration, new links,
incoming budget) and a NEP gateway bridge. Co-simulation against
`sim/polis/hw1.py`: 17,632 spikes and every word of every table identical
across 6 runs with migration, growth, budget sweeps and rewards; migration
is transparent (same spikes as without it). NEP request bytes from a
socket client run on the RTL SoC and come back as correct sums.
Critical path: 181 gate levels in node update (router: 33) — needs
pipelining. See [docs/rtl-v1.md](docs/rtl-v1.md).

## RTL v0 (verified)

Synthesizable Verilog for the tile, router and mesh (compiled mode) in
`rtl/`. Co-simulated against the Python hardware model with Verilator:
an 8-bit adder and random programs on 4x4 and 8x8 meshes — 88,474
spikes, every tick identical. Yosys: ~97.5K transistors per tile +
router, about 1/10 of the tile's SRAM area, so the chip is memory-bound.
See [docs/rtl.md](docs/rtl.md).

## At scale (e08)

Vectorised model up to 262K nodes / 16.8M synapses on a synthetic,
brain-like modular graph. Migration that balances traffic *and* work
gives 2.2x shorter ticks and 3x less energy than random placement;
optimising traffic alone creates compute hotspots. Memory is the weak
spot: 29–38 bits per synapse at best, ~10–20x fewer synapses per mm²
than Loihi 2 — the price of free, migrating, per-synapse-plastic
connections. See [docs/scale.md](docs/scale.md).

## Hardware model (e07)

`sim/polis/hw.py` models the tile design with int8 weights, int16
potentials, fixed-size SRAM tables and a per-tick cycle count. It matches
the float simulator bit for bit on compiled programs and spike for spike
on a learned field. Open issues: free connectivity costs ~48 bits per
synapse, and at small scale migration shifts the bottleneck from network
to compute. See [docs/tile.md](docs/tile.md).

## Talks TCP/UDP (e05)

Inside the chip, packets are tiny custom spikes; at the edge, gateway
tiles run a standard TCP/IP stack. Any ordinary program can call Polis
with plain sockets using the small NEP v1 message format:

```
$ python experiments/e05_network.py 17 25
UDP: 17 + 25 = 42
TCP: 17 + 25 = 42
```

See [docs/network.md](docs/network.md).

## Migration without errors (e04)

Nodes keep moving while a compiled 8-bit adder runs. With the Polis
protocol (home directory + address caches + forwarding pointers + tick
barrier) all 200 additions stay correct even at 5 moves per tick, at
0.5–3 control messages per 100 packets at realistic rates. Without cache
correction and forwarding, almost every addition is wrong even when
migration is rare. See [docs/migration-protocol.md](docs/migration-protocol.md).

## Learning experiment (e03)

Taught only by a broadcast +1/−1 reward (no backprop), a 512-node field
learns a simple mapping (answer = a): 65–78% vs 31% for the untaught
control. It does **not** yet learn XOR (42–53%, control 42%): one global
reward is too coarse to train hidden computation. Details in
[docs/architecture.md](docs/architecture.md#e03-계산할-수-있나-교육).

## Docs

- [Glossary: CS ↔ Polis](docs/glossary.md)
- [Architecture](docs/architecture.md)
- [Constitution (local rules)](docs/constitution.md)
- [Programming model](docs/programming-model.md)
- [Migration protocol](docs/migration-protocol.md)
- [Network and compatibility](docs/network.md)
- [On-chip packets and routing](docs/packets.md)
- [Tile microarchitecture](docs/tile.md)
- [Evaluation at scale](docs/scale.md)
- [RTL v0 and verification](docs/rtl.md)
- [RTL v1: full feature set](docs/rtl-v1.md)

## License

Apache 2.0
