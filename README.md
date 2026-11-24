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

## License

Apache 2.0
