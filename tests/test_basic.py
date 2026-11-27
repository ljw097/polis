import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from polis import Field, Substrate


def test_migration_reduces_hops():
    rng = np.random.default_rng(0)
    f = Field(256, init_degree=8, rng=rng, w_init=(0.2, 0.5))
    s = Substrate(256, grid=4, capacity=16, region_tiles=2, rng=np.random.default_rng(1))
    first, last = [], []
    for t in range(600):
        ext = np.zeros(256)
        ext[(t // 30 % 4) * 16:(t // 30 % 4 + 1) * 16] = 1.2
        src, dst = f.step(ext)
        if len(src):
            (first if t < 100 else last).append(s.route(src, dst))
        if (t + 1) % 25 == 0:
            s.migrate()
    assert np.concatenate(last).mean() < np.concatenate(first).mean()
    assert sorted(np.bincount(s.tile, minlength=16)) == [16] * 16  # capacity kept
