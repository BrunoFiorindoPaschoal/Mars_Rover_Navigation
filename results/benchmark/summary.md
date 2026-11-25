# Mars Yard Benchmark Summary

Total runs: **30** across **6** planner × controller combo(s) and **4** terrain group(s).

## 1. Success rate (combo × terrain group)

| Combo | Flat | Slope | Rocks | Sand | Overall |
|---|---:|---:|---:|---:|---:|
| `navfn+dwb` | — | 100.0% (1/1) | 100.0% (1/1) | — | **100.0% (2/2)** |
| `navfn+mppi` | — | 0.0% (0/1) | 100.0% (1/1) | — | **50.0% (1/2)** |
| `navfn+rpp` | — | 100.0% (1/1) | 100.0% (1/1) | — | **100.0% (2/2)** |
| `smac_hybrid+dwb` | 100.0% (5/5) | 100.0% (5/5) | 100.0% (5/5) | 100.0% (5/5) | **100.0% (20/20)** |
| `smac_hybrid+mppi` | — | 0.0% (0/1) | 0.0% (0/1) | — | **0.0% (0/2)** |
| `smac_hybrid+rpp` | — | 100.0% (1/1) | 100.0% (1/1) | — | **100.0% (2/2)** |

## 2. Navigation metrics over successful runs (mean ± std)

| Combo | N ok | Time (s) | Path ratio | Tracking err (m) | Slip ratio | Lat slide (m) | EKF drift (m) | Energy (J) | Min clr (m) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `navfn+dwb` | 2 | 32.55 ± 6.22 | 0.991 ± 0.013 | 0.117 ± 0.018 | 0.009 ± 0.010 | 0.291 ± 0.150 | 0.327 ± 0.079 | 349.6 ± 42.1 | 2.30 ± 2.42 |
| `navfn+mppi` | 1 | 32.35 ± 0.00 | 1.000 ± 0.000 | 0.300 ± 0.000 | 0.019 ± 0.000 | 0.412 ± 0.000 | 0.501 ± 0.000 | 507.3 ± 0.0 | 0.61 ± 0.00 |
| `navfn+rpp` | 2 | 31.97 ± 5.91 | 0.993 ± 0.015 | 0.134 ± 0.027 | 0.010 ± 0.015 | 0.242 ± 0.052 | 0.302 ± 0.019 | 463.7 ± 29.4 | 2.32 ± 2.40 |
| `smac_hybrid+dwb` | 20 | 26.59 ± 4.39 | 0.979 ± 0.022 | 0.105 ± 0.111 | 0.011 ± 0.023 | 0.198 ± 0.149 | 0.274 ± 0.205 | 425.8 ± 69.2 | 3.24 ± 2.60 |
| `smac_hybrid+mppi` | 0 | — | — | — | — | — | — | — | — |
| `smac_hybrid+rpp` | 2 | 32.82 ± 5.30 | 1.014 ± 0.015 | 0.399 ± 0.100 | 0.004 ± 0.021 | 0.588 ± 0.182 | 0.640 ± 0.120 | 535.8 ± 22.2 | 2.37 ± 2.33 |

## 3. Failure counts by type

| Combo | timeout | stuck | collision | tipover | aborted | goal_miss | Total fail |
|---|---:|---:|---:|---:|---:|---:|---:|
| `navfn+dwb` | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `navfn+mppi` | 0 | 0 | 0 | 0 | 0 | 1 | 1 |
| `navfn+rpp` | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `smac_hybrid+dwb` | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `smac_hybrid+mppi` | 0 | 0 | 0 | 0 | 0 | 2 | 2 |
| `smac_hybrid+rpp` | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
