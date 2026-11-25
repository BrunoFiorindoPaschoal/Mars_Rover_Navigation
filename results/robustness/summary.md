# Mars Yard Robustness Study Summary (Domain Randomization Levels 0–3)

Total runs: **96** across **3** combo(s), **4** randomization level(s) (L0, L1, L2, L3), and **4** terrain group(s).
Success criterion (Phase 6 rule): Nav2 `SUCCEEDED` **and** ground-truth goal error `<= 1.0 m` (else `goal_miss`).

## 1. True success rate vs randomization level

| Combo | L0 | L1 | L2 | L3 |
|---|---:|---:|---:|---:|
| `smac_hybrid+dwb` | 100.0% (8/8) | 100.0% (8/8) | 75.0% (6/8) | 75.0% (6/8) |
| `smac_hybrid+mppi` | 50.0% (4/8) | 50.0% (4/8) | 62.5% (5/8) | 50.0% (4/8) |
| `smac_hybrid+rpp` | 100.0% (8/8) | 87.5% (7/8) | 87.5% (7/8) | 62.5% (5/8) |

## 2. Mean ground-truth goal error (m) vs level (all runs, mean ± std)

| Combo | L0 | L1 | L2 | L3 |
|---|---:|---:|---:|---:|
| `smac_hybrid+dwb` | 0.528 ± 0.182 | 0.574 ± 0.201 | 0.626 ± 0.357 | 0.994 ± 0.780 |
| `smac_hybrid+mppi` | 1.759 ± 1.931 | 1.742 ± 1.974 | 2.027 ± 2.342 | 3.394 ± 4.621 |
| `smac_hybrid+rpp` | 0.557 ± 0.238 | 0.680 ± 0.499 | 0.670 ± 0.494 | 1.182 ± 0.876 |

## 3. Mean energy (J) vs level (all runs, mean ± std)

| Combo | L0 | L1 | L2 | L3 |
|---|---:|---:|---:|---:|
| `smac_hybrid+dwb` | 435.8 ± 54.7 | 556.6 ± 54.8 | 564.3 ± 73.9 | 509.3 ± 77.0 |
| `smac_hybrid+mppi` | 629.3 ± 281.9 | 695.9 ± 325.4 | 565.8 ± 231.7 | 564.4 ± 290.4 |
| `smac_hybrid+rpp` | 487.8 ± 66.9 | 599.3 ± 104.3 | 584.9 ± 90.9 | 551.4 ± 96.7 |

## 4. Failure counts by combo and level

| Combo | Level | Nav2 ok | timeout | stuck | collision | tipover | aborted | goal_miss | Total fail |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `smac_hybrid+dwb` | L0 | 8/8 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `smac_hybrid+dwb` | L1 | 8/8 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `smac_hybrid+dwb` | L2 | 8/8 | 0 | 0 | 0 | 0 | 0 | 2 | 2 |
| `smac_hybrid+dwb` | L3 | 8/8 | 0 | 0 | 0 | 0 | 0 | 2 | 2 |
| `smac_hybrid+mppi` | L0 | 8/8 | 0 | 0 | 0 | 0 | 0 | 4 | 4 |
| `smac_hybrid+mppi` | L1 | 8/8 | 0 | 0 | 0 | 0 | 0 | 4 | 4 |
| `smac_hybrid+mppi` | L2 | 7/8 | 0 | 0 | 1 | 0 | 0 | 2 | 3 |
| `smac_hybrid+mppi` | L3 | 7/8 | 0 | 0 | 1 | 0 | 0 | 3 | 4 |
| `smac_hybrid+rpp` | L0 | 8/8 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `smac_hybrid+rpp` | L1 | 8/8 | 0 | 0 | 0 | 0 | 0 | 1 | 1 |
| `smac_hybrid+rpp` | L2 | 8/8 | 0 | 0 | 0 | 0 | 0 | 1 | 1 |
| `smac_hybrid+rpp` | L3 | 8/8 | 0 | 0 | 0 | 0 | 0 | 3 | 3 |

## 5. True success rate by terrain group

| Combo | Level | Flat | Slope | Rocks | Sand |
|---|---:|---:|---:|---:|---:|
| `smac_hybrid+dwb` | L0 | 100.0% (2/2) | 100.0% (2/2) | 100.0% (2/2) | 100.0% (2/2) |
| `smac_hybrid+dwb` | L1 | 100.0% (2/2) | 100.0% (2/2) | 100.0% (2/2) | 100.0% (2/2) |
| `smac_hybrid+dwb` | L2 | 100.0% (2/2) | 0.0% (0/2) | 100.0% (2/2) | 100.0% (2/2) |
| `smac_hybrid+dwb` | L3 | 100.0% (2/2) | 0.0% (0/2) | 100.0% (2/2) | 100.0% (2/2) |
| `smac_hybrid+mppi` | L0 | 100.0% (2/2) | 0.0% (0/2) | 0.0% (0/2) | 100.0% (2/2) |
| `smac_hybrid+mppi` | L1 | 100.0% (2/2) | 0.0% (0/2) | 0.0% (0/2) | 100.0% (2/2) |
| `smac_hybrid+mppi` | L2 | 100.0% (2/2) | 0.0% (0/2) | 50.0% (1/2) | 100.0% (2/2) |
| `smac_hybrid+mppi` | L3 | 100.0% (2/2) | 0.0% (0/2) | 0.0% (0/2) | 100.0% (2/2) |
| `smac_hybrid+rpp` | L0 | 100.0% (2/2) | 100.0% (2/2) | 100.0% (2/2) | 100.0% (2/2) |
| `smac_hybrid+rpp` | L1 | 100.0% (2/2) | 50.0% (1/2) | 100.0% (2/2) | 100.0% (2/2) |
| `smac_hybrid+rpp` | L2 | 100.0% (2/2) | 50.0% (1/2) | 100.0% (2/2) | 100.0% (2/2) |
| `smac_hybrid+rpp` | L3 | 100.0% (2/2) | 0.0% (0/2) | 50.0% (1/2) | 100.0% (2/2) |
