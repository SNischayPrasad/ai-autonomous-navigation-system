# Benchmark summary

## Planner comparison (fully known maps)

| Planner | Avg time (ms) | Avg nodes expanded | Avg path length (m) |
|---|---|---|---|
| dijkstra | 214.2 | 33843 | 86.2 |
| astar | 103.1 | 15032 | 86.2 |
| weighted_astar | 23.8 | 2893 | 86.2 |

## Closed-loop autonomous runs (unknown map + moving obstacles)

| Planner | Runs | Success rate | Collisions | Avg time to goal (s) | Avg min clearance (m) |
|---|---|---|---|---|---|
| dijkstra | 11 | 100 % | 0 | 43.9 | 1.01 |
| astar | 11 | 100 % | 0 | 40.8 | 1.08 |
| weighted_astar | 11 | 100 % | 0 | 42.1 | 0.94 |
