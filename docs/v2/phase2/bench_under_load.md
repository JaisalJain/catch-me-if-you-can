| cell | v2.0 under load (3 sims in parallel) | re-plan rate | v2.1 alone | re-plan rate |
|---|---|---|---|---|
| vision-only, pinch, noise none | 93 % [84–97] (55/59) | 29.5 Hz | 100 % [94–100] (59/59) | 39.3 Hz |
| vision-only, pinch, noise low | 86 % [75–93] (51/59) | 29.5 Hz | 100 % [94–100] (59/59) | 39.3 Hz |
| **vision-only, pinch, noise medium (default)** | 95 % [86–98] (56/59) | 29.3 Hz | 100 % [94–100] (59/59) | 39.3 Hz |
| vision-only, pinch, noise high | 90 % [80–95] (53/59) | 31.4 Hz | 95 % [86–98] (56/59) | 39.3 Hz |
| plan-informed, pinch, noise none | 98 % [91–100] (57/58) | 0.0 Hz | 100 % [94–100] (59/59) | 0.0 Hz |
| plan-informed, pinch, noise low | 98 % [91–100] (58/59) | 0.0 Hz | 100 % [94–100] (59/59) | 0.0 Hz |
| plan-informed, pinch, noise medium | 39 % [28–52] (23/59) | 0.0 Hz | 39 % [28–52] (23/59) | 0.0 Hz |
| plan-informed, pinch, noise high | 27 % [17–40] (16/59) | 0.0 Hz | 22 % [14–35] (13/58) | 0.0 Hz |
| vision-only, pre-closed cup (square throws), medium | 79 % [67–88] (45/57) | 27.3 Hz | 90 % [79–95] (52/58) | 35.7 Hz |
| vision-only, pinch, 64 Hz cameras, medium | 95 % [86–98] (56/59) | 31.4 Hz | 97 % [88–99] (57/59) | 39.2 Hz |
| vision-only, pinch, gripper latency 0 ms (B assumes 10), medium | 76 % [63–85] (44/58) | 28.6 Hz | 97 % [88–99] (57/59) | 39.3 Hz |
| ... latency 20 ms | 79 % [67–88] (46/58) | 29.5 Hz | 71 % [59–81] (42/59) | 39.3 Hz |
| ... latency 30 ms | 58 % [45–69] (34/59) | 29.5 Hz | 64 % [52–75] (38/59) | 39.3 Hz |
| B's gripper fast_close (NOT a 2F-85; 10x speed, realistic force), medium  | 90 % [80–95] (53/59) | 39.3 Hz | 88 % [77–94] (52/59) | 39.3 Hz |
| B's gripper fast_idealized (NOT a 2F-85; 10x speed, 5x force), medium | 97 % [88–99] (57/59) | 29.0 Hz | 100 % [94–100] (59/59) | 39.3 Hz |
