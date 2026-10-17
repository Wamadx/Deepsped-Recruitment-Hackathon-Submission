# Team Deepsped

## Members
| Name | Matric number | Email |
| --- | --- | --- |
| Dzul | <MATRIC> | <EMAIL> |
| Harikaran | <MATRIC> | <EMAIL> |
| Zhi Ning | <MATRIC> | <EMAIL> |
| Muhammad | <MATRIC> | <EMAIL> |

## Approach
Pure-pursuit path following against the circuit centreline, with a live
heading-change speed controller and a LiDAR safety override.

The driver subscribes to ground-truth odometry (`/ego_racecar/odom`, allowed and
recommended) to locate itself on the line, and steers with **pure pursuit**: it
aims at a point a `lookahead` distance ahead on the line and steers along the
circular arc that reaches it, with the lookahead scaled by speed (0.98–2.08 m).

Speed is decided **online** from the local curvature of the line: the driver
measures how much the line bends over the next 10–20 waypoints (the heading
change) and scales the target speed from 7.6 m/s on straights down to 3.6 m/s in
the tightest corners. A short lookahead, plus a small "sweeper correction"
(nudging the target inward on wide, high-speed bends), keeps pure pursuit from
cutting the inside of corners.

A forward LiDAR arc provides a last-resort safety limit: if clearance ahead drops
below 1.4 m, the requested speed is capped so the car slows rather than hits.

We chose pure pursuit because it is a few lines of geometry with one intuitive
knob (lookahead) and is stable at these speeds. We also tried a precomputed
curvature-based speed profile and a minimum-curvature racing line; the online
heading-change controller was both faster and reliably collision-free, so we
submitted it.

## How it uses the inputs
- **LiDAR** (`/scan`): forward-clearance safety limit only (not steering).
- **Odometry** (`/ego_racecar/odom`): ground-truth pose + velocity — the primary input.
- **Map** (`/map`): not read at runtime; the shipped centreline is loaded from file.

## Results on our own machine
The runs committed in `results/submitted/`, summarised:

| Best lap | 10-lap total | Collisions | Runs attempted |
| --- | --- | --- | --- |
| 19.44 | 194.81 | 0 | 3 |

### The machine that produced them
| CPU | GPU | RAM | OS | Typical real-time factor |
| --- | --- | --- | --- | --- |
| Intel Core Ultra 7 155H | integrated (no discrete GPU) | 16 GB | Windows 11 + WSL2 (Ubuntu 24.04) | 0.7–1.8 |

## Anything precomputed
None. The speed is computed online from the heading change; the only file loaded is
the shipped centreline `maps/icra26_centerline.csv`.

## Dependencies we added
None. The driver uses only `numpy`, which is preinstalled in the judging image.

| Package | pip or apt | What we use it for |
| --- | --- | --- |
| (none) | — | — |

## Third-party code and references
- Pure pursuit path tracking: R. C. Coulter, "Implementation of the Pure Pursuit
  Path Tracking Algorithm" (CMU-RI-TR-92-01).
- F1TENTH / RoboRacer course materials (vehicle dynamics, reactive baselines).

## AI assistance
OpenCode (DeepSeek V4 Pro) was used to scaffold the driver, diagnose simulator and
environment issues, and evaluate tuning. All code was reviewed and understood by
the team before submission.

## Known issues
- Pure pursuit still cuts the inside of corners at high speed; a minimum-curvature
  racing line would allow more, but the online controller was sufficient.
- No recovery logic: if the car is displaced off the line (e.g. by a collision),
  pure pursuit will steer back, but there is no explicit replanning.
