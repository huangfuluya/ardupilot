# Cooperative Lift · two-aircraft SITL demo

Two ArduCopter SITL instances flying a cooperative-lift formation on the
`clift-sitl` branch (based on `lthall/master`, which carries the 3-D position
controller offsets and the `FOLLOW_TARGET` support).

| File | Purpose |
| --- | --- |
| `run_sim.py` | launches the two SITL instances, loads `clift.lua`, relays `FOLLOW_TARGET`, flies the scripted path, writes `sim_log.csv` |
| `make_animation.py` | reads the log, reconstructs the slung payload (10 m rod, Verlet + constraint), renders `clift_sim.mp4` and `clift_summary.png` |
| `clift.lua` | the coordination script under test (same file as `cooperative_lift/clift_repro/clift.lua`) |
| `run0/`, `run1/` | SITL working directories (`APM/scripts/`, dataflash logs) |

## Run

```powershell
python run_sim.py --repo C:\Users\86913\Desktop\mh\ardupilot
python make_animation.py --fps 20
```

`run_sim.py` expects `build/sitl/bin/arducopter.exe` in the repo (built with
`./waf configure --board sitl && ./waf copter` in cygwin).

## What is simulated vs. reconstructed

* **Simulated**: both aircraft (ArduPilot SITL quad model), GUIDED/FOLLOW modes,
  `FOLLOW_TARGET` stream, the Lua offsets (acceleration lean + energy balance).
* **Reconstructed**: the slung payload. SITL has no tether physics, so the
  animation integrates a point mass on a rigid 10 m rod attached to the
  midpoint of the two aircraft (gravity + distance constraint + damping).
  It shows the *expected* payload response to the logged formation motion, not
  a closed-loop tether simulation.

## Notes

* SITL runs at 1× real time; one run takes ~90 s.
* `CLIFT_*` parameters only exist after `clift.lua` has loaded; `run_sim.py`
  waits for the `clift.lua loaded` STATUSTEXT before writing parameters.
* The follower's `FOLL_OFS_X = -10` puts it 10 m behind the lead; `CLIFT_D = ±5`
  makes the two stations symmetric about the formation centre.
