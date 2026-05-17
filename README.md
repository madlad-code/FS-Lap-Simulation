# FS Lap Sim

Lap time simulation engine for Formula Student vehicles.

**Point-mass quasi-steady-state model** — fast enough for design iteration, accurate enough to rank parameters by impact on lap time.

```
python src/simulator.py --track fsg
python src/sensitivity.py --sweep all
python src/simulator.py --track data/custom.csv --export out/telemetry.csv
```

---

## What it does

| Module | Purpose |
|---|---|
| `vehicle.py` | Vehicle parameter set + Pacejka tire model |
| `simulator.py` | Lap time simulation (forward/backward integration) |
| `sensitivity.py` | Parametric sweep — ranks parameters by lap time impact |

**Outputs per simulation:**
- Lap time (mm:ss.ms)
- v(s), a_long(s), a_lat(s) traces
- Energy consumption + regen (EV)
- CSV telemetry export for overlay with real data

---

## Method

1. Parse track geometry (x, y) from CSV or parametric template  
2. Compute arc-length `s` and curvature `κ(s)` via numerical differentiation  
3. Compute `v_max(s)` from lateral grip (Pacejka Fy, aero downforce)  
4. Forward pass: acceleration limited by traction force and power  
5. Backward pass: deceleration limited by braking force  
6. Integrate `dt = ds / v` for lap time, energy, and g-g envelope  

---

## Built-in tracks

| Name | Description |
|---|---|
| `fsg` | Approximate FSG endurance layout |
| `oval` | Simple oval (baseline sanity check) |
| `figure8` | Figure-8 (handling test) |
| `skidpad` | FS skid pad (r = 9.125 m) |

Custom track: `--track path/to/track.csv` — CSV with columns `x,y` in meters.

---

## Sensitivity analysis

Ranks every vehicle parameter by its total lap time impact across a realistic range:

```
python src/sensitivity.py --sweep all --track fsg
```

Output (example ranking):
```
mass_kg                          +1.820 s
cl                               +1.240 s
peak_power_kw                    +0.980 s
cg_height_m                      +0.310 s
cd                               +0.280 s
```

Use this to direct engineering effort.

---

## Requirements

```
numpy
```

No other dependencies. Pure Python + NumPy.

```
pip install numpy
python src/simulator.py --track fsg
```

---

## Extending

- **Real track data**: export GPS from any logger (AiM, MoTeC, Pi) → convert to x,y CSV
- **Tire model**: replace `TireModel` in `vehicle.py` with measured Pacejka coefficients
- **Powertrain map**: replace constant peak power with a torque-speed lookup table
- **Damper/kinematics**: extend `VehicleParams` with suspension model for transient response

---

Oscar Enghag · Datateknik LTH · [github.com/madlad-code](https://github.com/madlad-code)
