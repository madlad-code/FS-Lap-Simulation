"""
fs_lap_sim.simulator
Quasi-steady-state lap time simulation.
Point mass model — accurate enough for FS track analysis.

Method:
  1. Parse track from GPS/cones or parametric definition.
  2. Compute local curvature κ(s).
  3. Find max speed at each point from lateral grip limit.
  4. Forward-pass: limit acceleration from traction.
  5. Backward-pass: limit deceleration from braking.
  6. Integrate time, energy, and forces.

Usage:
    python simulator.py --track data/fsg_endurance.csv --car configs/default.json
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from vehicle import VehicleParams, TireModel


# ── TRACK ────────────────────────────────────────────────────────────────────

def load_track_csv(path: str) -> tuple[np.ndarray, np.ndarray]:
    """
    Load track from CSV with columns: x,y (meters, cartesian).
    Returns (x, y) arrays.
    """
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    return data[:, 0], data[:, 1]


def generate_track(name: str) -> tuple[np.ndarray, np.ndarray]:
    """
    Built-in parametric tracks for testing.
    """
    t = np.linspace(0, 2 * np.pi, 500, endpoint=False)

    if name == "oval":
        x = 50 * np.cos(t)
        y = 25 * np.sin(t)
    elif name == "figure8":
        x = 40 * np.sin(t)
        y = 40 * np.sin(t) * np.cos(t)
    elif name == "fsg":
        # Approximate FSG endurance layout (not exact)
        x = (60 * np.cos(t) + 20 * np.cos(2 * t))
        y = (40 * np.sin(t) + 15 * np.sin(3 * t))
    elif name == "skidpad":
        # FS skid pad: two circles of radius 9.125 m
        half = len(t) // 2
        x = np.concatenate([9.125 * np.cos(t[:half]), 9.125 * np.cos(t[half:]) + 18.25])
        y = np.concatenate([9.125 * np.sin(t[:half]), 9.125 * np.sin(t[half:])])
    else:
        raise ValueError(f"Unknown track: {name}")

    return x, y


def compute_curvature(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute arc-length s and curvature κ at each point.
    κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
    """
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    ds = np.sqrt(dx**2 + dy**2)
    s = np.cumsum(ds)
    s -= s[0]

    denom = (dx**2 + dy**2) ** 1.5
    kappa = np.abs(dx * ddy - dy * ddx) / np.maximum(denom, 1e-9)

    return s, kappa


# ── SOLVER ────────────────────────────────────────────────────────────────────

class LapSimulator:
    def __init__(self, car: VehicleParams):
        self.car = car

    def v_corner_max(self, kappa: np.ndarray) -> np.ndarray:
        """
        Maximum speed from lateral grip at each point.
        v = sqrt(a_lat_max / κ)
        κ < 1e-4 → effectively straight → cap at 80 m/s.
        """
        car = self.car
        v_max = np.zeros_like(kappa)
        for i, k in enumerate(kappa):
            if k < 1e-4:
                v_max[i] = 80.0
            else:
                # Iterative: a_lat depends on v (aero), solve approximately
                # Initial guess without aero
                v0 = np.sqrt(car.max_lateral_accel_ms2(0) / k)
                for _ in range(5):
                    a_lat = car.max_lateral_accel_ms2(v0)
                    v0 = min(np.sqrt(a_lat / k), 80.0)
                v_max[i] = v0
        return v_max

    def run(self, x: np.ndarray, y: np.ndarray) -> dict:
        car = self.car
        s, kappa = compute_curvature(x, y)
        n = len(s)
        ds = np.diff(s, append=s[-1] - s[-2])  # segment lengths

        v_limit = self.v_corner_max(kappa)

        # ── Forward pass (Acceleration: Beräkna maxfart utifrån grepp och effekt)
        v = np.zeros(n)
        v[0] = min(v_limit[0], 5.0)  # Startfart (t.ex. från stillastående)

        for i in range(1, n):
            seg = ds[i - 1]
            f_drive = car.max_traction_n(v[i - 1]) # Drivkraft begränsad av motor/däck
            f_drag = car.drag_n(v[i - 1])           # Luftmotstånd
            f_net = f_drive - f_drag
            a = f_net / car.mass_kg                 # F = ma => a = F/m
            # Beräkna ny fart via v^2 = u^2 + 2as
            v_new = np.sqrt(max(v[i - 1] ** 2 + 2 * a * seg, 0.01))
            v[i] = min(v_new, v_limit[i])           # Begränsa till kurvtagningsförmåga

        # ── Backward pass (Braking: Justera farten bakåt för att klara inbromsningar)
        for i in range(n - 2, -1, -1):
            seg = ds[i]
            f_brake = car.max_brake_force_n         # Max bromskraft
            f_drag = car.drag_n(v[i + 1])           # Luftmotstånd hjälper till vid inbromsning
            f_net_brake = f_brake + f_drag
            a_brake = f_net_brake / car.mass_kg
            # Beräkna tänkbar ingångsfart inför nästa punkt
            v_back = np.sqrt(max(v[i + 1] ** 2 + 2 * a_brake * seg, 0.01))
            v[i] = min(v[i], v_back, v_limit[i])    # Välj lägsta farten för säkerhet

        # ── Time integration
        dt = np.where(
            (v[:-1] + v[1:]) > 0,
            2 * ds[:-1] / (v[:-1] + v[1:]),
            0.0,
        )
        t_total = float(np.sum(dt))
        t_cum = np.concatenate([[0], np.cumsum(dt)])

        # ── Accelerations
        a_long = np.gradient(v, s)
        a_lat = v ** 2 * kappa

        # ── Energy (simplified)
        # Positive work = drive energy consumed
        # Negative work (braking) → regen if electric
        power = np.zeros(n)
        energy_drive_j = 0.0
        energy_regen_j = 0.0

        for i in range(1, n):
            v_avg = 0.5 * (v[i - 1] + v[i])
            f_drive = max(car.max_traction_n(v_avg) * (v[i] > v[i - 1]), 0)
            f_brake_active = car.max_brake_force_n * (v[i] < v[i - 1])
            work_drive = f_drive * ds[i - 1]
            work_regen = f_brake_active * ds[i - 1] * car.regen_efficiency if car.electric else 0.0
            energy_drive_j += work_drive
            energy_regen_j += work_regen

        energy_net_kwh = (energy_drive_j - energy_regen_j) / 3.6e6

        return {
            "s": s,
            "v": v,
            "kappa": kappa,
            "t_cum": t_cum,
            "a_long": a_long,
            "a_lat": a_lat,
            "lap_time_s": t_total,
            "v_max_ms": float(v.max()),
            "v_avg_ms": float(v.mean()),
            "energy_net_kwh": float(energy_net_kwh),
            "track_length_m": float(s[-1]),
            "n_points": n,
        }


# ── REPORT ───────────────────────────────────────────────────────────────────

def print_report(result: dict, car: VehicleParams) -> None:
    lap = result["lap_time_s"]
    mins = int(lap // 60)
    secs = lap % 60

    print()
    print("=" * 52)
    print("  FS LAP SIMULATION RESULT")
    print("=" * 52)
    print(f"  Lap time          {mins}:{secs:06.3f}")
    print(f"  Track length      {result['track_length_m']:.1f} m")
    print(f"  v_max             {result['v_max_ms'] * 3.6:.1f} km/h")
    print(f"  v_avg             {result['v_avg_ms'] * 3.6:.1f} km/h")
    print(f"  Energy (net)      {result['energy_net_kwh']:.3f} kWh")
    print(f"  SoC remaining     {100*(car.battery_capacity_kwh - result['energy_net_kwh'])/car.battery_capacity_kwh:.1f}%")
    print(f"  Points simulated  {result['n_points']}")
    print("=" * 52)
    print()


def export_telemetry(result: dict, path: str) -> None:
    """Export point-by-point telemetry as CSV."""
    import csv
    n = result["n_points"]
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["s_m", "v_ms", "v_kmh", "kappa", "t_s", "a_long_ms2", "a_lat_ms2"])
        for i in range(n):
            writer.writerow([
                round(result["s"][i], 3),
                round(result["v"][i], 4),
                round(result["v"][i] * 3.6, 3),
                round(result["kappa"][i], 6),
                round(result["t_cum"][i], 4),
                round(result["a_long"][i], 4),
                round(result["a_lat"][i], 4),
            ])
    print(f"  Telemetry → {path}")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="FS Lap Time Simulator")
    parser.add_argument("--track", default="fsg", help="Track: fsg|oval|figure8|skidpad or path/to/track.csv")
    parser.add_argument("--car", default=None, help="Path to car JSON config (optional)")
    parser.add_argument("--export", default=None, help="Export telemetry to CSV path")
    parser.add_argument("--laps", type=int, default=1, help="Number of laps to simulate")
    args = parser.parse_args()

    # Load car
    car = VehicleParams()
    if args.car:
        with open(args.car) as f:
            cfg = json.load(f)
        for k, v in cfg.items():
            if hasattr(car, k):
                setattr(car, k, v)

    # Load track
    if Path(args.track).exists():
        x, y = load_track_csv(args.track)
    else:
        x, y = generate_track(args.track)

    print(f"\n  Track: {args.track}  |  Car: {car.mass_kg} kg  |  Peak: {car.peak_power_kw} kW")

    t0 = time.perf_counter()
    sim = LapSimulator(car)
    result = sim.run(x, y)
    elapsed = time.perf_counter() - t0

    print_report(result, car)
    print(f"  Simulation time   {elapsed*1000:.1f} ms")

    if args.export:
        export_telemetry(result, args.export)

    if args.laps > 1:
        print(f"\n  {args.laps}-lap projection: {result['lap_time_s'] * args.laps / 60:.2f} min")
        print(f"  Energy projection: {result['energy_net_kwh'] * args.laps:.3f} kWh")


if __name__ == "__main__":
    main()
