"""
fs_lap_sim.sensitivity
Parametric sensitivity analysis: how much does each parameter move lap time?

Usage:
    python sensitivity.py --track fsg --param mass --range 180,260,10
    python sensitivity.py --track fsg --sweep all
"""

import argparse
import sys
from copy import deepcopy

import numpy as np

sys.path.insert(0, ".")
from vehicle import VehicleParams
from simulator import LapSimulator, generate_track, load_track_csv
from pathlib import Path


SWEEP_PARAMS = {
    "mass_kg":          (170, 280, 12, "kg"),
    "cl":               (1.0, 4.5, 8,  "—"),
    "cd":               (0.8, 2.5, 8,  "—"),
    "peak_power_kw":    (50,  100, 6,  "kW"),
    "cg_height_m":      (0.22, 0.38, 8, "m"),
    "cg_bias_front":    (0.40, 0.56, 8, "—"),
    "brake_bias_front": (0.50, 0.70, 5, "—"),
}


def run_baseline(car: VehicleParams, x, y) -> float:
    return LapSimulator(car).run(x, y)["lap_time_s"]


def sweep_param(param: str, lo: float, hi: float, steps: int,
                base_car: VehicleParams, x, y) -> list[tuple[float, float]]:
    results = []
    for val in np.linspace(lo, hi, steps):
        car = deepcopy(base_car)
        setattr(car, param, float(val))
        t = LapSimulator(car).run(x, y)["lap_time_s"]
        results.append((float(val), t))
    return results


def print_sensitivity_table(param: str, data: list[tuple[float, float]],
                             baseline: float, unit: str) -> None:
    print(f"\n  ── {param} [{unit}] ──")
    print(f"  {'value':>10}  {'lap (s)':>8}  {'delta (s)':>10}  {'delta (%)':>10}")
    print(f"  {'─'*44}")
    for val, t in data:
        delta = t - baseline
        pct = 100 * delta / baseline
        marker = " ←base" if abs(delta) < 0.01 else ""
        print(f"  {val:>10.3f}  {t:>8.3f}  {delta:>+10.3f}  {pct:>+9.2f}%{marker}")


def sweep_all(base_car: VehicleParams, x, y) -> None:
    baseline = run_baseline(base_car, x, y)
    mins = int(baseline // 60)
    secs = baseline % 60
    print(f"\n  Baseline: {mins}:{secs:06.3f}  ({baseline:.3f} s)")

    sensitivities = []
    for param, (lo, hi, steps, unit) in SWEEP_PARAMS.items():
        data = sweep_param(param, lo, hi, steps, base_car, x, y)
        # sensitivity = total lap time variation across range
        times = [t for _, t in data]
        spread = max(times) - min(times)
        sensitivities.append((param, spread, unit, data))

    # Sort by impact
    sensitivities.sort(key=lambda x: x[1], reverse=True)

    print(f"\n  SENSITIVITY RANKING (lap time range across sweep)")
    print(f"  {'parameter':30}  {'impact (s)':>10}  {'unit':>6}")
    print(f"  {'─'*50}")
    for param, spread, unit, _ in sensitivities:
        bar = "█" * int(spread / 0.05)
        print(f"  {param:30}  {spread:>+10.3f}  {unit:>6}  {bar}")

    print()
    for param, spread, unit, data in sensitivities:
        print_sensitivity_table(param, data, baseline, unit)


def main():
    parser = argparse.ArgumentParser(description="FS Lap Sim — Sensitivity Analysis")
    parser.add_argument("--track", default="fsg")
    parser.add_argument("--sweep", default="all", help="all or param name")
    parser.add_argument("--range", default=None, help="lo,hi,steps  (only for single param)")
    args = parser.parse_args()

    car = VehicleParams()
    if Path(args.track).exists():
        from simulator import load_track_csv
        x, y = load_track_csv(args.track)
    else:
        x, y = generate_track(args.track)

    print(f"\n  Track: {args.track}  |  Car: {car.mass_kg} kg / {car.peak_power_kw} kW")

    if args.sweep == "all":
        sweep_all(car, x, y)
    else:
        param = args.sweep
        if args.range:
            lo, hi, steps = [float(v) for v in args.range.split(",")]
            steps = int(steps)
        elif param in SWEEP_PARAMS:
            lo, hi, steps, _ = SWEEP_PARAMS[param]
        else:
            print(f"Unknown param: {param}. Use --sweep all or one of: {list(SWEEP_PARAMS)}")
            return

        baseline = run_baseline(car, x, y)
        data = sweep_param(param, lo, hi, int(steps), car, x, y)
        print_sensitivity_table(param, data, baseline, "")


if __name__ == "__main__":
    main()
