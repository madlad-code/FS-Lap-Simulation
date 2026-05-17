"""
fs_lap_sim.simulator
Quasi-steady-state varvtidssimulering.
Punktmassemodell — tillräckligt noggrann för FS-bananalys.

Metod:
  1. Parsa bana från GPS/koner eller parametrisk definition.
  2. Beräkna lokal krökning κ(s).
  3. Hitta maxfart vid varje punkt utifrån lateral greppgräns.
  4. Forward-pass: begränsa acceleration utifrån drivkraft.
  5. Backward-pass: begränsa deacceleration utifrån bromsförmåga.
  6. Integrera tid, energi och krafter.

Användning:
    python simulator.py --track data/fsg_endurance.csv --car configs/default.json
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from vehicle import VehicleParams, TireModel


# ── BANA ─────────────────────────────────────────────────────────────────────

def load_track_csv(path: str) -> tuple[np.ndarray, np.ndarray]:
    """
    Ladda bana från CSV med kolumner: x,y (meter, kartesiskt).
    Returnerar (x, y) arrayer.
    """
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    return data[:, 0], data[:, 1]


def generate_track(name: str) -> tuple[np.ndarray, np.ndarray]:
    """
    Inbyggda parametriska banor för testning.
    """
    t = np.linspace(0, 2 * np.pi, 500, endpoint=False)

    if name == "oval":
        x = 50 * np.cos(t)
        y = 25 * np.sin(t)
    elif name == "figure8":
        x = 40 * np.sin(t)
        y = 40 * np.sin(t) * np.cos(t)
    elif name == "fsg":
        # Approximativ FSG endurance-layout
        x = (60 * np.cos(t) + 20 * np.cos(2 * t))
        y = (40 * np.sin(t) + 15 * np.sin(3 * t))
    elif name == "skidpad":
        # FS skid pad: två cirklar med radie 9.125 m
        half = len(t) // 2
        x = np.concatenate([9.125 * np.cos(t[:half]), 9.125 * np.cos(t[half:]) + 18.25])
        y = np.concatenate([9.125 * np.sin(t[:half]), 9.125 * np.sin(t[half:])])
    else:
        raise ValueError(f"Okänd bana: {name}")

    return x, y


def compute_curvature(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Beräkna båglängd s och krökning κ vid varje punkt.
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


# ── LÖSARE ────────────────────────────────────────────────────────────────────

class LapSimulator:
    """
    Simuleringens huvudmotor.
    Använder en Quasi-Steady-State (QSS) metod för att hitta minsta varvtid.
    """
    def __init__(self, car: VehicleParams):
        self.car = car

    def v_corner_max(self, kappa: np.ndarray) -> np.ndarray:
        """
        Beräknar maximal möjlig hastighet vid varje punkt på banan
        baserat enbart på lateralt grepp (greppgränsen).
        
        v = sqrt(a_lat_max / κ), där κ är krökningen.
        """
        car = self.car
        v_max = np.zeros_like(kappa)
        for i, k in enumerate(kappa):
            if k < 1e-4:
                # Praktiskt taget en raksträcka
                v_max[i] = 80.0
            else:
                # Iterativ lösare: lateralt grepp beror på hastighet (pga aero downforce),
                # så vi löser för v tills det stabiliseras.
                v0 = np.sqrt(car.max_lateral_accel_ms2(0) / k)
                for _ in range(5):
                    a_lat = car.max_lateral_accel_ms2(v0)
                    v0 = min(np.sqrt(a_lat / k), 80.0)
                v_max[i] = v0
        return v_max

    def run(self, x: np.ndarray, y: np.ndarray) -> dict:
        """
        Kör den fullständiga varvtidssimuleringen med forward och backward passes.
        """
        car = self.car
        s, kappa = compute_curvature(x, y)
        n = len(s)
        ds = np.diff(s, append=s[-1] - s[-2])  # Avstånd mellan punkter

        # Beräkna det absoluta taket för hastighet baserat på kurvor
        v_limit = self.v_corner_max(kappa)

        # ── Forward pass (Accelerations-pass)
        # Beräknar hur snabbt bilen kan köra givet dess effekt och drivgrepp.
        v = np.zeros(n)
        v[0] = min(v_limit[0], 5.0)  # Anta en låg startfart (t.ex. ut ur en kurva)

        for i in range(1, n):
            seg = ds[i - 1]
            f_drive = car.max_traction_n(v[i - 1]) # Motor/däck greppgräns
            f_drag = car.drag_n(v[i - 1])           # Luftmotstånd
            f_net = f_drive - f_drag
            a = f_net / car.mass_kg                 # Newtons andra lag: F = ma
            
            # v^2 = u^2 + 2as => Beräkna hastighet vid nästa punkt
            v_new = np.sqrt(max(v[i - 1] ** 2 + 2 * a * seg, 0.01))
            v[i] = min(v_new, v_limit[i])           # Kan inte överskrida kurvgränsen

        # ── Backward pass (Broms-pass)
        # Justerar hastighetsprofilen för att säkerställa att bilen kan sakta ner inför kurvor.
        for i in range(n - 2, -1, -1):
            seg = ds[i]
            f_brake = car.max_brake_force_n         # Mekanisk bromsgräns
            f_drag = car.drag_n(v[i + 1])           # Luftmotstånd hjälper till vid inbromsning
            f_net_brake = f_brake + f_drag
            a_brake = f_net_brake / car.mass_kg
            
            # Beräkna vad ingångshastigheten måste vara för att nå nästa punkts hastighet
            v_back = np.sqrt(max(v[i + 1] ** 2 + 2 * a_brake * seg, 0.01))
            v[i] = min(v[i], v_back, v_limit[i])    # Sluthastigheten är den mest restriktiva

        # ── Tidintegrering
        # t = s / v_avg
        dt = np.where(
            (v[:-1] + v[1:]) > 0,
            2 * ds[:-1] / (v[:-1] + v[1:]),
            0.0,
        )
        t_total = float(np.sum(dt))
        t_cum = np.concatenate([[0], np.cumsum(dt)])

        # ── Telemetriberäkning
        a_long = np.gradient(v, s)
        a_lat = v ** 2 * kappa

        # ── Energiförbrukning
        energy_drive_j = 0.0
        energy_regen_j = 0.0

        for i in range(1, n):
            v_avg = 0.5 * (v[i - 1] + v[i])
            # Räkna bara energi vid acceleration
            f_drive = max(car.max_traction_n(v_avg) * (v[i] > v[i - 1]), 0)
            # Räkna regen vid inbromsning (om elektrisk)
            f_brake_active = car.max_brake_force_n * (v[i] < v[i - 1])
            
            energy_drive_j += f_drive * ds[i - 1]
            if car.electric:
                energy_regen_j += f_brake_active * ds[i - 1] * car.regen_efficiency

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


# ── RAPPORT ──────────────────────────────────────────────────────────────────

def print_report(result: dict, car: VehicleParams) -> None:
    lap = result["lap_time_s"]
    mins = int(lap // 60)
    secs = lap % 60

    print()
    print("=" * 52)
    print("  FS LAP SIMULATION RESULTAT")
    print("=" * 52)
    print(f"  Varvtid           {mins}:{secs:06.3f}")
    print(f"  Banlängd          {result['track_length_m']:.1f} m")
    print(f"  v_max             {result['v_max_ms'] * 3.6:.1f} km/h")
    print(f"  v_avg             {result['v_avg_ms'] * 3.6:.1f} km/h")
    print(f"  Energi (netto)    {result['energy_net_kwh']:.3f} kWh")
    print(f"  SoC kvar          {100*(car.battery_capacity_kwh - result['energy_net_kwh'])/car.battery_capacity_kwh:.1f}%")
    print(f"  Simulerade punkter {result['n_points']}")
    print("=" * 52)
    print()


def export_telemetry(result: dict, path: str) -> None:
    """Exportera telemetri punkt-för-punkt som CSV."""
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
    print(f"  Telemetri → {path}")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="FS Lap Time Simulator")
    parser.add_argument("--track", default="fsg", help="Bana: fsg|oval|figure8|skidpad eller sökväg/till/bana.csv")
    parser.add_argument("--car", default=None, help="Sökväg till bil-konfiguration (JSON)")
    parser.add_argument("--export", default=None, help="Exportera telemetri till CSV")
    parser.add_argument("--laps", type=int, default=1, help="Antal varv att simulera")
    args = parser.parse_args()

    # Ladda bil
    car = VehicleParams()
    if args.car:
        with open(args.car) as f:
            cfg = json.load(f)
        for k, v in cfg.items():
            if hasattr(car, k):
                setattr(car, k, v)

    # Ladda bana
    if Path(args.track).exists():
        x, y = load_track_csv(args.track)
    else:
        x, y = generate_track(args.track)

    print(f"\n  Bana: {args.track}  |  Bil: {car.mass_kg} kg  |  Toppeffekt: {car.peak_power_kw} kW")

    t0 = time.perf_counter()
    sim = LapSimulator(car)
    result = sim.run(x, y)
    elapsed = time.perf_counter() - t0

    print_report(result, car)
    print(f"  Simuleringstid    {elapsed*1000:.1f} ms")

    if args.export:
        export_telemetry(result, args.export)

    if args.laps > 1:
        print(f"\n  {args.laps}-varvs projektion: {result['lap_time_s'] * args.laps / 60:.2f} min")
        print(f"  Energiprojektion: {result['energy_net_kwh'] * args.laps:.3f} kWh")


if __name__ == "__main__":
    main()
