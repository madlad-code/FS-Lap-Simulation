"""
fs_lap_sim.vehicle
Fordonsparametrar och fysikmodeller för en Formula Student-bil.
Alla enheter är i SI (m, kg, s, N, W) om inget annat anges.
"""

from dataclasses import dataclass, field


@dataclass
class TireModel:
    """
    Förenklad Pacejka Magic Formula för beräkning av däckkrafter.
    Modellerar hur däcket genererar kraft baserat på slip och vertikal last.
    """
    # Longitudinella (X) koefficienter
    Bx: float = 10.0   # Styvhetsfaktor
    Cx: float = 1.9    # Formfaktor
    Dx: float = 1.0    # Toppvärde (friktionskoefficient mu)
    Ex: float = 0.97   # Krökningsfaktor

    # Laterala (Y) koefficienter
    By: float = 8.0
    Cy: float = 1.3
    Dy: float = 1.0
    Ey: float = -1.0

    def fx(self, slip_ratio: float, Fz: float) -> float:
        """
        Beräknar longitudinell kraft [N].
        Fz: Vertikal last [N]
        """
        x = self.Bx * slip_ratio
        return Fz * self.Dx * (
            (2 / 3.14159) * (1 - self.Ex / 3)
            * (1 - (self.Ex - 1) / (self.Ex * x + 1e-9) * x)
            * (3.14159 / 2 - (self.Cx - 1) * x / (self.Cx * x + 1e-9))
        )

    def fy(self, slip_angle_rad: float, Fz: float) -> float:
        """
        Beräknar lateral kraft [N].
        Används för att bestämma maximal kurvhastighet.
        """
        alpha = slip_angle_rad
        return Fz * self.Dy * (
            (1 - self.Ey * (self.By * alpha - 1) ** 2)
            * (self.By * alpha)
            / ((self.Cy * self.By * alpha) ** 2 + 1) ** 0.5
        )

    def mu_x(self) -> float:
        """Maximal longitudinell friktionskoefficient."""
        return self.Dx

    def mu_y(self) -> float:
        """Maximal lateral friktionskoefficient."""
        return self.Dy


@dataclass
class VehicleParams:
    """
    Fullständig uppsättning fordonsparametrar.
    Dessa värden representerar en typisk konkurrenskraftig Formula Student-bil.
    """
    # Massa & Tröghet
    mass_kg: float = 220.0          # Total massa inkl. förare [kg]
    driver_mass_kg: float = 75.0

    # Geometri
    wheelbase_m: float = 1.53       # Axelavstånd [m]
    track_front_m: float = 1.22     # Spårvidd fram [m]
    track_rear_m: float = 1.18      # Spårvidd bak [m]
    cg_height_m: float = 0.29       # Tyngdpunkthöjd [m]
    cg_bias_front: float = 0.47     # Viktfördelning (0.47 = 47% på framaxeln)

    # Aerodynamik (Hög downforce-konfiguration)
    cl: float = 2.8                 # Lyftkoefficient (negativ för downforce)
    cd: float = 1.5                 # Luftmotståndskoefficient
    aero_ref_area_m2: float = 0.96  # Frontarea för aero-beräkningar [m^2]

    # Drivlina
    electric: bool = True           # Växla mellan elektrisk och förbränningslogik
    peak_power_kw: float = 80.0     # Maximal uteffekt [kW]
    peak_torque_nm: float = 230.0   # Maximalt vridmoment [Nm]
    final_drive_ratio: float = 3.27
    wheel_radius_m: float = 0.254   # 10-tums hjul är standard i FS

    # Bromssystem
    brake_bias_front: float = 0.60  # Procentuell bromskraft på framaxeln
    max_brake_force_n: float = 6500.0 # Total maximal bromskraft [N]

    # Effektivitet
    drivetrain_efficiency: float = 0.93

    # Däckmodell-instans
    tire: TireModel = field(default_factory=TireModel)

    # Batteri & Återvinning (Endast el)
    battery_capacity_kwh: float = 6.0
    regen_efficiency: float = 0.65  # Effektivitet för energiåtervinning vid bromsning

    @property
    def weight_n(self) -> float:
        """Statisk tyngdkraft [N]."""
        return self.mass_kg * 9.81

    @property
    def weight_front_n(self) -> float:
        """Statisk tyngd på framaxeln [N]."""
        return self.weight_n * self.cg_bias_front

    @property
    def weight_rear_n(self) -> float:
        """Statisk tyngd på bakaxeln [N]."""
        return self.weight_n * (1 - self.cg_bias_front)

    @property
    def max_power_w(self) -> float:
        """Toppeffekt i Watt."""
        return self.peak_power_kw * 1000

    def downforce_n(self, v_ms: float) -> float:
        """Beräknar aerodynamiskt marktryck (downforce) vid hastighet v."""
        rho = 1.225  # Standard luftdensitet vid havsnivå [kg/m^3]
        return 0.5 * rho * v_ms ** 2 * self.cl * self.aero_ref_area_m2

    def drag_n(self, v_ms: float) -> float:
        """Beräknar luftmotstånd vid hastighet v."""
        rho = 1.225
        return 0.5 * rho * v_ms ** 2 * self.cd * self.aero_ref_area_m2

    def max_traction_n(self, v_ms: float) -> float:
        """
        Beräknar maximal tillgänglig drivkraft vid hastighet v.
        Begränsas av antingen däckgrepp (longitudinellt) eller motoreffekt.
        """
        # 1. Greppgräns: Total vertikal kraft * friktionskoefficient
        fz_total = self.weight_n + self.downforce_n(v_ms)
        f_tire = fz_total * self.tire.mu_x() * self.drivetrain_efficiency
        
        # 2. Effektgräns: Kraft = Effekt / Hastighet (P = F*v)
        # Undvik division med noll vid stillastående genom ett litet epsilon
        f_power = self.max_power_w / max(v_ms, 0.5)
        
        return min(f_tire, f_power)

    def max_lateral_accel_ms2(self, v_ms: float) -> float:
        """
        Beräknar maximal möjlig lateral acceleration [m/s^2].
        Kombinerar mekaniskt grepp och aerodynamisk downforce.
        """
        fz_total = self.weight_n + self.downforce_n(v_ms)
        return (fz_total * self.tire.mu_y()) / self.mass_kg
