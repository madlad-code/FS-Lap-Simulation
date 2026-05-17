"""
fs_lap_sim.vehicle
Vehicle parameters for Formula Student car.
All SI units unless noted.
"""

from dataclasses import dataclass, field


@dataclass
class TireModel:
    """Simplified Pacejka Magic Formula (lateral + longitudinal)."""
    Bx: float = 10.0   # stiffness factor longitudinal
    Cx: float = 1.9
    Dx: float = 1.0    # peak value (mu)
    Ex: float = 0.97   # shape

    By: float = 8.0
    Cy: float = 1.3
    Dy: float = 1.0
    Ey: float = -1.0

    def fx(self, slip_ratio: float, Fz: float) -> float:
        """Longitudinal force [N]. Fz in N."""
        x = self.Bx * slip_ratio
        return Fz * self.Dx * (
            (2 / 3.14159) * (1 - self.Ex / 3)
            * (1 - (self.Ex - 1) / (self.Ex * x + 1e-9) * x)
            * (3.14159 / 2 - (self.Cx - 1) * x / (self.Cx * x + 1e-9))
        )

    def fy(self, slip_angle_rad: float, Fz: float) -> float:
        """Lateral force [N]."""
        alpha = slip_angle_rad
        return Fz * self.Dy * (
            (1 - self.Ey * (self.By * alpha - 1) ** 2)
            * (self.By * alpha)
            / ((self.Cy * self.By * alpha) ** 2 + 1) ** 0.5
        )

    def mu_x(self) -> float:
        return self.Dx

    def mu_y(self) -> float:
        return self.Dy


@dataclass
class VehicleParams:
    """
    Complete FS vehicle parameter set.
    Adjust for your specific car — these are reasonable FS defaults.
    """
    # Mass
    mass_kg: float = 220.0          # total with driver
    driver_mass_kg: float = 75.0

    # Geometry
    wheelbase_m: float = 1.53
    track_front_m: float = 1.22
    track_rear_m: float = 1.18
    cg_height_m: float = 0.29
    cg_bias_front: float = 0.47     # fraction of weight on front axle

    # Aero (FS-spec with wings)
    cl: float = 2.8                 # lift coefficient (downforce)
    cd: float = 1.5                 # drag coefficient
    aero_ref_area_m2: float = 0.96  # reference area [m²]

    # Powertrain (combustion or electric flag)
    electric: bool = True
    peak_power_kw: float = 80.0
    peak_torque_nm: float = 230.0
    final_drive_ratio: float = 3.27
    wheel_radius_m: float = 0.254   # 10" wheel

    # Braking
    brake_bias_front: float = 0.60
    max_brake_force_n: float = 6500.0

    # Drivetrain losses
    drivetrain_efficiency: float = 0.93

    # Tire
    tire: TireModel = field(default_factory=TireModel)

    # Energy (electric only)
    battery_capacity_kwh: float = 6.0
    regen_efficiency: float = 0.65

    @property
    def weight_n(self) -> float:
        return self.mass_kg * 9.81

    @property
    def weight_front_n(self) -> float:
        return self.weight_n * self.cg_bias_front

    @property
    def weight_rear_n(self) -> float:
        return self.weight_n * (1 - self.cg_bias_front)

    @property
    def max_power_w(self) -> float:
        return self.peak_power_kw * 1000

    def downforce_n(self, v_ms: float) -> float:
        """Beräkna aerodynamisk marktryck (downforce) vid hastighet v."""
        rho = 1.225  # kg/m³ standard luftdensitet
        return 0.5 * rho * v_ms ** 2 * self.cl * self.aero_ref_area_m2

    def drag_n(self, v_ms: float) -> float:
        """Beräkna luftmotstånd vid hastighet v."""
        rho = 1.225
        return 0.5 * rho * v_ms ** 2 * self.cd * self.aero_ref_area_m2

    def max_traction_n(self, v_ms: float) -> float:
        """Maximal drivkraft begränsad av antingen däckens grepp eller motorns effekt."""
        # Totalt vertikalt tryck = Vikt + Downforce
        fz_total = self.weight_n + self.downforce_n(v_ms)
        # Greppgräns: Friktion * Normalkraft
        f_tire = fz_total * self.tire.mu_x() * self.drivetrain_efficiency
        # Effektgräns: Kraft = Effekt / Hastighet (P = Fv)
        f_power = self.max_power_w / max(v_ms, 0.5)
        return min(f_tire, f_power)

    def max_lateral_accel_ms2(self, v_ms: float) -> float:
        """Max lateral acceleration [m/s²]."""
        fz_total = self.weight_n + self.downforce_n(v_ms)
        return (fz_total * self.tire.mu_y()) / self.mass_kg
