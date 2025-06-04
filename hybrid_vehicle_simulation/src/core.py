# hybrid_vehicle_simulation/src/core.py
from typing import List, Dict, Union, Optional
import pandas as pd
import math

# --- Engine Class ---
class Engine:
    """
    Represents the internal combustion engine (ICE) of a vehicle.
    This class stores engine-specific parameters like its torque curve and redline RPM,
    and provides methods to calculate torque and power at a given RPM.
    """
    def __init__(self, torque_curve: Union[Dict[int, float], pd.Series], redline_rpm: int):
        """
        Initializes an Engine object.

        Args:
            torque_curve: A dictionary or pandas Series representing the engine's torque curve.
                          The keys/index are RPM values, and values are the corresponding torque in Newton-meters (Nm).
            redline_rpm: The maximum safe operating RPM for the engine.
        """
        if isinstance(torque_curve, dict): # Convert dict to sorted Series
            self.torque_curve = pd.Series(torque_curve).sort_index() if torque_curve else pd.Series(dtype=float)
        elif isinstance(torque_curve, pd.Series):
            self.torque_curve = torque_curve.sort_index() if not torque_curve.index.is_monotonic_increasing else torque_curve
        else: # Default to empty Series if input is neither dict nor Series
            self.torque_curve = pd.Series(dtype=float)
        self.redline_rpm = redline_rpm

    def get_torque(self, rpm: float) -> float:
        """
        Calculates the available engine torque at a given RPM.
        If RPM is outside operational range or torque curve is empty, returns 0.
        Uses closest-point lookup if RPM is not an exact match in the curve.
        """
        if self.torque_curve.empty or rpm <= 0 or rpm > self.redline_rpm:
            return 0.0

        # Simplified: find closest available RPM in the index if not exact.
        # For more accuracy, linear interpolation should be used.
        available_rpms = self.torque_curve.index
        closest_rpm = min(available_rpms, key=lambda x: abs(x - rpm))
        return self.torque_curve.get(closest_rpm, 0.0)

    def get_power(self, rpm: float) -> float:
        """Calculates engine power (kW) at a given RPM."""
        torque_nm = self.get_torque(rpm)
        if rpm <= 0: return 0.0
        return (torque_nm * rpm) / 9549.3

# --- Motor Class ---
class Motor:
    """
    Represents the electric motor of a vehicle.
    Includes torque curve, redline RPM, and max power output.
    Calculates torque considering the motor's power limit.
    """
    def __init__(self, torque_curve: Union[Dict[int, float], pd.Series], redline_rpm: int, max_power_kw: float):
        """
        Initializes a Motor object.

        Args:
            torque_curve: Dictionary or pandas Series for RPM vs. Torque (Nm).
            redline_rpm: Maximum motor RPM.
            max_power_kw: Maximum power output of the motor in kW.
        """
        if isinstance(torque_curve, dict):
            self.torque_curve = pd.Series(torque_curve).sort_index() if torque_curve else pd.Series(dtype=float)
        elif isinstance(torque_curve, pd.Series):
            self.torque_curve = torque_curve.sort_index() if not torque_curve.index.is_monotonic_increasing else torque_curve
        else:
            self.torque_curve = pd.Series(dtype=float)
        self.redline_rpm = redline_rpm
        self.max_power_kw = max_power_kw

    def get_torque(self, rpm: float) -> float:
        """
        Calculates motor torque at RPM, considering its max power limit.
        Returns 0 if RPM > redline or torque curve is empty.
        Handles 0 RPM torque based on curve.
        """
        if self.torque_curve.empty: return 0.0
        if rpm > self.redline_rpm: return 0.0

        base_torque: float
        if rpm <= 0:
            # For motor, 0 RPM torque can be significant and is usually defined in the curve.
            # Get torque at 0 RPM if defined, otherwise use the first point in the curve if available.
            base_torque = self.torque_curve.get(0, self.torque_curve.iloc[0] if not self.torque_curve.empty else 0.0)
            return base_torque # Power limit not applicable at 0 RPM for torque output.

        # Simplified closest-point lookup. Interpolation would be more accurate.
        available_rpms = self.torque_curve.index
        if rpm < available_rpms.min(): # Positive RPM below the start of the curve
             closest_rpm = available_rpms.min()
        else: # RPM within or above defined curve range (up to redline)
            closest_rpm = min(available_rpms, key=lambda x: abs(x - rpm))
        base_torque = self.torque_curve.get(closest_rpm, 0.0)

        # Calculate torque limit based on the motor's maximum power output
        power_limited_torque = (self.max_power_kw * 9549.3) / rpm
        return min(base_torque, power_limited_torque)

    def get_power(self, rpm: float) -> float:
        """Calculates motor power (kW) at RPM, capped by max_power_kw."""
        torque_nm = self.get_torque(rpm) # Torque is already motor-power-limited
        if rpm <= 0: return 0.0
        power_kw = (torque_nm * rpm) / 9549.3
        # Explicitly cap by max_power_kw as a safeguard, though get_torque should handle it.
        return min(power_kw, self.max_power_kw)

# --- Transmission Class ---
class Transmission:
    """Stores gear ratios, final drive, and overall drivetrain efficiency."""
    def __init__(self, gear_ratios: List[float], final_gear_ratio: float, efficiency: float = 0.85):
        """
        Args:
            gear_ratios: List of forward gear ratios.
            final_gear_ratio: Final drive ratio.
            efficiency: Overall drivetrain efficiency (0.0 to 1.0). Default is 0.85.
        """
        if not gear_ratios: raise ValueError("기어비 목록은 비어 있을 수 없습니다.")
        if not (0 < efficiency <= 1.0): raise ValueError("효율은 0보다 크고 1 이하여야 합니다.")
        self.gear_ratios = gear_ratios
        self.final_gear_ratio = final_gear_ratio
        self.efficiency = efficiency # Represents overall drivetrain efficiency

    def get_torque_multiplier(self, gear_number: int) -> float:
        """Calculates torque multiplier for a gear (1-indexed), including efficiency."""
        if 1 <= gear_number <= len(self.gear_ratios):
            return self.gear_ratios[gear_number - 1] * self.final_gear_ratio * self.efficiency
        return 0.0

# --- Battery Class ---
class Battery:
    """Stores battery capacity and max discharge power."""
    def __init__(self, capacity_kwh: float, max_discharge_power_kw: float):
        """
        Args:
            capacity_kwh: Total battery energy capacity (kWh).
            max_discharge_power_kw: Max continuous battery discharge power (kW).
        """
        self.capacity_kwh = capacity_kwh
        self.max_discharge_power_kw = max_discharge_power_kw
        self.current_charge_kwh = capacity_kwh # Assume fully charged

# --- Vehicle Class ---
class Vehicle:
    """Integrates components and calculates vehicle dynamics."""
    def __init__(self,
                 base_mass_kg: float,
                 frontal_area_m2: float,
                 drag_coefficient: float,
                 rolling_resistance_coefficient: float,
                 tire_radius_m: float,
                 transmission: Transmission,
                 engine: Optional[Engine] = None,
                 motor: Optional[Motor] = None,
                 battery: Optional[Battery] = None,
                 drivetrain_type: str = "FWD",
                 additional_mass_kg: float = 0.0,
                 max_total_torque_nm: Optional[float] = None
                ):
        """
        Args:
            base_mass_kg: Base mass of the vehicle (kg).
            additional_mass_kg: Additional mass (payload, etc.) (kg). Default 0.
            max_total_torque_nm: Optional cap for the combined torque from engine and motor (Nm).
            (Other args as before)
        """
        self.mass_kg = base_mass_kg + additional_mass_kg # Total vehicle mass
        self.frontal_area_m2 = frontal_area_m2
        self.drag_coefficient = drag_coefficient
        self.rolling_resistance_coefficient = rolling_resistance_coefficient
        self.tire_radius_m = tire_radius_m
        self.transmission = transmission # Assumed initialized with overall drivetrain efficiency
        self.engine = engine
        self.motor = motor
        self.battery = battery
        self.drivetrain_type = drivetrain_type
        self.max_total_torque_nm = max_total_torque_nm

        if self.motor and not self.battery:
            raise ValueError("모터가 있는 경우 배터리 객체가 반드시 제공되어야 합니다.")

        self.gravity = 9.81  # m/s^2
        self.air_density = 1.225  # kg/m^3

    def load_torque_curves_from_csv(self, engine_csv_path: Optional[str] = None, motor_csv_path: Optional[str] = None):
        """Loads torque curves from CSV files for engine and/or motor."""
        if engine_csv_path and self.engine:
            try:
                engine_df = pd.read_csv(engine_csv_path)
                engine_df = engine_df.sort_values(by='RPM')
                self.engine.torque_curve = pd.Series(engine_df['Torque'].values, index=engine_df['RPM'].values)
                print(f"엔진 토크 커브 로드: {engine_csv_path}")
            except Exception as e: print(f"엔진 토크 커브 로드 오류 {engine_csv_path}: {e}")

        if motor_csv_path and self.motor:
            try:
                motor_df = pd.read_csv(motor_csv_path)
                motor_df = motor_df.sort_values(by='RPM')
                self.motor.torque_curve = pd.Series(motor_df['Torque'].values, index=motor_df['RPM'].values)
                print(f"모터 토크 커브 로드: {motor_csv_path}")
            except Exception as e: print(f"모터 토크 커브 로드 오류 {motor_csv_path}: {e}")

    def get_combined_torque(self, engine_rpm: float, motor_rpm: float, requested_throttle_percentage: float = 1.0, mode: str = "hybrid") -> float:
        """
        Calculates combined torque from engine/motor based on mode, throttle, and limits.
        Applies overall max_total_torque_nm cap if set.
        """
        engine_torque_contrib = 0.0
        if self.engine and mode in ["hybrid", "engine_only"]:
            engine_base_torque = self.engine.get_torque(engine_rpm)
            engine_torque_contrib = engine_base_torque * requested_throttle_percentage

        motor_torque_contrib = 0.0
        if self.motor and self.battery and mode in ["hybrid", "motor_only"]:
            # Motor torque from its curve (already considers motor's own max_power_kw)
            motor_curve_torque = self.motor.get_torque(motor_rpm)
            throttled_motor_torque = motor_curve_torque * requested_throttle_percentage

            torque_limit_from_battery = float('inf')
            if motor_rpm > 0 and self.battery.max_discharge_power_kw > 0:
                 torque_limit_from_battery = (self.battery.max_discharge_power_kw * 9549.3) / motor_rpm
            elif self.battery.max_discharge_power_kw <= 0 :
                 torque_limit_from_battery = 0.0 # No contribution if battery can't discharge
            motor_torque_contrib = min(throttled_motor_torque, torque_limit_from_battery)

        current_total_torque: float
        if mode == "engine_only":
            current_total_torque = engine_torque_contrib
        elif mode == "motor_only":
            current_total_torque = motor_torque_contrib
        elif mode == "hybrid":
            current_total_torque = engine_torque_contrib + motor_torque_contrib
        else:
            current_total_torque = 0.0 # Default for unknown mode

        # Apply overall max total torque cap if specified
        if self.max_total_torque_nm is not None:
            if current_total_torque > self.max_total_torque_nm:
                current_total_torque = self.max_total_torque_nm
            # Assuming propulsion torque is positive. If negative torque (regen) is considered,
            # this cap might need to be handled differently for braking.

        return current_total_torque

    def calculate_resistive_forces(self, velocity_mps: float, road_gradient_percent: float = 0.0) -> float:
        """Calculates total resistive forces: drag, rolling, and grade resistance."""
        drag_force = 0.5 * self.air_density * self.frontal_area_m2 * self.drag_coefficient * (velocity_mps ** 2)
        rolling_resistance_force = self.rolling_resistance_coefficient * self.mass_kg * self.gravity

        # Grade Resistance Force: F_grade = m * g * sin(theta)
        # theta = arctan(gradient_percent / 100)
        grade_force = self.mass_kg * self.gravity * math.sin(math.atan(road_gradient_percent / 100.0))

        return drag_force + rolling_resistance_force + grade_force

    def calculate_wheel_rpm(self, vehicle_speed_mps: float) -> float:
        """Calculates wheel RPM from vehicle speed and tire radius."""
        if self.tire_radius_m <= 0: return 0.0
        return (vehicle_speed_mps / self.tire_radius_m) * (60 / (2 * math.pi))

    def _calculate_source_rpm(self, vehicle_speed_mps: float, gear_number: int) -> float:
        """Internal helper for engine/motor RPM based on speed and gear."""
        if self.tire_radius_m <= 0 or not self.transmission or not (1 <= gear_number <= len(self.transmission.gear_ratios)): return 0.0
        overall_ratio = self.transmission.gear_ratios[gear_number - 1] * self.transmission.final_gear_ratio
        if overall_ratio == 0: return 0.0
        return self.calculate_wheel_rpm(vehicle_speed_mps) * overall_ratio

    def calculate_engine_rpm(self, vehicle_speed_mps: float, gear_number: int) -> float:
        """Calculates engine RPM if an engine is present."""
        if not self.engine: return 0.0
        return self._calculate_source_rpm(vehicle_speed_mps, gear_number)

    def calculate_motor_rpm(self, vehicle_speed_mps: float, gear_number: int) -> float:
        """Calculates motor RPM if a motor is present."""
        if not self.motor: return 0.0
        return self._calculate_source_rpm(vehicle_speed_mps, gear_number)

    def calculate_tractive_force(self, combined_input_torque_to_transmission: float, gear_number: int) -> float:
        """Calculates tractive force at wheels from torque input to transmission."""
        if self.tire_radius_m <= 0 or not self.transmission: return 0.0
        # Transmission efficiency is now part of get_torque_multiplier from Transmission class
        torque_multiplier = self.transmission.get_torque_multiplier(gear_number)
        return (combined_input_torque_to_transmission * torque_multiplier) / self.tire_radius_m
