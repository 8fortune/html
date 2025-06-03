# hybrid_vehicle_simulation/src/core.py
from typing import List, Dict, Union, Optional
import pandas as pd
import math

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
                          Example: {1000: 150, 2000: 200, 3000: 220} or pd.Series([150, 200, 220], index=[1000, 2000, 3000]).
            redline_rpm: The maximum safe operating RPM for the engine.
        """
        self.torque_curve = torque_curve
        self.redline_rpm = redline_rpm
        # TODO: Implement proper torque interpolation, especially if 'torque_curve' is a dictionary.
        # Currently, for dictionaries, it finds the closest RPM key. For pd.Series, it also uses closest.
        # A linear interpolation would be more accurate for RPM values between defined points.

    def get_torque(self, rpm: float) -> float:
        """
        Calculates the available engine torque at a given RPM.

        If the requested RPM is outside the operational range (below 0 or above redline),
        torque is considered zero. If RPM is below the lowest point in the torque curve
        but positive, the torque at the lowest defined RPM is returned (simplification).
        A more advanced implementation should interpolate torque values.

        Args:
            rpm: The current engine RPM.

        Returns:
            The engine torque (Nm) at the specified RPM. Returns 0.0 if RPM is invalid or
            above redline.
        """
        if rpm > self.redline_rpm:
            return 0.0  # RPM exceeds redline
        if rpm <= 0:
            return 0.0  # Engine off or invalid RPM

        # Current implementation: Find torque at the closest RPM point defined in the curve.
        # This is a simplification. Linear interpolation would be more realistic.
        if isinstance(self.torque_curve, pd.Series):
            if self.torque_curve.empty:
                return 0.0
            # Find the closest RPM in the Series index
            available_rpms = self.torque_curve.index
            # Ensure available_rpms is not empty before calling min/max or accessing elements
            if not available_rpms.is_monotonic_increasing:
                 self.torque_curve = self.torque_curve.sort_index() # Ensure sorted for consistent closest search
                 available_rpms = self.torque_curve.index

            if available_rpms.empty: return 0.0

            # Find closest available RPM (handles out-of-range by effectively clamping to min/max of curve for positive RPMs)
            closest_rpm = min(available_rpms, key=lambda x: abs(x - rpm))
            return self.torque_curve.get(closest_rpm, 0.0)
        elif isinstance(self.torque_curve, dict):
            if not self.torque_curve:
                return 0.0
            # Find the closest RPM key in the dictionary
            closest_rpm_key = min(self.torque_curve.keys(), key=lambda k: abs(k - rpm))
            return self.torque_curve.get(closest_rpm_key, 0.0)

        return 0.0 # Should not be reached if torque_curve is Series or Dict

    def get_power(self, rpm: float) -> float:
        """
        Calculates the engine power in kilowatts (kW) at a given RPM.

        Power is derived from torque using the formula: Power (kW) = Torque (Nm) * RPM / 9549.3.

        Args:
            rpm: The current engine RPM.

        Returns:
            The engine power (kW) at the specified RPM.
        """
        torque_nm = self.get_torque(rpm)
        if rpm <= 0:  # Cannot produce power at 0 or negative RPM
            return 0.0
        power_kw = (torque_nm * rpm) / 9549.3
        return power_kw

class Motor:
    """
    Represents the electric motor of a vehicle.
    This class stores motor-specific parameters like its torque curve, redline RPM,
    and maximum power output. It provides methods to calculate torque and power,
    considering the motor's power limit.
    """
    def __init__(self, torque_curve: Union[Dict[int, float], pd.Series], redline_rpm: int, max_power_kw: float):
        """
        Initializes a Motor object.

        Args:
            torque_curve: A dictionary or pandas Series representing the motor's torque curve (RPM vs. Torque in Nm).
                          Example: {0: 150, 1000: 140, 2000: 120} or pd.Series([150,140,120], index=[0,1000,2000]).
            redline_rpm: The maximum safe operating RPM for the motor.
            max_power_kw: The maximum power output of the motor in kilowatts (kW).
        """
        self.torque_curve = torque_curve
        self.redline_rpm = redline_rpm
        self.max_power_kw = max_power_kw
        # TODO: Implement proper torque interpolation for dictionary-based torque curves.

    def get_torque(self, rpm: float) -> float:
        """
        Calculates the available motor torque at a given RPM, considering its maximum power limit.

        The torque obtained from the curve is compared with the torque calculated based on
        `max_power_kw` at the given RPM. The lower of the two values is returned.
        If RPM is above redline, torque is zero. Handles 0 RPM appropriately.

        Args:
            rpm: The current motor RPM.

        Returns:
            The motor torque (Nm) at the specified RPM, limited by `max_power_kw`.
        """
        if rpm > self.redline_rpm:
            return 0.0 # RPM exceeds redline

        # Determine base torque from the curve (simplified lookup, needs interpolation for accuracy)
        base_torque = 0.0
        if isinstance(self.torque_curve, pd.Series):
            if self.torque_curve.empty:
                return 0.0
            available_rpms = self.torque_curve.index
            if not available_rpms.is_monotonic_increasing:
                 self.torque_curve = self.torque_curve.sort_index()
                 available_rpms = self.torque_curve.index
            if available_rpms.empty: return 0.0
            # Find closest RPM for torque lookup
            closest_rpm = min(available_rpms, key=lambda x: abs(x-rpm))
            base_torque = self.torque_curve.get(closest_rpm, 0.0)
        elif isinstance(self.torque_curve, dict):
            if not self.torque_curve:
                return 0.0
            closest_rpm_key = min(self.torque_curve.keys(), key=lambda k: abs(k - rpm))
            base_torque = self.torque_curve.get(closest_rpm_key, 0.0)

        if rpm <= 0:
            # Motor can provide torque at 0 RPM (e.g., holding torque or during initial launch).
            # For simplicity, return the torque defined at or near 0 RPM in the curve.
            # Power limit calculation below is not applicable for 0 RPM.
            return base_torque

        # Calculate torque limit based on the motor's maximum power output
        # Power (kW) = Torque (Nm) * RPM / 9549.3  => Torque_max (Nm) = Power_max (kW) * 9549.3 / RPM
        power_limited_torque = (self.max_power_kw * 9549.3) / rpm

        # Return the minimum of the curve-defined torque and the power-limited torque
        return min(base_torque, power_limited_torque)

    def get_power(self, rpm: float) -> float:
        """
        Calculates the motor power in kilowatts (kW) at a given RPM.

        The power is derived from the torque (which already considers `max_power_kw`)
        and then explicitly capped by `max_power_kw` again to ensure consistency.

        Args:
            rpm: The current motor RPM.

        Returns:
            The motor power (kW) at the specified RPM.
        """
        torque_nm = self.get_torque(rpm) # This torque is already power-limited by the motor's max_power_kw
        if rpm <= 0:
            return 0.0 # No power at or below 0 RPM

        # Calculate power from the (potentially limited) torque
        power_kw = (torque_nm * rpm) / 9549.3

        # Ensure the calculated power does not exceed the motor's maximum power rating.
        # This acts as a safeguard, as get_torque should have already limited the torque.
        return min(power_kw, self.max_power_kw)

class Transmission:
    """
    Represents the vehicle's transmission system.
    This class stores transmission parameters like gear ratios, final drive ratio,
    and efficiency. It provides methods to calculate torque multiplication.
    """
    def __init__(self, gear_ratios: List[float], final_gear_ratio: float, efficiency: float = 0.9):
        """
        Initializes a Transmission object.

        Args:
            gear_ratios: A list of forward gear ratios (e.g., [3.5, 2.0, 1.5, 1.0, 0.7]).
                         The list should not include reverse or neutral. The first element
                         (index 0) is considered 1st gear.
            final_gear_ratio: The final drive (differential) ratio.
            efficiency: The mechanical efficiency of the transmission (0.0 to 1.0).
                        Defaults to 0.9 (90%).
        """
        if not gear_ratios:
            raise ValueError("Gear ratios list cannot be empty.")
        self.gear_ratios = gear_ratios  # Example: [2.97 (1st), 2.07 (2nd), ..., 0.56 (6th)]
        self.final_gear_ratio = final_gear_ratio
        self.efficiency = efficiency
        self.current_gear = 1  # Default to 1st gear for simulation state; 0 could represent neutral.

    def get_torque_multiplier(self, gear_number: int) -> float:
        """
        Calculates the overall torque multiplier for a given gear number (1-indexed).

        The multiplier considers the specified gear ratio, the final drive ratio,
        and the transmission's mechanical efficiency.

        Args:
            gear_number: The selected gear number (1 for 1st gear, 2 for 2nd, etc.).

        Returns:
            The total torque multiplication factor. Returns 0.0 if the gear_number
            is invalid (e.g., 0 or out of the defined range).
        """
        if 1 <= gear_number <= len(self.gear_ratios):
            # Overall ratio = specific gear ratio * final drive ratio
            # Effective torque multiplication also considers efficiency
            return self.gear_ratios[gear_number - 1] * self.final_gear_ratio * self.efficiency
        return 0.0  # Return 0 for invalid gears (e.g., neutral or out of bounds)

class Battery:
    """
    Represents the vehicle's battery pack.
    This class stores battery parameters like capacity and maximum discharge power.
    Future enhancements could include state of charge (SoC) tracking and charging/discharging logic.
    """
    def __init__(self, capacity_kwh: float, max_discharge_power_kw: float):
        """
        Initializes a Battery object.

        Args:
            capacity_kwh: The total energy capacity of the battery in kilowatt-hours (kWh).
            max_discharge_power_kw: The maximum continuous power the battery can discharge in kilowatts (kW).
                                    This can limit the motor's output if the motor's demand exceeds this.
        """
        self.capacity_kwh = capacity_kwh
        self.max_discharge_power_kw = max_discharge_power_kw
        self.current_charge_kwh = capacity_kwh  # Assume battery is fully charged initially

    # Placeholder for future methods:
    # def discharge(self, power_kw: float, duration_seconds: float):
    #     """Simulates discharging the battery."""
    #     energy_drawn_kwh = power_kw * (duration_seconds / 3600.0)
    #     self.current_charge_kwh -= energy_drawn_kwh
    #     self.current_charge_kwh = max(0, self.current_charge_kwh) # Prevent negative charge

    # def charge(self, power_kw: float, duration_seconds: float):
    #     """Simulates charging the battery."""
    #     energy_added_kwh = power_kw * (duration_seconds / 3600.0)
    #     self.current_charge_kwh += energy_added_kwh
    #     self.current_charge_kwh = min(self.capacity_kwh, self.current_charge_kwh) # Do not exceed capacity

    # def get_soc_percentage(self) -> float:
    #     """Calculates the current State of Charge (SoC) in percentage."""
    #     return (self.current_charge_kwh / self.capacity_kwh) * 100.0 if self.capacity_kwh > 0 else 0.0

class Vehicle:
    """
    Represents the entire hybrid vehicle, integrating all its components and physical properties.
    This class orchestrates the interactions between engine, motor, battery, and transmission,
    and calculates vehicle-level dynamics like RPMs, forces, and overall performance.
    """
    def __init__(self,
                 mass_kg: float,
                 frontal_area_m2: float,
                 drag_coefficient: float,
                 rolling_resistance_coefficient: float,
                 tire_radius_m: float,
                 transmission: 'Transmission',
                 engine: 'Optional[Engine]' = None,
                 motor: 'Optional[Motor]' = None,
                 battery: 'Optional[Battery]' = None,
                 drivetrain_type: str = "FWD"
                ):
        """
        Initializes a Vehicle object.

        Args:
            mass_kg: Total mass of the vehicle in kilograms.
            frontal_area_m2: Frontal area of the vehicle in square meters, used for aerodynamic drag calculation.
            drag_coefficient: Aerodynamic drag coefficient (dimensionless).
            rolling_resistance_coefficient: Coefficient of rolling resistance (dimensionless).
            tire_radius_m: Effective radius of the driven tires in meters.
            transmission: An instance of the Transmission class.
            engine: An optional instance of the Engine class. Required for simulations involving an engine.
            motor: An optional instance of the Motor class. Required for simulations involving an electric motor.
            battery: An optional instance of the Battery class. Required if a motor is present.
            drivetrain_type: Type of drivetrain (e.g., "FWD", "RWD", "AWD"). Currently informational.
                             Defaults to "FWD".

        Raises:
            ValueError: If a Motor is provided without a Battery.
        """
        self.mass_kg = mass_kg
        self.frontal_area_m2 = frontal_area_m2
        self.drag_coefficient = drag_coefficient
        self.rolling_resistance_coefficient = rolling_resistance_coefficient
        self.tire_radius_m = tire_radius_m
        self.transmission = transmission
        self.engine = engine
        self.motor = motor
        self.battery = battery
        self.drivetrain_type = drivetrain_type # Informational for now, could be used for AWD torque split etc.

        # Validate that a battery is present if a motor is specified
        if self.motor and not self.battery:
            raise ValueError("A Battery object must be provided if a Motor object is present.")

        # Physical constants
        self.gravity = 9.81  # Acceleration due to gravity (m/s^2)
        self.air_density = 1.225  # Standard air density at sea level (kg/m^3)

    def load_torque_curves_from_csv(self, engine_csv_path: Optional[str] = None, motor_csv_path: Optional[str] = None):
        """
        Loads engine and/or motor torque curves from CSV files.

        Each CSV file should have two columns: 'RPM' and 'Torque'.
        The data will be loaded into the respective engine or motor object's torque_curve attribute
        as a pandas Series, sorted by RPM.

        Args:
            engine_csv_path: Optional path to the CSV file for the engine torque curve.
            motor_csv_path: Optional path to the CSV file for the motor torque curve.
        """
        if engine_csv_path and self.engine:
            try:
                engine_df = pd.read_csv(engine_csv_path)
                engine_df = engine_df.sort_values(by='RPM') # Ensure data is sorted by RPM
                self.engine.torque_curve = pd.Series(engine_df['Torque'].values, index=engine_df['RPM'].values)
                print(f"Engine torque curve loaded successfully from {engine_csv_path}")
            except Exception as e:
                print(f"Error loading engine torque curve from {engine_csv_path}: {e}")

        if motor_csv_path and self.motor:
            try:
                motor_df = pd.read_csv(motor_csv_path)
                motor_df = motor_df.sort_values(by='RPM') # Ensure data is sorted by RPM
                self.motor.torque_curve = pd.Series(motor_df['Torque'].values, index=motor_df['RPM'].values)
                print(f"Motor torque curve loaded successfully from {motor_csv_path}")
            except Exception as e:
                print(f"Error loading motor torque curve from {motor_csv_path}: {e}")

    def calculate_wheel_rpm(self, vehicle_speed_mps: float) -> float:
        """
        Calculates the rotational speed of the vehicle's wheels at a given vehicle speed.

        Args:
            vehicle_speed_mps: The linear speed of the vehicle in meters per second.

        Returns:
            The wheel speed in revolutions per minute (RPM). Returns 0.0 if tire radius is not positive.
        """
        if self.tire_radius_m <= 0:
            return 0.0 # Avoid division by zero or invalid calculation
        # Wheel angular velocity (rad/s) = vehicle_speed_mps / tire_radius_m
        # Convert rad/s to RPM: RPM = (rad/s) * (60 seconds/minute) / (2 * pi radians/revolution)
        return (vehicle_speed_mps / self.tire_radius_m) * (60 / (2 * math.pi))

    def _calculate_source_rpm(self, vehicle_speed_mps: float, gear_number: int) -> float:
        """
        Internal helper to calculate engine or motor RPM based on vehicle speed and gear.

        This method is used by `calculate_engine_rpm` and `calculate_motor_rpm`.
        It assumes the power source (engine or motor) is directly connected to the transmission input.

        Args:
            vehicle_speed_mps: The linear speed of the vehicle in meters per second.
            gear_number: The current transmission gear (1-indexed).

        Returns:
            The calculated RPM of the power source. Returns 0.0 if inputs are invalid
            (e.g., non-positive tire radius, invalid gear).
        """
        if self.tire_radius_m <= 0 or not self.transmission or not (1 <= gear_number <= len(self.transmission.gear_ratios)):
            return 0.0

        # Overall gear ratio for the selected gear
        # This does not include transmission efficiency as RPM is a kinematic property.
        overall_gear_ratio = self.transmission.gear_ratios[gear_number - 1] * self.transmission.final_gear_ratio
        if overall_gear_ratio == 0: # Should not happen with valid gear ratios
            return 0.0

        # Source RPM = Wheel RPM * Overall Gear Ratio
        wheel_rpm = self.calculate_wheel_rpm(vehicle_speed_mps)
        return wheel_rpm * overall_gear_ratio

    def calculate_engine_rpm(self, vehicle_speed_mps: float, gear_number: int) -> float:
        """
        Calculates the engine's RPM based on vehicle speed and current gear.

        Args:
            vehicle_speed_mps: The linear speed of the vehicle in meters per second.
            gear_number: The current transmission gear (1-indexed).

        Returns:
            The engine RPM. Returns 0.0 if no engine is present or inputs are invalid.
        """
        if not self.engine:
            return 0.0
        return self._calculate_source_rpm(vehicle_speed_mps, gear_number)

    def calculate_motor_rpm(self, vehicle_speed_mps: float, gear_number: int) -> float:
        """
        Calculates the electric motor's RPM based on vehicle speed and current gear.
        Assumes motor is connected via the same transmission as the engine for hybrid setups,
        or is the primary mover using this transmission.

        Args:
            vehicle_speed_mps: The linear speed of the vehicle in meters per second.
            gear_number: The current transmission gear (1-indexed).

        Returns:
            The motor RPM. Returns 0.0 if no motor is present or inputs are invalid.
        """
        if not self.motor:
            return 0.0
        # Note: This assumes a parallel or series-parallel hybrid where motor RPM is related to wheel speed via transmission.
        # For some hybrid architectures (e.g. certain series hybrids or power-split devices),
        # motor RPM might be independent of vehicle speed or have a more complex relationship.
        return self._calculate_source_rpm(vehicle_speed_mps, gear_number)

    def get_combined_torque(self, engine_rpm: float, motor_rpm: float, requested_throttle_percentage: float = 1.0, mode: str = "hybrid") -> float:
        """
        Calculates the combined torque from the engine and/or motor based on the specified operational mode.

        The method considers the requested throttle percentage and applies it to the torque
        available from the active power sources (engine and/or motor). For the motor,
        torque is also limited by its own maximum power (`motor.max_power_kw`) and
        the battery's maximum discharge capability (`battery.max_discharge_power_kw`).

        Args:
            engine_rpm: Current RPM of the engine.
            motor_rpm: Current RPM of the electric motor.
            requested_throttle_percentage: Throttle input (0.0 to 1.0). Defaults to 1.0 (100%).
            mode: Operational mode. Can be one of:
                  - "hybrid": Combines torque from both engine and motor.
                  - "engine_only": Uses torque only from the engine.
                  - "motor_only": Uses torque only from the motor.
                  Defaults to "hybrid".

        Returns:
            The total torque (Nm) available at the input of the transmission,
            after considering the operational mode, throttle, and power limits.
        """
        total_torque = 0.0

        # Calculate available engine torque, scaled by throttle
        if self.engine and mode in ["hybrid", "engine_only"]:
            engine_base_torque = self.engine.get_torque(engine_rpm)
            total_torque += engine_base_torque * requested_throttle_percentage

        # Calculate available motor torque, scaled by throttle and limited by battery
        if self.motor and self.battery and mode in ["hybrid", "motor_only"]:
            # Get motor torque from its curve (already considers motor's own max_power_kw)
            motor_curve_torque = self.motor.get_torque(motor_rpm)
            throttled_motor_torque = motor_curve_torque * requested_throttle_percentage

            # Determine torque limit imposed by the battery's max discharge power
            torque_limit_from_battery = float('inf') # Assume no limit if no battery power or RPM is zero
            if motor_rpm > 0 and self.battery.max_discharge_power_kw > 0:
                # Torque_max_from_battery (Nm) = Battery_Power_max (kW) * 9549.3 / Motor_RPM
                 torque_limit_from_battery = (self.battery.max_discharge_power_kw * 9549.3) / motor_rpm
            elif self.battery.max_discharge_power_kw <= 0 : # If battery cannot discharge (or is depleted)
                 torque_limit_from_battery = 0.0 # No torque contribution from motor possible via battery

            # Actual motor torque is the minimum of throttled curve torque and battery-limited torque
            actual_motor_torque = min(throttled_motor_torque, torque_limit_from_battery)

            # Combine with total torque based on mode
            if mode == "motor_only":
                total_torque = actual_motor_torque # Motor is the sole source
            else: # "hybrid" mode
                total_torque += actual_motor_torque # Add motor torque to engine torque (if any)

        return total_torque

    def calculate_tractive_force(self, combined_input_torque_to_transmission: float, gear_number: int) -> float:
        """
        Calculates the total tractive force generated at the wheels.

        This force is the result of the combined engine/motor torque, multiplied by
        the transmission ratios and efficiency, and then divided by the tire radius.

        Args:
            combined_input_torque_to_transmission: The total torque (Nm) from power sources
                                                   input to the transmission.
            gear_number: The current transmission gear (1-indexed).

        Returns:
            The total tractive force (N) at the driven wheels. Returns 0.0 if tire radius
            is not positive or transmission is invalid.
        """
        if self.tire_radius_m <= 0 or not self.transmission:
            return 0.0

        # Get torque multiplier from transmission (includes gear ratio, final drive, and efficiency)
        torque_multiplier = self.transmission.get_torque_multiplier(gear_number)

        # Torque at wheels = Input torque * Overall transmission multiplier
        torque_at_wheels = combined_input_torque_to_transmission * torque_multiplier

        # Tractive force = Torque at wheels / Tire radius
        return torque_at_wheels / self.tire_radius_m

    def calculate_resistive_forces(self, velocity_mps: float) -> float:
        """
        Calculates the sum of resistive forces acting on the vehicle.

        These include aerodynamic drag and rolling resistance.

        Args:
            velocity_mps: The current speed of the vehicle in meters per second.

        Returns:
            The total resistive force (N) acting against the vehicle's motion.
        """
        # Aerodynamic Drag Force: F_drag = 0.5 * rho * A * Cd * v^2
        # rho: air density, A: frontal area, Cd: drag coefficient, v: velocity
        drag_force = 0.5 * self.air_density * self.frontal_area_m2 * self.drag_coefficient * (velocity_mps**2)

        # Rolling Resistance Force: F_roll = Crr * m * g
        # Crr: rolling resistance coefficient, m: mass, g: gravity
        rolling_resistance_force = self.rolling_resistance_coefficient * self.mass_kg * self.gravity

        return drag_force + rolling_resistance_force
