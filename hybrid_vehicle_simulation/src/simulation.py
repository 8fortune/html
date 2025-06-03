# hybrid_vehicle_simulation/src/simulation.py
import math
import pandas as pd
from typing import Tuple, List, Dict, Any
# Ensure core classes can be imported. Assuming src is in PYTHONPATH or using relative imports if run as module.
# For simplicity in subtask, direct import might work if file structure is flat for execution.
# from .core import Vehicle, Engine, Motor, Transmission, Battery
# To make it runnable directly for now, let's assume core can be imported if in same root or PYTHONPATH
# For a real package structure, relative imports would be like: from .core import Vehicle

# If running this file directly and core.py is in the same directory:
try:
    from core import Vehicle, Engine, Motor, Transmission, Battery
except ImportError:
    # This is for cases where the script might be run directly and src is not in python path
    # In a proper package, you'd use relative imports: from .core import ...
    print("Attempting to import core components for simulation.py")
    # This path adjustment is a hack for direct script execution in some environments
    # and might not be needed or work in all setups.
    # import sys
    # import os
    # sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    # from core import Vehicle, Engine, Motor, Transmission, Battery


# Constants
DEFAULT_TIME_STEP = 0.01  # s, Default time step for simulation iterations.
KMH_TO_MPS = 1 / 3.6      # Conversion factor from kilometers per hour to meters per second.
MPS_TO_KMH = 3.6          # Conversion factor from meters per second to kilometers per hour.
# Defines the RPM threshold for upshifting, as a percentage of the engine's redline RPM.
# E.g., 0.85 means shift when engine RPM reaches 85% of its redline.
DEFAULT_SHIFT_RPM_PERCENTAGE = 0.85


def simulate_acceleration(
    vehicle: Vehicle,
    target_speeds_kmh: List[float] = [100.0],
    time_step: float = DEFAULT_TIME_STEP,
    initial_gear: int = 1,
    simulation_mode: str = "hybrid"
) -> Tuple[Dict[float, float], pd.DataFrame]:
    """
    Simulates vehicle acceleration from a standstill to one or more target speeds.

    The simulation iteratively calculates the vehicle's state (speed, acceleration, RPMs, forces)
    at each time step. It employs a basic gear shifting logic based on a percentage of the
    engine's redline RPM. The simulation stops when the highest target speed is reached,
    all specified targets are met, or a maximum simulation time is exceeded.

    Args:
        vehicle: The `Vehicle` object configured with all necessary components and parameters.
        target_speeds_kmh: A list of target speeds in km/h for which acceleration times
                           are to be recorded (e.g., [60.0, 100.0, 120.0]).
                           Defaults to [100.0].
        time_step: The duration of each simulation step in seconds. A smaller time step
                   increases accuracy but also computation time. Defaults to `DEFAULT_TIME_STEP`.
        initial_gear: The gear in which the vehicle starts accelerating. Defaults to 1.
        simulation_mode: The powertrain operational mode ("hybrid", "engine_only", or "motor_only").
                         This determines which power sources are used. Defaults to "hybrid".

    Returns:
        A tuple containing:
            - times_to_targets (Dict[float, float]): A dictionary where keys are the target speeds (km/h)
              and values are the corresponding times (in seconds) taken to reach those speeds from
              a standstill. If a target speed is not reached, its time is `float('inf')`.
            - results_df (pd.DataFrame): A pandas DataFrame logging the detailed state of the vehicle
              at each time step. Columns include: 'Time_s', 'Speed_mps', 'Speed_kmh',
              'Acceleration_mps2', 'Gear', 'Engine_RPM', 'Motor_RPM', 'Tractive_Force_N',
              'Resistive_Force_N', 'Net_Force_N', and 'Distance_m'.
    """
    if not target_speeds_kmh:
        return {}, pd.DataFrame() # Return empty if no targets are specified

    # Prepare target speeds: sort and convert to m/s
    sorted_targets_kmh = sorted(list(set(target_speeds_kmh))) # Ensure unique and sorted targets
    max_target_speed_mps = max(sorted_targets_kmh) * KMH_TO_MPS

    # Initialize dictionaries to store results
    times_to_targets: Dict[float, float] = {target: float('inf') for target in sorted_targets_kmh}
    targets_mps = {target_kmh: target_kmh * KMH_TO_MPS for target_kmh in sorted_targets_kmh}

    # Initialize simulation variables
    current_time = 0.0
    current_speed_mps = 0.0 # Start from standstill
    current_gear = initial_gear
    distance_m = 0.0 # Total distance covered

    # Data logging setup
    log_data = {
        'Time_s': [], 'Speed_mps': [], 'Speed_kmh': [], 'Acceleration_mps2': [], 'Gear': [],
        'Engine_RPM': [], 'Motor_RPM': [], 'Tractive_Force_N': [],
        'Resistive_Force_N': [], 'Net_Force_N': [], 'Distance_m': []
    }
    max_simulation_time = 300 # seconds, safety break for the simulation loop

    # Flags to track progress towards targets
    hit_targets_kmh = {target: False for target in sorted_targets_kmh}
    all_targets_hit = False

    # Main simulation loop
    # Continues until all targets are hit, or max speed/time is exceeded.
    # A small speed buffer (5 KMH_TO_MPS) is added to ensure targets are properly crossed.
    while (not all_targets_hit) and \
          current_speed_mps < (max_target_speed_mps + 5 * KMH_TO_MPS) and \
          current_time < max_simulation_time:

        # Calculate RPM for engine and motor based on current speed and gear
        engine_rpm = vehicle.calculate_engine_rpm(current_speed_mps, current_gear) if vehicle.engine else 0
        motor_rpm = vehicle.calculate_motor_rpm(current_speed_mps, current_gear) if vehicle.motor else 0

        # --- RPM Adjustments (Stall/Redline) ---
        if vehicle.engine:
            min_engine_rpm_for_torque = vehicle.engine.torque_curve.index.min()
            # Prevent engine stall: if RPM is too low at low speed, set to min torque RPM
            if current_speed_mps > 0.1 and engine_rpm < min_engine_rpm_for_torque * 0.8:
                 engine_rpm = min_engine_rpm_for_torque
            # Cap RPM at redline
            if engine_rpm > vehicle.engine.redline_rpm :
                engine_rpm = vehicle.engine.redline_rpm
        if vehicle.motor and motor_rpm > vehicle.motor.redline_rpm:
            motor_rpm = vehicle.motor.redline_rpm # Cap motor RPM at its redline

        # --- Force Calculation ---
        # Get combined torque from power sources, considering mode and throttle (100% for max acceleration)
        combined_torque_at_transmission_input = vehicle.get_combined_torque(
            engine_rpm, motor_rpm, 1.0, mode=simulation_mode
        )
        # Calculate tractive force at the wheels
        tractive_force_n = vehicle.calculate_tractive_force(combined_torque_at_transmission_input, current_gear)
        # Calculate resistive forces (drag + rolling resistance)
        resistive_force_n = vehicle.calculate_resistive_forces(current_speed_mps)
        # Calculate net force acting on the vehicle
        net_force_n = tractive_force_n - resistive_force_n

        # Calculate acceleration using Newton's second law (F=ma)
        acceleration_mps2 = (net_force_n / vehicle.mass_kg) if vehicle.mass_kg > 0 else 0.0
        # Prevent negative acceleration from standstill (e.g., if resistive forces are high initially)
        if current_speed_mps == 0 and acceleration_mps2 < 0:
            acceleration_mps2 = 0

        # --- Update Vehicle State (Euler integration) ---
        previous_speed_mps = current_speed_mps # Store speed before update for interpolation
        current_speed_mps += acceleration_mps2 * time_step
        if current_speed_mps < 0: # Vehicle should not move backward in this simulation
            current_speed_mps = 0

        distance_m += current_speed_mps * time_step # Update distance covered

        # --- Data Logging ---
        log_data['Time_s'].append(current_time)
        log_data['Speed_mps'].append(current_speed_mps)
        current_speed_kmh = current_speed_mps * MPS_TO_KMH
        log_data['Speed_kmh'].append(current_speed_kmh)
        log_data['Acceleration_mps2'].append(acceleration_mps2)
        log_data['Gear'].append(current_gear)
        log_data['Engine_RPM'].append(engine_rpm if vehicle.engine else 0)
        log_data['Motor_RPM'].append(motor_rpm if vehicle.motor else 0)
        log_data['Tractive_Force_N'].append(tractive_force_n)
        log_data['Resistive_Force_N'].append(resistive_force_n)
        log_data['Net_Force_N'].append(net_force_n)
        log_data['Distance_m'].append(distance_m)

        # --- Target Achievement Check ---
        for target_kmh_val in sorted_targets_kmh:
            if not hit_targets_kmh[target_kmh_val] and current_speed_kmh >= target_kmh_val:
                # Interpolate to find more precise time when target speed was crossed
                if acceleration_mps2 > 0: # Avoid division by zero if acceleration is zero
                    time_to_reach_target_exactly = current_time - time_step + \
                        (targets_mps[target_kmh_val] - previous_speed_mps) / acceleration_mps2
                elif previous_speed_mps > 0 :
                     time_to_reach_target_exactly = current_time - time_step if previous_speed_mps >= targets_mps[target_kmh_val] else current_time
                else:
                    time_to_reach_target_exactly = current_time

                times_to_targets[target_kmh_val] = time_to_reach_target_exactly
                hit_targets_kmh[target_kmh_val] = True

        all_targets_hit = all(hit_targets_kmh.values()) # Update overall target status

        # --- Gear Shifting Logic ---
        # Basic upshift logic: shift up if engine RPM reaches a certain percentage of redline
        if vehicle.engine and engine_rpm >= vehicle.engine.redline_rpm * DEFAULT_SHIFT_RPM_PERCENTAGE:
            if current_gear < len(vehicle.transmission.gear_ratios): # Check if a higher gear exists
                current_gear += 1
                # TODO: Consider adding shift delay or more sophisticated shift point logic

        # Increment time for the next step
        current_time += time_step

        # Optimization: Exit loop early if all targets are met and vehicle is past the max target speed
        if current_speed_mps >= max_target_speed_mps and all_targets_hit:
            if len(sorted_targets_kmh) == 1 and sorted_targets_kmh[0] * KMH_TO_MPS <= current_speed_mps:
                 pass # Allow one more step if it's the only target and just crossed, for data logging
            elif all_targets_hit :
                break

    # Convert logged data to a pandas DataFrame
    results_df = pd.DataFrame(log_data)
    return times_to_targets, results_df


def estimate_top_speed(
    vehicle: Vehicle,
    time_step: float = DEFAULT_TIME_STEP,
    initial_gear: int = 1,
    max_simulation_duration_s: float = 300.0, # Maximum time to run the simulation
    min_acceleration_threshold: float = 0.005, # m/s^2, threshold to consider vehicle at top speed
    mode: str = "hybrid" # Powertrain mode: "hybrid", "engine_only", "motor_only"
) -> Tuple[float, pd.DataFrame]:
    """
    Estimates the vehicle's top speed for a given powertrain mode.

    The simulation runs until the vehicle's acceleration drops below a specified
    minimum threshold (indicating that resistive forces nearly equal tractive forces)
    or until a maximum simulation duration is reached. The gear shifting logic
    attempts to find the optimal gear for maximum speed.

    Args:
        vehicle: The `Vehicle` object.
        time_step: Simulation time step in seconds. Defaults to `DEFAULT_TIME_STEP`.
        initial_gear: The gear to start the simulation in. Defaults to 1.
        max_simulation_duration_s: Maximum duration for the simulation run in seconds.
                                   Defaults to 300s.
        min_acceleration_threshold: The minimum positive acceleration (m/s^2) below which
                                    the vehicle is considered to have reached its top speed.
                                    Defaults to 0.005 m/s^2.
        mode: The powertrain operational mode ("hybrid", "engine_only", "motor_only").
              Defaults to "hybrid".

    Returns:
        A tuple containing:
            - top_speed_kmh (float): The estimated top speed in km/h.
            - results_df (pd.DataFrame): A pandas DataFrame with the simulation data log.
    """
    current_time = 0.0
    current_speed_mps = 0.0
    current_gear = initial_gear # Start in initial_gear, will try to shift up

    # Data logging setup
    log_data = {
        'Time_s': [], 'Speed_mps': [], 'Speed_kmh': [], 'Acceleration_mps2': [], 'Gear': [],
        'Engine_RPM': [], 'Motor_RPM': [], 'Tractive_Force_N': [],
        'Resistive_Force_N': [], 'Net_Force_N': []
    }
    acceleration_mps2 = 1.0 # Initialize with a value greater than the threshold to start the loop

    # Main simulation loop for top speed
    while acceleration_mps2 > min_acceleration_threshold and current_time < max_simulation_duration_s:
        # Calculate RPMs for power sources
        engine_rpm = vehicle.calculate_engine_rpm(current_speed_mps, current_gear) if vehicle.engine else 0
        motor_rpm = vehicle.calculate_motor_rpm(current_speed_mps, current_gear) if vehicle.motor else 0

        # --- RPM Adjustments (Redline) ---
        if vehicle.engine and engine_rpm > vehicle.engine.redline_rpm :
            engine_rpm = vehicle.engine.redline_rpm
        if vehicle.motor and motor_rpm > vehicle.motor.redline_rpm:
             motor_rpm = vehicle.motor.redline_rpm

        # --- Force Calculation ---
        # Get combined torque based on mode and 100% throttle
        combined_torque = vehicle.get_combined_torque(engine_rpm, motor_rpm, 1.0, mode=mode)
        tractive_force = vehicle.calculate_tractive_force(combined_torque, current_gear)
        resistive_force = vehicle.calculate_resistive_forces(current_speed_mps)
        net_force = tractive_force - resistive_force

        # Calculate acceleration
        acceleration_mps2 = (net_force / vehicle.mass_kg) if vehicle.mass_kg > 0 else 0.0

        # Check termination condition: if acceleration is below threshold, top speed is likely reached
        if acceleration_mps2 < min_acceleration_threshold :
             if not log_data['Speed_kmh'] or current_speed_mps == 0 :
                 pass # Will result in 0 top speed if loop breaks immediately
             break

        # --- Update Vehicle State ---
        previous_speed_mps = current_speed_mps
        current_speed_mps += acceleration_mps2 * time_step
        if current_speed_mps < 0 : current_speed_mps = 0 # Safety check

        # --- Data Logging ---
        log_data['Time_s'].append(current_time)
        log_data['Speed_mps'].append(current_speed_mps)
        log_data['Speed_kmh'].append(current_speed_mps * MPS_TO_KMH)
        log_data['Acceleration_mps2'].append(acceleration_mps2)
        log_data['Gear'].append(current_gear)
        log_data['Engine_RPM'].append(engine_rpm if vehicle.engine else 0)
        log_data['Motor_RPM'].append(motor_rpm if vehicle.motor else 0)
        log_data['Tractive_Force_N'].append(tractive_force)
        log_data['Resistive_Force_N'].append(resistive_force)
        log_data['Net_Force_N'].append(net_force)

        # --- Gear Shifting Logic for Top Speed ---
        # Aims to find the gear that allows the highest speed.
        # This simplified logic shifts up when engine (or motor in motor_only mode) hits RPM threshold.
        # A more advanced strategy might test different gears at high speeds.
        perform_shift = False
        if mode == "motor_only" and vehicle.motor:
            if len(vehicle.transmission.gear_ratios) > 1 and \
               motor_rpm >= vehicle.motor.redline_rpm * DEFAULT_SHIFT_RPM_PERCENTAGE:
                perform_shift = True
        elif vehicle.engine and mode != "motor_only": # Primarily engine-driven or hybrid
             if engine_rpm >= vehicle.engine.redline_rpm * DEFAULT_SHIFT_RPM_PERCENTAGE:
                perform_shift = True

        if perform_shift and current_gear < len(vehicle.transmission.gear_ratios):
            current_gear += 1
            # Note: After shifting, RPMs will drop. The simulation continues to find equilibrium.

        current_time += time_step

    results_df = pd.DataFrame(log_data)
    top_speed_kmh = results_df['Speed_kmh'].max() if not results_df.empty else 0.0
    return top_speed_kmh, results_df

# Placeholder for fuel economy - very complex, requires drive cycle and fuel map
def estimate_fuel_economy(vehicle: Vehicle, cycle_data_path: str) -> float:
    """
    Basic placeholder for fuel economy estimation.
    This is a very complex calculation requiring a drive cycle (speed vs. time)
    and engine/motor efficiency maps or fuel consumption maps.

    Args:
        vehicle: The Vehicle object.
        cycle_data_path: Path to a CSV file defining a drive cycle (e.g., columns: Time_s, Speed_kmh).

    Returns:
        Estimated fuel economy (e.g., in L/100km or MPG) - currently a placeholder.
    """
    # print("Fuel economy estimation is a complex feature and requires detailed maps and cycle data.")
    # 1. Load drive cycle (time vs speed)
    # 2. For each point in cycle:
    #    a. Calculate required tractive force to meet speed and acceleration.
    #    b. Determine engine/motor RPM and torque split (hybrid strategy needed).
    #    c. Use engine fuel consumption map (e.g., g/kWh vs RPM & torque) to find fuel rate.
    #    d. Use motor efficiency map to find electrical power needed (-> battery discharge).
    #    e. Integrate fuel consumption over cycle.
    #    f. Integrate distance covered.
    #    g. Calculate L/100km or MPG.
    # This is a significant sub-project in itself.
    return -1.0 # Placeholder
