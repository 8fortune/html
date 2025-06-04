# hybrid_vehicle_simulation/src/simulation.py
import math
import pandas as pd
from typing import Tuple, List, Dict, Any, Optional

try:
    from core import Vehicle # Normal execution
except ImportError:
    # Fallback for direct script execution or specific test environments
    import sys, os
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
    from src.core import Vehicle


KMH_TO_MPS = 1 / 3.6
MPS_TO_KMH = 3.6
DEFAULT_TIME_STEP = 0.01
DEFAULT_SHIFT_RPM_PERCENTAGE_OF_REDLINE = 0.85 # Default: shift at 85% of engine redline

def simulate_acceleration(
    vehicle: Vehicle,
    target_speeds_kmh: List[float] = [100.0],
    time_step: float = DEFAULT_TIME_STEP,
    initial_gear: int = 1,
    simulation_mode: str = "hybrid",
    road_gradient_percent: float = 0.0,
    shift_time_s: float = 0.0,
    common_shift_rpm: Optional[int] = None
) -> Tuple[Dict[float, float], pd.DataFrame]:
    """
    Simulates vehicle acceleration from standstill to target speeds, considering road gradient and shift times.
    The simulation continues until max_simulation_time or vehicle stops accelerating significantly,
    ensuring a full acceleration curve is generated even if all specified targets are met early.
    """
    if not target_speeds_kmh: target_speeds_kmh = [100.0] # Default target if list is empty

    target_speeds_kmh = [float(t) for t in target_speeds_kmh]
    sorted_targets_kmh = sorted(list(set(target_speeds_kmh)))
    # max_target_speed_for_hitting_logic_mps is the highest speed for which time is explicitly recorded.
    max_target_speed_for_hitting_logic_mps = max(sorted_targets_kmh) * KMH_TO_MPS if sorted_targets_kmh else 0.0

    times_to_targets: Dict[float, float] = {target: float('inf') for target in sorted_targets_kmh}
    targets_mps = {target_kmh: target_kmh * KMH_TO_MPS for target_kmh in sorted_targets_kmh}

    current_time = 0.0
    current_speed_mps = 0.0
    current_gear = initial_gear
    distance_m = 0.0
    log_data = {
        'Time_s': [], 'Speed_mps': [], 'Speed_kmh': [], 'Acceleration_mps2': [], 'Gear': [],
        'Engine_RPM': [], 'Motor_RPM': [], 'Tractive_Force_N': [],
        'Resistive_Force_N': [], 'Net_Force_N': [], 'Distance_m': []
    }
    max_simulation_time = 300
    hit_targets_kmh = {target: False for target in sorted_targets_kmh}
    all_specified_targets_hit_flag = False # Tracks if all *specified* targets are hit
    is_shifting = False
    shift_timer = 0.0

    effective_shift_rpm_target = float('inf')
    if vehicle.engine:
        effective_shift_rpm_target = common_shift_rpm if (common_shift_rpm and common_shift_rpm > 0) else \
                                     (vehicle.engine.redline_rpm * DEFAULT_SHIFT_RPM_PERCENTAGE_OF_REDLINE)

    min_accel_for_continuation = 0.01 # m/s^2, threshold to consider if vehicle is still accelerating

    # Main simulation loop: continue as long as time allows and vehicle is still accelerating meaningfully
    while current_time < max_simulation_time:
        engine_rpm = 0.0; motor_rpm = 0.0; acceleration_mps2 = 0.0
        tractive_force_n = 0.0; net_force_n = 0.0

        resistive_force_n = vehicle.calculate_resistive_forces(current_speed_mps, road_gradient_percent)

        if is_shifting:
            shift_timer -= time_step
            if shift_timer <= 0: is_shifting = False
            tractive_force_n = 0.0
            net_force_n = tractive_force_n - resistive_force_n
            acceleration_mps2 = net_force_n / vehicle.mass_kg if vehicle.mass_kg > 0 else 0.0
            engine_rpm = vehicle.calculate_engine_rpm(current_speed_mps, current_gear) if vehicle.engine else 0.0
            motor_rpm = vehicle.calculate_motor_rpm(current_speed_mps, current_gear) if vehicle.motor else 0.0
        else:
            engine_rpm = vehicle.calculate_engine_rpm(current_speed_mps, current_gear) if vehicle.engine else 0.0
            motor_rpm = vehicle.calculate_motor_rpm(current_speed_mps, current_gear) if vehicle.motor else 0.0

            if vehicle.engine and (vehicle.engine.torque_curve is not None and not vehicle.engine.torque_curve.empty):
                min_engine_rpm = vehicle.engine.torque_curve.index.min()
                if current_speed_mps > 0.01 and engine_rpm < min_engine_rpm : engine_rpm = min_engine_rpm
                if engine_rpm > vehicle.engine.redline_rpm : engine_rpm = vehicle.engine.redline_rpm
            if vehicle.motor and motor_rpm > vehicle.motor.redline_rpm: motor_rpm = vehicle.motor.redline_rpm

            combined_torque_at_transmission_input = vehicle.get_combined_torque(engine_rpm, motor_rpm, 1.0, mode=simulation_mode)
            tractive_force_n = vehicle.calculate_tractive_force(combined_torque_at_transmission_input, current_gear)
            net_force_n = tractive_force_n - resistive_force_n
            acceleration_mps2 = (net_force_n / vehicle.mass_kg) if vehicle.mass_kg > 0 else 0.0
            if current_speed_mps < 0.01 and acceleration_mps2 < 0: acceleration_mps2 = 0.0

        previous_speed_mps = current_speed_mps
        current_speed_mps += acceleration_mps2 * time_step
        if current_speed_mps < 0: current_speed_mps = 0.0; acceleration_mps2 = 0.0;

        distance_m += current_speed_mps * time_step

        log_data['Time_s'].append(current_time)
        log_data['Speed_mps'].append(current_speed_mps)
        current_speed_kmh = current_speed_mps * MPS_TO_KMH
        log_data['Speed_kmh'].append(current_speed_kmh)
        log_data['Acceleration_mps2'].append(acceleration_mps2)
        log_data['Gear'].append(f"{current_gear}{'(S)' if is_shifting else ''}")
        log_data['Engine_RPM'].append(engine_rpm)
        log_data['Motor_RPM'].append(motor_rpm)
        log_data['Tractive_Force_N'].append(tractive_force_n)
        log_data['Resistive_Force_N'].append(resistive_force_n)
        log_data['Net_Force_N'].append(net_force_n)
        log_data['Distance_m'].append(distance_m)

        if not is_shifting:
            temp_all_targets_hit_this_step = True # Assume all hit, prove otherwise
            for target_kmh_val_float in sorted_targets_kmh:
                if not hit_targets_kmh[target_kmh_val_float] and current_speed_kmh >= target_kmh_val_float:
                    time_to_reach_target_exactly = current_time
                    if abs(current_speed_mps - previous_speed_mps) > 1e-6 and previous_speed_mps < targets_mps[target_kmh_val_float]:
                         time_to_reach_target_exactly = current_time - time_step + \
                             time_step * (targets_mps[target_kmh_val_float] - previous_speed_mps) / (current_speed_mps - previous_speed_mps)
                    times_to_targets[target_kmh_val_float] = time_to_reach_target_exactly
                    hit_targets_kmh[target_kmh_val_float] = True
                if not hit_targets_kmh[target_kmh_val_float]: # Check again after potential update
                    temp_all_targets_hit_this_step = False
            all_specified_targets_hit_flag = temp_all_targets_hit_this_step

            current_engine_rpm_for_shift_check = vehicle.calculate_engine_rpm(current_speed_mps, current_gear) if vehicle.engine else float('inf')
            if simulation_mode != "motor_only" and vehicle.engine and current_engine_rpm_for_shift_check >= effective_shift_rpm_target:
                if current_gear < len(vehicle.transmission.gear_ratios):
                    current_gear += 1
                    if shift_time_s > 1e-6:
                        is_shifting = True
                        shift_timer = shift_time_s

        current_time += time_step

        # Termination condition:
        # If all explicitly requested targets have been met, AND
        # the vehicle is no longer significantly accelerating (below min_accel_for_continuation), AND
        # its current speed is at least the highest requested target speed.
        if all_specified_targets_hit_flag and \
           acceleration_mps2 < min_accel_for_continuation and \
           (not sorted_targets_kmh or current_speed_mps >= max_target_speed_for_hitting_logic_mps):
            break

    results_df = pd.DataFrame(log_data)
    return times_to_targets, results_df


def estimate_top_speed(
    vehicle: Vehicle,
    time_step: float = DEFAULT_TIME_STEP,
    initial_gear: int = 1,
    max_simulation_duration_s: float = 300.0,
    min_acceleration_threshold: float = 0.001,
    mode: str = "hybrid",
    road_gradient_percent: float = 0.0,
    common_shift_rpm: Optional[int] = None
) -> Tuple[float, pd.DataFrame]:
    current_time = 0.0; current_speed_mps = 0.0
    current_gear = initial_gear
    if vehicle.transmission.gear_ratios and len(vehicle.transmission.gear_ratios) > 3 and initial_gear < 3:
        current_gear = min(3, len(vehicle.transmission.gear_ratios))

    log_data = { 'Time_s': [], 'Speed_mps': [], 'Speed_kmh': [], 'Acceleration_mps2': [], 'Gear': [],
                 'Engine_RPM': [], 'Motor_RPM': [], 'Tractive_Force_N': [], 'Resistive_Force_N': [], 'Net_Force_N': [] }
    acceleration_mps2 = 1.0

    effective_shift_rpm_target = float('inf')
    if vehicle.engine:
        effective_shift_rpm_target = common_shift_rpm if (common_shift_rpm and common_shift_rpm > 0) else \
                                     (vehicle.engine.redline_rpm * DEFAULT_SHIFT_RPM_PERCENTAGE_OF_REDLINE)

    max_gear = len(vehicle.transmission.gear_ratios) if vehicle.transmission.gear_ratios else 1

    while current_time < max_simulation_duration_s:
        engine_rpm = vehicle.calculate_engine_rpm(current_speed_mps, current_gear) if vehicle.engine else 0.0
        motor_rpm = vehicle.calculate_motor_rpm(current_speed_mps, current_gear) if vehicle.motor else 0.0
        if vehicle.engine and engine_rpm > vehicle.engine.redline_rpm : engine_rpm = vehicle.engine.redline_rpm
        if vehicle.motor and motor_rpm > vehicle.motor.redline_rpm: motor_rpm = vehicle.motor.redline_rpm

        combined_torque = vehicle.get_combined_torque(engine_rpm, motor_rpm, 1.0, mode=mode)
        tractive_force = vehicle.calculate_tractive_force(combined_torque, current_gear)
        resistive_force = vehicle.calculate_resistive_forces(current_speed_mps, road_gradient_percent)
        net_force = tractive_force - resistive_force

        acceleration_mps2 = (net_force / vehicle.mass_kg) if vehicle.mass_kg > 0 else 0.0

        if acceleration_mps2 < min_acceleration_threshold :
            if current_gear == max_gear:
                break
            else:
                current_gear += 1
                acceleration_mps2 = 1.0

        current_speed_mps += acceleration_mps2 * time_step
        if current_speed_mps < 0: current_speed_mps = 0.0

        log_data['Time_s'].append(current_time); log_data['Speed_mps'].append(current_speed_mps)
        log_data['Speed_kmh'].append(current_speed_mps * MPS_TO_KMH); log_data['Acceleration_mps2'].append(acceleration_mps2)
        log_data['Gear'].append(current_gear); log_data['Engine_RPM'].append(engine_rpm)
        log_data['Motor_RPM'].append(motor_rpm); log_data['Tractive_Force_N'].append(tractive_force)
        log_data['Resistive_Force_N'].append(resistive_force); log_data['Net_Force_N'].append(net_force)

        can_shift_up = False
        if current_gear < max_gear:
            rpm_for_shift_check = 0.0
            shift_target_rpm_value = float('inf')
            if mode == "motor_only" and vehicle.motor and vehicle.transmission.gear_ratios: # Motor only with gears
                rpm_for_shift_check = motor_rpm
                motor_specific_shift_target = vehicle.motor.redline_rpm * DEFAULT_SHIFT_RPM_PERCENTAGE_OF_REDLINE
                shift_target_rpm_value = common_shift_rpm if (common_shift_rpm and common_shift_rpm > 0) else motor_specific_shift_target
            elif vehicle.engine and mode != "motor_only": # Engine involved
                rpm_for_shift_check = engine_rpm
                shift_target_rpm_value = effective_shift_rpm_target
            if rpm_for_shift_check >= shift_target_rpm_value: can_shift_up = True

        if can_shift_up: current_gear += 1
        current_time += time_step

    results_df = pd.DataFrame(log_data)
    top_speed_kmh = results_df['Speed_kmh'].max() if not results_df.empty else 0.0
    return top_speed_kmh, results_df

def calculate_max_gradeability(vehicle: Vehicle, reference_rpm_for_torque: float = 500.0, start_gear: int = 1) -> float:
    if not vehicle.transmission.gear_ratios or not (1 <= start_gear <= len(vehicle.transmission.gear_ratios)): return -999.0

    engine_rpm_static = 0.0
    if vehicle.engine:
        engine_rpm_static = reference_rpm_for_torque
        if vehicle.engine.torque_curve is not None and not vehicle.engine.torque_curve.empty:
            min_eng_rpm = vehicle.engine.torque_curve.index.min()
            if engine_rpm_static < min_eng_rpm : engine_rpm_static = min_eng_rpm
        if vehicle.engine.redline_rpm is not None and engine_rpm_static > vehicle.engine.redline_rpm :
            engine_rpm_static = vehicle.engine.redline_rpm

    motor_rpm_static = 0.0
    if vehicle.motor:
        motor_rpm_static = reference_rpm_for_torque
        if vehicle.motor.torque_curve is not None and not vehicle.motor.torque_curve.empty:
            min_motor_rpm = vehicle.motor.torque_curve.index.min()
            if motor_rpm_static < min_motor_rpm and motor_rpm_static > 0 : motor_rpm_static = min_motor_rpm
            elif motor_rpm_static <= 0 and 0 not in vehicle.motor.torque_curve.index :
                motor_rpm_static = min_motor_rpm
        if vehicle.motor.redline_rpm is not None and motor_rpm_static > vehicle.motor.redline_rpm :
            motor_rpm_static = vehicle.motor.redline_rpm

    max_combined_torque_at_input = vehicle.get_combined_torque(
        engine_rpm_static, motor_rpm_static, requested_throttle_percentage=1.0, mode="hybrid"
    )
    max_tractive_force_at_wheels = vehicle.calculate_tractive_force(max_combined_torque_at_input, start_gear)
    rolling_resistance_N = vehicle.rolling_resistance_coefficient * vehicle.mass_kg * vehicle.gravity
    net_force_for_climbing = max_tractive_force_at_wheels - rolling_resistance_N
    if net_force_for_climbing <= 0: return 0.0

    sin_theta_val = net_force_for_climbing / (vehicle.mass_kg * vehicle.gravity)
    sin_theta_val = max(-1.0, min(1.0, sin_theta_val))

    if abs(abs(sin_theta_val) - 1.0) < 1e-9:
        gradient_percent = float('inf') if sin_theta_val > 0 else float('-inf')
    elif abs(1.0 - sin_theta_val**2) < 1e-12:
         gradient_percent = float('inf') if sin_theta_val > 0 else (float('-inf') if sin_theta_val < 0 else 0.0)
    else:
         gradient_percent = (sin_theta_val / math.sqrt(1.0 - sin_theta_val**2)) * 100.0

    return gradient_percent

def estimate_fuel_economy(vehicle: Vehicle, cycle_data_path: str) -> float:
    """Placeholder for fuel economy estimation."""
    return -1.0 # Not implemented
