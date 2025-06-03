# hybrid_vehicle_simulation/src/outputs.py
import pandas as pd
import numpy as np
from typing import Dict, Any, Optional

# Assuming core.py and simulation.py are in the same directory or PYTHONPATH
try:
    from core import Vehicle, Engine, Motor
    from simulation import KMH_TO_MPS # For potential use, though not directly used in this snippet
except ImportError:
    print("Attempting to import core/simulation components for outputs.py")
    # Add path adjustments if necessary for direct execution, similar to simulation.py
    # import sys
    # import os
    # sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    # from core import Vehicle, Engine, Motor
    # from simulation import KMH_TO_MPS

def display_simulation_results(
    acceleration_times: Optional[Dict[float, float]] = None, # e.g., {100.0: 7.5, 120.0: 10.2}
    top_speed_results: Optional[Dict[str, float]] = None, # e.g., {"Hybrid": 220.5, "Engine Only": 190.0}
    fuel_economy_estimate: Optional[float] = None
) -> None:
    """
    Prints the key performance indicators from the simulation.

    Args:
        acceleration_times: A dictionary mapping target speed (km/h) to time (s).
                            Includes various segments like 0-100, 80-120 etc.
                            The specific segments depend on what was requested from simulate_acceleration.
                            For an X-Y km/h time, the value for Y would be used, assuming X is 0
                            or previous segment time is subtracted by the caller.
                            This function will just display raw target times from 0.
                            Caller should compute X-Y times if needed.
        top_speed_results: A dictionary mapping mode (str) to estimated top speed (km/h).
        fuel_economy_estimate: Estimated fuel economy (e.g., in L/100km).
    """
    print("\n--- 차량 성능 요약 ---")

    if acceleration_times:
        print("\n가속 성능:")
        # Sort targets for consistent display order
        sorted_targets = sorted(acceleration_times.keys())

        # Display 0-X times
        for target_kmh in sorted_targets:
            time_val = acceleration_times[target_kmh]
            if time_val == float('inf'):
                print(f"  0-{int(target_kmh)} km/h: 목표 도달 실패")
            else:
                print(f"  0-{int(target_kmh)} km/h: {time_val:.2f} 초")

        # Calculate and display specific segments like 80-120 km/h if data is available
        # This requires times for both 80 and 120 to be present.
        if 80.0 in acceleration_times and 120.0 in acceleration_times:
            time_to_80 = acceleration_times[80.0]
            time_to_120 = acceleration_times[120.0]
            if time_to_80 != float('inf') and time_to_120 != float('inf'):
                time_80_to_120 = time_to_120 - time_to_80
                print(f"  80-120 km/h: {time_80_to_120:.2f} 초")
            elif time_to_120 != float('inf'): # Reached 120 but not 80 (should not happen if sorted) or 80 failed
                 print(f"  80-120 km/h: 80km/h 도달 실패 또는 120km/h 데이터만 존재")
            else:
                print(f"  80-120 km/h: 목표 구간 도달 실패")
    else:
        print("\n가속 성능: 데이터 없음")

    if top_speed_results:
        print("\n최고 속도:")
        for mode, speed_kmh in top_speed_results.items():
            if speed_kmh == float('inf') or speed_kmh <= 0: # Should not be inf, but good check
                print(f"  {mode}: 계산 불가 또는 0 km/h")
            else:
                print(f"  {mode}: {speed_kmh:.2f} km/h")
    else:
        print("\n최고 속도: 데이터 없음")

    if fuel_economy_estimate is not None:
        print("\n연비:")
        if fuel_economy_estimate < 0: # Placeholder value
            print("  기본 연비 예측: 제공되지 않음 (기본 시뮬레이션).")
        else:
            # TODO: Add units based on how fuel economy is calculated
            print(f"  기본 연비 예측: {fuel_economy_estimate:.2f} (단위 보류)")
    else:
        print("\n연비: 데이터 없음")

    print("-----------------------------------\n")

def get_torque_curves_data(
    vehicle: Vehicle,
    rpm_points: Optional[np.ndarray] = None
) -> pd.DataFrame:
    """
    Generates data for plotting torque curves (engine, motor, combined).

    Args:
        vehicle: The Vehicle object.
        rpm_points: Optional numpy array of RPM points to calculate torque for.
                    If None, uses a default range or available curve points.

    Returns:
        A pandas DataFrame with columns: 'RPM', 'EngineTorque_Nm', 'MotorTorque_Nm', 'CombinedTorque_Nm'.
    """
    engine_torques = []
    motor_torques = []
    combined_torques = []

    # Determine RPM range for calculation
    # Consider engine and motor redlines and curve data points
    # For simplicity, let's try to use a common range up to max redline found
    max_rpm_limit = 0
    if vehicle.engine:
        max_rpm_limit = max(max_rpm_limit, vehicle.engine.redline_rpm)
        if isinstance(vehicle.engine.torque_curve, pd.Series) and not vehicle.engine.torque_curve.empty:
            max_rpm_limit = max(max_rpm_limit, vehicle.engine.torque_curve.index.max())
    if vehicle.motor:
        max_rpm_limit = max(max_rpm_limit, vehicle.motor.redline_rpm)
        if isinstance(vehicle.motor.torque_curve, pd.Series) and not vehicle.motor.torque_curve.empty:
            max_rpm_limit = max(max_rpm_limit, vehicle.motor.torque_curve.index.max())

    if max_rpm_limit == 0: # No engine or motor, or no data
        max_rpm_limit = 7000 # Default fallback

    if rpm_points is None:
        rpm_points = np.linspace(0, max_rpm_limit, 200) # Generate 200 points up to max RPM

    for rpm in rpm_points:
        engine_tq = 0.0
        if vehicle.engine:
            # Assume engine RPM and motor RPM are the same for this combined curve plotting purpose
            # This is a simplification; in reality, they might differ based on hybrid architecture
            # or if one is off. For a "potential" combined torque curve, this is a common approach.
            engine_tq = vehicle.engine.get_torque(rpm) if rpm <= vehicle.engine.redline_rpm else 0.0

        motor_tq = 0.0
        if vehicle.motor and vehicle.battery:
            # Motor get_torque already considers motor's own power limit.
            # Here, we also need to consider battery's power limit for the combined curve context.
            potential_motor_tq = vehicle.motor.get_torque(rpm) if rpm <= vehicle.motor.redline_rpm else 0.0

            if rpm > 0:
                power_for_potential_tq_kw = (potential_motor_tq * rpm) / 9549.3
                battery_max_power_kw = vehicle.battery.max_discharge_power_kw

                if power_for_potential_tq_kw > battery_max_power_kw:
                    motor_tq = (battery_max_power_kw * 9549.3) / rpm
                else:
                    motor_tq = potential_motor_tq
            else: # RPM is 0
                motor_tq = potential_motor_tq # Torque at 0 RPM from curve

        engine_torques.append(engine_tq)
        motor_torques.append(motor_tq)
        combined_torques.append(engine_tq + motor_tq) # Simple addition for potential combined torque

    return pd.DataFrame({
        'RPM': rpm_points,
        'EngineTorque_Nm': engine_torques,
        'MotorTorque_Nm': motor_torques,
        'CombinedTorque_Nm': combined_torques
    })

def get_power_curves_data(
    vehicle: Vehicle,
    rpm_points: Optional[np.ndarray] = None # Same RPM points as torque curve
) -> pd.DataFrame:
    """
    Generates data for plotting power curves (engine, motor, combined).
    Power (kW) = Torque (Nm) * RPM / 9549.3

    Args:
        vehicle: The Vehicle object.
        rpm_points: Optional numpy array of RPM points. If None, uses torque_curves_df RPMs.

    Returns:
        A pandas DataFrame with columns: 'RPM', 'EnginePower_kW', 'MotorPower_kW', 'CombinedPower_kW'.
    """
    torque_data_df = get_torque_curves_data(vehicle, rpm_points) # Reuse RPM points logic

    # Calculate power from torque
    # Avoid division by zero for RPM; power is 0 at 0 RPM if torque is finite.
    rpm_col = torque_data_df['RPM']

    engine_power_kw = (torque_data_df['EngineTorque_Nm'] * rpm_col).divide(9549.3).fillna(0)
    motor_power_kw = (torque_data_df['MotorTorque_Nm'] * rpm_col).divide(9549.3).fillna(0)

    # Combined power is sum of individual powers, not from combined torque directly multiplied by RPM
    # because engine and motor might hit their respective power limits at different RPMs or conditions.
    # However, the get_torque_curves_data already provides torque limited by battery for motor.
    # So, combined_power = (combined_torque * rpm) / 9549.3 should be consistent if combined_torque is true available.
    # Let's sum individual powers for clarity and to respect individual limits better.

    # Ensure motor power does not exceed its own max_power_kw (already handled in motor.get_torque -> get_torque_curves_data)
    # and also respects battery.max_discharge_power_kw (handled in get_torque_curves_data for motor)
    if vehicle.motor:
         motor_power_kw = np.minimum(motor_power_kw, vehicle.motor.max_power_kw)
         if vehicle.battery:
             motor_power_kw = np.minimum(motor_power_kw, vehicle.battery.max_discharge_power_kw)


    # Engine power calculation is direct.
    # For combined power, it's the sum of the actual available engine power and actual available motor power.
    combined_power_kw = engine_power_kw + motor_power_kw

    return pd.DataFrame({
        'RPM': rpm_col,
        'EnginePower_kW': engine_power_kw,
        'MotorPower_kW': motor_power_kw,
        'CombinedPower_kW': combined_power_kw
    })


def get_gear_dependent_acceleration_map_data(
    vehicle: Vehicle,
    speed_points_kmh: Optional[np.ndarray] = None # e.g. np.linspace(0, 150, 15)
) -> pd.DataFrame:
    """
    Generates data for max acceleration in each gear vs. speed.
    This is a simplified representation.

    Args:
        vehicle: The Vehicle object.
        speed_points_kmh: Optional array of speed points (km/h) to evaluate acceleration.

    Returns:
        A pandas DataFrame with columns: 'Speed_kmh', 'Gear_1_Accel', 'Gear_2_Accel', ...
    """
    if speed_points_kmh is None:
        # Determine a reasonable max speed for this map, e.g., based on estimated top speed or a fixed value
        max_eval_speed_kmh = 160
        speed_points_kmh = np.linspace(0, max_eval_speed_kmh, 20)

    speed_points_mps = speed_points_kmh * KMH_TO_MPS

    accel_data = {'Speed_kmh': speed_points_kmh}
    num_gears = len(vehicle.transmission.gear_ratios)

    for gear_num in range(1, num_gears + 1):
        gear_accel_values = []
        for speed_mps in speed_points_mps:
            if speed_mps < 0.1 and gear_num > 1 : # Avoid calculating high gear accel at near zero speed
                gear_accel_values.append(0) # Or NaN
                continue

            engine_rpm = vehicle.calculate_engine_rpm(speed_mps, gear_num) if vehicle.engine else 0
            motor_rpm = vehicle.calculate_motor_rpm(speed_mps, gear_num) if vehicle.motor else 0

            # Check if RPMs are within operational limits for the sources
            valid_rpm = True
            if vehicle.engine and (engine_rpm > vehicle.engine.redline_rpm or engine_rpm < vehicle.engine.torque_curve.index.min()):
                # For this map, if RPM is out of comfortable range, assume low/no accel contribution from engine in this state
                # We are interested in *max possible* acceleration. If engine is off, it is off.
                 pass # get_combined_torque will use 0 torque if RPM is too high/low based on get_torque logic

            if vehicle.motor and (motor_rpm > vehicle.motor.redline_rpm):
                 pass # Similar for motor

            combined_torque = vehicle.get_combined_torque(engine_rpm, motor_rpm, 1.0) # Max throttle
            tractive_force = vehicle.calculate_tractive_force(combined_torque, gear_num)
            resistive_forces = vehicle.calculate_resistive_forces(speed_mps)
            net_force = tractive_force - resistive_forces

            acceleration = net_force / vehicle.mass_kg if vehicle.mass_kg > 0 else 0
            acceleration = max(0, acceleration) # Show only positive acceleration capability
            gear_accel_values.append(acceleration)

        accel_data[f'Gear_{gear_num}_Accel_mps2'] = gear_accel_values

    return pd.DataFrame(accel_data)
