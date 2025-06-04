# hybrid_vehicle_simulation/src/outputs.py
import pandas as pd
import numpy as np
from typing import Dict, Any, Optional

# Assuming core.py and simulation.py are in the same directory or PYTHONPATH
try:
    from core import Vehicle #, Engine, Motor # Not directly needed for these functions beyond Vehicle
    from simulation import KMH_TO_MPS
except ImportError:
    print("Attempting to import core/simulation components for outputs.py (may fail in some contexts)")
    # Add path adjustments if necessary for direct execution, similar to simulation.py
    # This fallback might be needed if running a script that only uses outputs.py directly
    # For the main application (main.py, main_gui.py), imports should work via sys.path adjustments there.
    from .core import Vehicle
    from .simulation import KMH_TO_MPS

GRAVITY_ACCEL = 9.81 # m/s^2, gravitational acceleration

def display_simulation_results(
    acceleration_times: Optional[Dict[float, float]] = None,
    top_speed_results: Optional[Dict[str, float]] = None,
    fuel_economy_estimate: Optional[float] = None,
    max_gradeability_percent: Optional[float] = None # Added for future use
) -> None:
    """
    Prints the key performance indicators from the simulation. (Korean Output)
    """
    print("\n--- 차량 성능 요약 ---")

    if acceleration_times:
        print("\n가속 성능:")
        sorted_targets = sorted(acceleration_times.keys())
        for target_kmh in sorted_targets:
            time_val = acceleration_times[target_kmh]
            if time_val == float('inf'):
                print(f"  0-{int(target_kmh)} km/h: 목표 도달 실패")
            else:
                print(f"  0-{int(target_kmh)} km/h: {time_val:.2f} 초")
        if 80.0 in acceleration_times and 120.0 in acceleration_times:
            time_to_80 = acceleration_times[80.0]
            time_to_120 = acceleration_times[120.0]
            if time_to_80 != float('inf') and time_to_120 != float('inf'):
                time_80_to_120 = time_to_120 - time_to_80
                print(f"  80-120 km/h: {time_80_to_120:.2f} 초")
            else:
                print(f"  80-120 km/h: 해당 구간 계산 불가 (목표 도달 실패 등)")
    else:
        print("\n가속 성능: 데이터 없음")

    if top_speed_results:
        print("\n최고 속도:")
        for mode, speed_kmh in top_speed_results.items():
            if speed_kmh == float('inf') or speed_kmh <= 0:
                print(f"  {mode}: 계산 불가 또는 0 km/h")
            else:
                print(f"  {mode}: {speed_kmh:.2f} km/h")
    else:
        print("\n최고 속도: 데이터 없음")

    if max_gradeability_percent is not None: # Display max gradeability
        print("\n최대 등판 능력:")
        if max_gradeability_percent == float('inf'):
            print("  최대 등판 능력: 수직 등반 가능 (이론상)")
        elif max_gradeability_percent == float('-inf'):
            print("  최대 등판 능력: 계산 오류 (하강)")
        elif max_gradeability_percent <= -999.0 : # Error code from function
            print("  최대 등판 능력: 입력 오류 또는 계산 불가")
        else:
            print(f"  최대 등판 능력: {max_gradeability_percent:.2f} %")
    else:
        print("\n최대 등판 능력: 데이터 없음")

    if fuel_economy_estimate is not None:
        print("\n연비:")
        if fuel_economy_estimate < 0:
            print("  기본 연비 예측: 제공되지 않음 (기본 시뮬레이션).")
        else:
            print(f"  기본 연비 예측: {fuel_economy_estimate:.2f} (단위 보류)")
    else:
        print("\n연비: 데이터 없음")
    print("-----------------------------------\n")


def get_torque_curves_data(
    vehicle: Vehicle,
    rpm_points: Optional[np.ndarray] = None
) -> pd.DataFrame:
    engine_torques = []
    motor_torques = []
    combined_torques = []
    max_rpm_limit = 0
    if vehicle.engine:
        max_rpm_limit = max(max_rpm_limit, vehicle.engine.redline_rpm)
        if isinstance(vehicle.engine.torque_curve, pd.Series) and not vehicle.engine.torque_curve.empty:
            max_rpm_limit = max(max_rpm_limit, vehicle.engine.torque_curve.index.max())
    if vehicle.motor:
        max_rpm_limit = max(max_rpm_limit, vehicle.motor.redline_rpm)
        if isinstance(vehicle.motor.torque_curve, pd.Series) and not vehicle.motor.torque_curve.empty:
            max_rpm_limit = max(max_rpm_limit, vehicle.motor.torque_curve.index.max())
    if max_rpm_limit == 0: max_rpm_limit = 7000
    if rpm_points is None: rpm_points = np.linspace(0, max_rpm_limit, 200)

    for rpm in rpm_points:
        engine_tq = 0.0
        if vehicle.engine:
            engine_tq = vehicle.engine.get_torque(rpm) if rpm <= vehicle.engine.redline_rpm else 0.0
        motor_tq = 0.0
        if vehicle.motor and vehicle.battery:
            potential_motor_tq = vehicle.motor.get_torque(rpm) if rpm <= vehicle.motor.redline_rpm else 0.0
            if rpm > 0 and vehicle.battery.max_discharge_power_kw > 0:
                battery_limited_tq = (vehicle.battery.max_discharge_power_kw * 9549.3) / rpm
                motor_tq = min(potential_motor_tq, battery_limited_tq)
            elif rpm > 0 and vehicle.battery.max_discharge_power_kw <=0: # battery cannot discharge
                motor_tq = 0.0
            else: # rpm is 0
                motor_tq = potential_motor_tq
        engine_torques.append(engine_tq)
        motor_torques.append(motor_tq)
        combined_torques.append(engine_tq + motor_tq)
    return pd.DataFrame({'RPM': rpm_points, 'EngineTorque_Nm': engine_torques, 'MotorTorque_Nm': motor_torques, 'CombinedTorque_Nm': combined_torques})

def get_power_curves_data(
    vehicle: Vehicle,
    rpm_points: Optional[np.ndarray] = None
) -> pd.DataFrame:
    torque_data_df = get_torque_curves_data(vehicle, rpm_points)
    rpm_col = torque_data_df['RPM']
    engine_power_kw = (torque_data_df['EngineTorque_Nm'] * rpm_col).divide(9549.3).fillna(0)
    motor_power_kw = (torque_data_df['MotorTorque_Nm'] * rpm_col).divide(9549.3).fillna(0)
    if vehicle.motor:
         motor_power_kw = np.minimum(motor_power_kw, vehicle.motor.max_power_kw)
         if vehicle.battery: # Also cap by battery's max discharge power
             motor_power_kw = np.minimum(motor_power_kw, vehicle.battery.max_discharge_power_kw)
    combined_power_kw = engine_power_kw + motor_power_kw
    return pd.DataFrame({'RPM': rpm_col, 'EnginePower_kW': engine_power_kw, 'MotorPower_kW': motor_power_kw, 'CombinedPower_kW': combined_power_kw})

def get_gear_dependent_acceleration_map_data(
    vehicle: Vehicle,
    speed_points_kmh: Optional[np.ndarray] = None,
    min_engine_rpm_for_map: float = 1000.0
) -> pd.DataFrame:
    if speed_points_kmh is None:
        max_eval_speed_kmh = 160
        speed_points_kmh = np.linspace(0, max_eval_speed_kmh, 30) # Increased points

    speed_points_mps = speed_points_kmh * KMH_TO_MPS

    accel_data = {'Speed_kmh': speed_points_kmh}
    if not vehicle.transmission or not vehicle.transmission.gear_ratios: return pd.DataFrame(accel_data)
    num_gears = len(vehicle.transmission.gear_ratios)

    min_rpm_to_use_for_engine = min_engine_rpm_for_map
    if vehicle.engine and vehicle.engine.torque_curve is not None and not vehicle.engine.torque_curve.empty:
        min_rpm_to_use_for_engine = max(min_engine_rpm_for_map, vehicle.engine.torque_curve.index.min())

    for gear_num in range(1, num_gears + 1):
        gear_accel_values_g = []
        for speed_mps in speed_points_mps:
            acceleration_g = 0.0 # Default to 0

            engine_rpm = vehicle.calculate_engine_rpm(speed_mps, gear_num) if vehicle.engine else 0
            motor_rpm = vehicle.calculate_motor_rpm(speed_mps, gear_num) if vehicle.motor else 0

            # Condition for valid operation:
            # Engine (if present) must be above its min operational RPM and below redline.
            # Motor (if present) must be below its redline.
            # If only motor, engine conditions are ignored. If only engine, motor conditions ignored.

            valid_op = True
            if vehicle.engine:
                if speed_mps > 0.1 and engine_rpm < min_rpm_to_use_for_engine: valid_op = False
                if engine_rpm > vehicle.engine.redline_rpm: valid_op = False

            if vehicle.motor and motor_rpm > vehicle.motor.redline_rpm:
                # If only motor is driving (e.g. engine_only mode is NOT selected implicitly or explicitly)
                # and motor is over redline, then operation might be invalid or limited.
                # For simplicity in map, if motor is over redline, consider it non-contributing or invalid.
                if not vehicle.engine: # Pure EV case
                    valid_op = False
                # If hybrid, and motor over redline, engine might still contribute.
                # get_combined_torque will handle motor torque being zero if over redline.

            if valid_op:
                # Assume flat road (gradient 0) for this standard acceleration map
                combined_torque = vehicle.get_combined_torque(engine_rpm, motor_rpm, 1.0, mode="hybrid")
                tractive_force = vehicle.calculate_tractive_force(combined_torque, gear_num)
                resistive_forces = vehicle.calculate_resistive_forces(speed_mps, road_gradient_percent=0.0)
                net_force = tractive_force - resistive_forces

                acceleration_mps2 = net_force / vehicle.mass_kg if vehicle.mass_kg > 0 else 0.0
                acceleration_mps2 = max(0, acceleration_mps2)
                acceleration_g = acceleration_mps2 / GRAVITY_ACCEL

            gear_accel_values_g.append(acceleration_g)

        accel_data[f'Gear_{gear_num}_Accel_g'] = gear_accel_values_g

    return pd.DataFrame(accel_data)
