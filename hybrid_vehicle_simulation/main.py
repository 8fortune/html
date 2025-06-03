# hybrid_vehicle_simulation/main.py
import os
import sys
import pandas as pd # For type checking, good to have at the top

# Adjust path to import from src directory
# This assumes main.py is in hybrid_vehicle_simulation/ and src/ is a subdirectory.
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'src'))

try:
    from core import Engine, Motor, Transmission, Battery, Vehicle
    from simulation import simulate_acceleration, estimate_top_speed
    from outputs import display_simulation_results, get_torque_curves_data, get_power_curves_data, get_gear_dependent_acceleration_map_data
    from plotting import plot_speed_vs_time, plot_torque_rpm_curves, plot_power_rpm_curves, plot_acceleration_map
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Please ensure that src directory is in PYTHONPATH or script is run from project root.")
    print(f"Current sys.path: {sys.path}")
    sys.exit(1)

def main():
    print("하이브리드 차량 동적 성능 시뮬레이션 시작")

    # 1. 차량 구성 요소 정의
    # 엔진 토크 커브는 CSV에서 로드되므로 여기서는 None 또는 빈 pd.Series로 초기화
    engine = Engine(torque_curve={}, redline_rpm=6500)

    # 모터 토크 커브도 CSV에서 로드
    motor = Motor(torque_curve={}, redline_rpm=8000, max_power_kw=75) # 예시 최대 모터 파워 75kW

    battery = Battery(capacity_kwh=10.0, max_discharge_power_kw=80) # 예시 배터리 용량 및 최대 방전 파워

    transmission = Transmission(
        gear_ratios=[3.6, 2.1, 1.4, 1.0, 0.75], # 5단 변속기 예시
        final_gear_ratio=3.5,
        efficiency=0.92
    )

    # 2. 차량 객체 생성
    vehicle = Vehicle(
        mass_kg=1500.0,
        frontal_area_m2=2.2,
        drag_coefficient=0.28,
        rolling_resistance_coefficient=0.01,
        tire_radius_m=0.32,
        transmission=transmission,
        engine=engine,
        motor=motor,
        battery=battery,
        drivetrain_type="FWD"
    )

    # 3. 토크 커브 CSV 파일로부터 로드
    # 파일 경로는 main.py 위치 기준으로 상대 경로 설정
    data_dir = os.path.join(current_dir, 'data')
    engine_torque_csv = os.path.join(data_dir, 'sample_engine_torque.csv')
    motor_torque_csv = os.path.join(data_dir, 'sample_motor_torque.csv')

    vehicle.load_torque_curves_from_csv(
        engine_csv_path=engine_torque_csv,
        motor_csv_path=motor_torque_csv
    )

    # 로드 확인 (선택 사항)
    if isinstance(vehicle.engine.torque_curve, pd.Series) and not vehicle.engine.torque_curve.empty:
        print("엔진 토크 커브 로드 완료.")
    else:
        print("경고: 엔진 토크 커브 로드 실패 또는 데이터 없음.")

    if isinstance(vehicle.motor.torque_curve, pd.Series) and not vehicle.motor.torque_curve.empty:
        print("모터 토크 커브 로드 완료.")
    else:
        print("경고: 모터 토크 커브 로드 실패 또는 데이터 없음.")


    # 4. 시뮬레이션 실행
    print("\n시뮬레이션 실행 중...")

    # 가속 시간 측정 (0-60, 0-80, 0-100, 0-120 km/h)
    accel_targets_kmh = [60.0, 80.0, 100.0, 120.0]
    acceleration_times, accel_sim_df = simulate_acceleration(
        vehicle,
        target_speeds_kmh=accel_targets_kmh,
        simulation_mode="hybrid" # 하이브리드 모드로 가속 테스트
    )

    # 최고 속도 측정
    top_speed_hybrid, top_speed_hybrid_sim_df = estimate_top_speed(vehicle, mode="hybrid")
    top_speed_engine_only, top_speed_engine_sim_df = estimate_top_speed(vehicle, mode="engine_only")
    # 모터 단독 최고 속도 (선택 사항)
    # top_speed_motor_only, _ = estimate_top_speed(vehicle, mode="motor_only")

    top_speed_results = {
        "하이브리드 모드": top_speed_hybrid,
        "엔진 단독 모드": top_speed_engine_only,
        # "모터 단독 모드": top_speed_motor_only
    }

    # (자리 표시자) 연비
    fuel_economy = -1.0

    # 5. 결과 출력
    display_simulation_results(
        acceleration_times=acceleration_times,
        top_speed_results=top_speed_results,
        fuel_economy_estimate=fuel_economy
    )

    # 6. 시각화 데이터 생성 및 플롯 표시
    # Matplotlib 백엔드 설정 (스크립트 실행 환경에 따라 필요할 수 있음)
    # import matplotlib
    # matplotlib.use('Agg') # 예: 'Agg'는 non-interactive 백엔드, 파일 저장용
    # print(f"Matplotlib current backend: {matplotlib.get_backend()}")


    print("\n플롯 생성 중...")

    # 속도 vs 시간 (0-120km/h 시뮬레이션 결과 사용)
    plot_speed_vs_time(accel_sim_df, title="속도 vs 시간 (하이브리드 가속)")

    # 토크 vs RPM (기어 변속점 포함)
    plot_torque_rpm_curves(vehicle, simulation_df=accel_sim_df, title="토크 vs RPM (하이브리드)")

    # 파워 vs RPM (기어 변속점 포함)
    plot_power_rpm_curves(vehicle, simulation_df=accel_sim_df, title="파워 vs RPM (하이브리드)")

    # 기어별 가속도 맵
    accel_map_df = get_gear_dependent_acceleration_map_data(vehicle)
    plot_acceleration_map(accel_map_df, title="기어별 최대 가속도")

    print("\n시뮬레이션 및 플롯 생성 완료. 플롯 창을 닫으면 프로그램이 종료됩니다.")

if __name__ == '__main__':
    main()
