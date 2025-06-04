# hybrid_vehicle_simulation/src/plotting.py
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from typing import Optional

# --- Add Korean Font Support ---
import platform
from matplotlib import font_manager, rcParams

try:
    system_os = platform.system()
    if system_os == 'Windows':
        font_path = "c:/Windows/Fonts/malgun.ttf" # Malgun Gothic for Windows
        font_name = font_manager.FontProperties(fname=font_path).get_name()
        rcParams['font.family'] = font_name
    elif system_os == 'Darwin': # macOS
        try:
            rcParams['font.family'] = 'AppleGothic'
        except RuntimeError:
            try:
                font_names = [f.name for f in font_manager.fontManager.ttflist]
                if 'NanumGothic' in font_names:
                    rcParams['font.family'] = 'NanumGothic'
                else:
                    print("경고: AppleGothic 또는 NanumGothic 폰트를 찾을 수 없습니다. macOS에서 그래프의 한글이 깨질 수 있습니다.")
            except Exception as e_font_macos:
                print(f"macOS 폰트 검색 중 오류: {e_font_macos}")
    elif system_os == 'Linux':
        font_path = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
        import os
        if os.path.exists(font_path):
            font_name = font_manager.FontProperties(fname=font_path).get_name()
            rcParams['font.family'] = font_name
        else:
            try:
                font_names = [f.name for f in font_manager.fontManager.ttflist]
                if 'NanumGothic' in font_names:
                    rcParams['font.family'] = 'NanumGothic'
                else:
                    print("경고: NanumGothic 폰트를 찾을 수 없거나 지정된 경로에 없습니다. 'sudo apt-get install fonts-nanum*' 등으로 설치해주세요. Linux에서 그래프의 한글이 깨질 수 있습니다.")
            except Exception as e_font_linux:
                print(f"Linux 폰트 검색 중 오류: {e_font_linux}")
    rcParams['axes.unicode_minus'] = False
except Exception as e:
    print(f"Matplotlib 한글 폰트 설정 중 오류 발생: {e}. 그래프의 한글 표시가 원활하지 않을 수 있습니다.")
# --- End Korean Font Support ---

try:
    from outputs import get_torque_curves_data, get_power_curves_data
    from core import Vehicle
except ImportError:
    print("Attempting to import components for plotting.py (may fail in some contexts if not run via main GUI)")
    # Fallback for direct script execution or specific test environments
    # This might be needed if you try to run plotting.py directly for its __main__ example
    # and the path adjustments in main_gui.py or main.py are not active.
    try:
        from .outputs import get_torque_curves_data, get_power_curves_data
        from .core import Vehicle
    except ImportError:
        # If relative import fails (e.g. plotting.py is run as top-level script)
        # This part is tricky and depends on execution context.
        # For the project, it's expected to be called from main_gui.py where sys.path is set.
        pass


def plot_speed_vs_time(simulation_df: pd.DataFrame, title: str = "속도 대 시간"): # Default title in Korean
    if not all(col in simulation_df.columns for col in ['Time_s', 'Speed_kmh']):
        print("오류: simulation_df에 'Time_s' 및 'Speed_kmh' 컬럼이 필요합니다.")
        return
    plt.figure(figsize=(10, 6))
    plt.plot(simulation_df['Time_s'], simulation_df['Speed_kmh'], label="속도")
    plt.xlabel("시간 (s)")
    plt.ylabel("속도 (km/h)")
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.show()

def plot_torque_rpm_curves(
    vehicle_obj: Vehicle,
    rpm_points_for_curve: Optional[np.ndarray] = None,
    simulation_df: Optional[pd.DataFrame] = None,
    title: str = "토크 대 RPM" # Default title in Korean
):
    if not vehicle_obj: # Guard against None vehicle_obj
        print("오류: 차량 객체가 제공되지 않았습니다.")
        return
    torque_curves_df = get_torque_curves_data(vehicle_obj, rpm_points=rpm_points_for_curve)
    if torque_curves_df.empty:
        print("경고: 토크 커브 데이터를 생성할 수 없습니다 (차량 또는 토크 데이터가 비어있을 수 있음).")
        return

    plt.figure(figsize=(12, 7))
    if vehicle_obj.engine and 'EngineTorque_Nm' in torque_curves_df:
        plt.plot(torque_curves_df['RPM'], torque_curves_df['EngineTorque_Nm'], label="엔진 토크 (잠재)", linestyle='--')
    if vehicle_obj.motor and 'MotorTorque_Nm' in torque_curves_df:
        plt.plot(torque_curves_df['RPM'], torque_curves_df['MotorTorque_Nm'], label="모터 토크 (잠재)", linestyle='--')
    if 'CombinedTorque_Nm' in torque_curves_df:
        plt.plot(torque_curves_df['RPM'], torque_curves_df['CombinedTorque_Nm'], label="결합 토크 (잠재)", color='black', linewidth=2)

    if simulation_df is not None and not simulation_df.empty and vehicle_obj.engine and \
       'Gear' in simulation_df.columns and 'Engine_RPM' in simulation_df.columns:
        shift_points_rpm = []
        shift_points_torque = []
        gears = simulation_df['Gear'].astype(str).str.extract(r'(\d+)').astype(float).to_numpy() # Handle '1(S)'
        rpms = simulation_df['Engine_RPM'].to_numpy()
        for i in range(1, len(simulation_df)):
            if gears[i] != gears[i-1] and gears[i-1] !=0 and not np.isnan(gears[i]) and not np.isnan(gears[i-1]):
                shift_rpm = rpms[i-1]
                shift_points_rpm.append(shift_rpm)
                if not torque_curves_df.empty and 'RPM' in torque_curves_df and 'CombinedTorque_Nm' in torque_curves_df:
                    shift_torque = np.interp(shift_rpm, torque_curves_df['RPM'], torque_curves_df['CombinedTorque_Nm'])
                    shift_points_torque.append(shift_torque)
        if shift_points_rpm and shift_points_torque: # Ensure both have points
            plt.scatter(shift_points_rpm, shift_points_torque, color='red', s=100, zorder=5, label="기어 변속 (근사)")

    plt.xlabel("RPM")
    plt.ylabel("토크 (Nm)")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.ylim(bottom=0)
    max_rpm_plot = 0
    if vehicle_obj.engine: max_rpm_plot = max(max_rpm_plot, vehicle_obj.engine.redline_rpm if vehicle_obj.engine.redline_rpm else 0)
    if vehicle_obj.motor: max_rpm_plot = max(max_rpm_plot, vehicle_obj.motor.redline_rpm if vehicle_obj.motor.redline_rpm else 0)
    if not torque_curves_df.empty and 'RPM' in torque_curves_df and torque_curves_df['RPM'].max() > max_rpm_plot : max_rpm_plot = torque_curves_df['RPM'].max()
    plt.xlim(left=0, right=max_rpm_plot * 1.05 if max_rpm_plot > 0 else 7000) # Default xlim if no redlines
    plt.show()

def plot_power_rpm_curves(
    vehicle_obj: Vehicle,
    rpm_points_for_curve: Optional[np.ndarray] = None,
    simulation_df: Optional[pd.DataFrame] = None,
    title: str = "파워 대 RPM" # Default title in Korean
):
    if not vehicle_obj:
        print("오류: 차량 객체가 제공되지 않았습니다.")
        return
    power_curves_df = get_power_curves_data(vehicle_obj, rpm_points=rpm_points_for_curve)
    if power_curves_df.empty:
        print("경고: 파워 커브 데이터를 생성할 수 없습니다.")
        return

    plt.figure(figsize=(12, 7))
    if vehicle_obj.engine and 'EnginePower_kW' in power_curves_df:
        plt.plot(power_curves_df['RPM'], power_curves_df['EnginePower_kW'], label="엔진 파워 (잠재)", linestyle='--')
    if vehicle_obj.motor and 'MotorPower_kW' in power_curves_df:
        plt.plot(power_curves_df['RPM'], power_curves_df['MotorPower_kW'], label="모터 파워 (잠재)", linestyle='--')
    if 'CombinedPower_kW' in power_curves_df:
        plt.plot(power_curves_df['RPM'], power_curves_df['CombinedPower_kW'], label="결합 파워 (잠재)", color='black', linewidth=2)

    if simulation_df is not None and not simulation_df.empty and vehicle_obj.engine and \
       'Gear' in simulation_df.columns and 'Engine_RPM' in simulation_df.columns:
        shift_points_rpm = []
        shift_points_power = []
        gears = simulation_df['Gear'].astype(str).str.extract(r'(\d+)').astype(float).to_numpy()
        rpms = simulation_df['Engine_RPM'].to_numpy()
        for i in range(1, len(simulation_df)):
            if gears[i] != gears[i-1] and gears[i-1] !=0 and not np.isnan(gears[i]) and not np.isnan(gears[i-1]):
                shift_rpm = rpms[i-1]
                shift_points_rpm.append(shift_rpm)
                if not power_curves_df.empty and 'RPM' in power_curves_df and 'CombinedPower_kW' in power_curves_df:
                    shift_power = np.interp(shift_rpm, power_curves_df['RPM'], power_curves_df['CombinedPower_kW'])
                    shift_points_power.append(shift_power)
        if shift_points_rpm and shift_points_power:
            plt.scatter(shift_points_rpm, shift_points_power, color='red', s=100, zorder=5, label="기어 변속 (근사)")

    plt.xlabel("RPM")
    plt.ylabel("파워 (kW)")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.ylim(bottom=0)
    max_rpm_plot = 0
    if vehicle_obj.engine: max_rpm_plot = max(max_rpm_plot, vehicle_obj.engine.redline_rpm if vehicle_obj.engine.redline_rpm else 0)
    if vehicle_obj.motor: max_rpm_plot = max(max_rpm_plot, vehicle_obj.motor.redline_rpm if vehicle_obj.motor.redline_rpm else 0)
    if not power_curves_df.empty and 'RPM' in power_curves_df and power_curves_df['RPM'].max() > max_rpm_plot : max_rpm_plot = power_curves_df['RPM'].max()
    plt.xlim(left=0, right=max_rpm_plot*1.05 if max_rpm_plot > 0 else 7000)
    plt.show()

def plot_acceleration_map(
    accel_map_df: pd.DataFrame,
    title: str = "기어별 최대 가속도"
):
    if 'Speed_kmh' not in accel_map_df.columns:
        print("오류: accel_map_df에 'Speed_kmh' 컬럼이 없습니다.")
        return
    plt.figure(figsize=(12, 7))
    for col in accel_map_df.columns:
        if col.startswith('Gear_') and col.endswith('_Accel_g'): # Updated to check for _Accel_g
            gear_label = col.replace('_Accel_g', '').replace('_', ' ') + "단"
            plt.plot(accel_map_df['Speed_kmh'], accel_map_df[col], label=gear_label)
    plt.xlabel("속도 (km/h)")
    plt.ylabel("최대 가속도 (g)") # Updated Y-axis label
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.ylim(bottom=0)
    plt.show()

if __name__ == '__main__':
    # This is a placeholder for the original __main__ block from plotting.py
    print("plotting.py executed as main for testing. (Test code should be here).")
    # Example: Create dummy data and call one of the plot functions
    # This requires Vehicle, get_torque_curves_data etc. to be available or mocked.
    # Due to complexity of setting up a full Vehicle for this standalone test here,
    # it's better to test these via main_gui.py or dedicated unit tests.
    pass
