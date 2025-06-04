# hybrid_vehicle_simulation/main_gui.py
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import json
import os
import sys
import pandas as pd
import math # For isnan, isinf

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'src'))
try:
    from core import Engine, Motor, Transmission, Battery, Vehicle
    from simulation import simulate_acceleration, estimate_top_speed, calculate_max_gradeability # Added calculate_max_gradeability
    from plotting import plot_speed_vs_time, plot_torque_rpm_curves, plot_power_rpm_curves, plot_acceleration_map
    from outputs import get_gear_dependent_acceleration_map_data
except ImportError as e:
    print(f"모듈 임포트 중 오류 발생: {e}.")
    sys.exit(1)

class VehicleSimApp:
    def __init__(self, root):
        self.root = root
        root.title("하이브리드 차량 성능 시뮬레이터 GUI")
        root.geometry("1000x850")

        self.spec_entries = {}
        self.simulation_results_df = None
        self.vehicle_obj_for_plotting = None
        self.accel_results = None
        self.top_speed_results = None # For flat ground
        self.max_gradeability_result = None
        self.uphill_top_speed_results = None # For specified gradient

        self.spec_validation_rules = {
            "mass_kg": True, "frontal_area_m2": True, "tire_radius_m": True,
            "engine_redline_rpm": True, "final_gear_ratio": True,
            "common_shift_rpm": True, "max_total_torque_nm": True,
            "drag_coefficient": False, "rolling_resistance_coefficient": False,
            "shift_time_s": False, "additional_mass_kg": False,
            "motor_redline_rpm": False,
            "battery_capacity_kwh": False,
            "motor_max_power_kw": False,
            "battery_max_discharge_power_kw": False,
            "road_gradient_percent": None,
            "drivetrain_efficiency_percent": (0, 100),
        }

        main_container = ttk.Frame(root, padding="10 10 10 10"); main_container.pack(expand=True, fill=tk.BOTH)
        top_pane = ttk.Frame(main_container); top_pane.pack(side=tk.TOP, fill=tk.X, expand=False, pady=5)
        self.input_frame = ttk.LabelFrame(top_pane, text="차량 제원 입력", padding="10 10"); self.input_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        input_params = [
            ("차량 질량 (kg):", "mass_kg", "1500.0"), ("전면 면적 (m²):", "frontal_area_m2", "2.2"),
            ("항력 계수 (Cd):", "drag_coefficient", "0.28"), ("구름 저항 계수 (μ):", "rolling_resistance_coefficient", "0.01"),
            ("타이어 반경 (m):", "tire_radius_m", "0.32"), ("엔진 레드라인 RPM:", "engine_redline_rpm", "6500"),
            ("모터 레드라인 RPM:", "motor_redline_rpm", "8000"), ("배터리 용량 (kWh):", "battery_capacity_kwh", "10.0"),
            ("최대 모터 파워 (kW):", "motor_max_power_kw", "75.0"), ("최대 배터리 방전 파워 (kW):", "battery_max_discharge_power_kw", "80.0"),
            ("기어비 (쉼표 구분):", "gear_ratios", "3.6,2.1,1.4,1.0,0.75"), ("최종 감속비 (FGR):", "final_gear_ratio", "3.5"),
            ("공통 변속 RPM (엔진):", "common_shift_rpm", "5500"), ("변속 시간 (초):", "shift_time_s", "0.2"),
            ("도로 경사도 (%):", "road_gradient_percent", "0.0"), ("등판/견인 추가 중량 (kg):", "additional_mass_kg", "0.0"),
            ("구동계 전체 효율 (%):", "drivetrain_efficiency_percent", "85.0"), ("최대 허용 총 토크 (Nm):", "max_total_torque_nm", "500.0"),
            ("엔진 토크 커브 CSV:", "engine_torque_csv", os.path.join(current_dir, 'data', 'sample_engine_torque.csv')),
            ("모터 토크 커브 CSV:", "motor_torque_csv", os.path.join(current_dir, 'data', 'sample_motor_torque.csv')),
        ]
        for i, (label_text, key, default_val) in enumerate(input_params):
            label = ttk.Label(self.input_frame, text=label_text); label.grid(row=i, column=0, sticky=tk.W, padx=5, pady=2)
            entry = ttk.Entry(self.input_frame, width=40); entry.grid(row=i, column=1, sticky=tk.EW, padx=5, pady=2); entry.insert(0, str(default_val))
            self.spec_entries[key] = entry
            if "_csv" in key: browse_button = ttk.Button(self.input_frame, text="찾아보기", command=lambda k=key: self.browse_file(k)); browse_button.grid(row=i, column=2, sticky=tk.W, padx=5, pady=2)
        self.input_frame.columnconfigure(1, weight=1)

        self.control_frame = ttk.LabelFrame(top_pane, text="제어", padding="10 10"); self.control_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        ttk.Button(self.control_frame, text="제원 저장", command=self.save_specifications).pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(self.control_frame, text="제원 불러오기", command=self.load_specifications).pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(self.control_frame, text="시뮬레이션 실행", command=self.run_simulation_sequence).pack(fill=tk.X, padx=5, pady=5, side=tk.BOTTOM)

        self.results_frame = ttk.LabelFrame(main_container, text="시뮬레이션 결과", padding="10 10"); self.results_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.results_text = tk.Text(self.results_frame, wrap=tk.WORD, height=20); self.results_text.pack(expand=True, fill=tk.BOTH, padx=5, pady=5)
        self.results_text.insert(tk.END, "시뮬레이션 결과가 여기에 표시됩니다."); self.results_text.config(state=tk.DISABLED)

        self.graph_buttons_frame = ttk.LabelFrame(main_container, text="그래프 생성", padding="10 10"); self.graph_buttons_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5, expand=False)
        ttk.Button(self.graph_buttons_frame, text="속도-시간", command=self.show_speed_time_plot).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(self.graph_buttons_frame, text="토크-RPM", command=self.show_torque_rpm_plot).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(self.graph_buttons_frame, text="파워-RPM", command=self.show_power_rpm_plot).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(self.graph_buttons_frame, text="가속도 맵", command=self.show_accel_map_plot).pack(side=tk.LEFT, padx=5, pady=5)

    def browse_file(self, entry_key: str):
        file_path = filedialog.askopenfilename(title=f"{entry_key} 파일 선택", filetypes=(("CSV files", "*.csv"), ("All files", "*.*")))
        if file_path:
            self.spec_entries[entry_key].delete(0, tk.END)
            self.spec_entries[entry_key].insert(0, file_path)

    def save_specifications(self):
        specs_data = {key: entry.get() for key, entry in self.spec_entries.items()}
        file_path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json"), ("All files", "*.*")], title="차량 제원 저장")
        if not file_path: return
        try:
            with open(file_path, 'w', encoding='utf-8') as f: json.dump(specs_data, f, ensure_ascii=False, indent=4)
            messagebox.showinfo("성공", f"제원이 '{file_path}'에 저장되었습니다.")
        except Exception as e: messagebox.showerror("오류", f"제원 저장 중 오류: {e}")

    def load_specifications(self):
        file_path = filedialog.askopenfilename(filetypes=[("JSON files", "*.json"), ("All files", "*.*")], title="차량 제원 불러오기")
        if not file_path: return
        try:
            with open(file_path, 'r', encoding='utf-8') as f: specs_data = json.load(f)
            for key, value in specs_data.items():
                if key in self.spec_entries:
                    self.spec_entries[key].delete(0, tk.END)
                    self.spec_entries[key].insert(0, value)
            messagebox.showinfo("성공", f"'{file_path}'에서 제원을 불러왔습니다.")
        except Exception as e: messagebox.showerror("오류", f"제원 불러오기 중 오류: {e}")

    def _get_and_validate_specs(self) -> dict:
        raw_specs = {key: entry.get() for key, entry in self.spec_entries.items()}
        processed_specs = {}
        try:
            processed_specs['mass_kg'] = float(raw_specs['mass_kg']); processed_specs['frontal_area_m2'] = float(raw_specs['frontal_area_m2'])
            processed_specs['drag_coefficient'] = float(raw_specs['drag_coefficient']); processed_specs['rolling_resistance_coefficient'] = float(raw_specs['rolling_resistance_coefficient'])
            processed_specs['tire_radius_m'] = float(raw_specs['tire_radius_m']); processed_specs['engine_redline_rpm'] = int(raw_specs['engine_redline_rpm'])
            processed_specs['motor_redline_rpm'] = int(raw_specs['motor_redline_rpm']); processed_specs['battery_capacity_kwh'] = float(raw_specs['battery_capacity_kwh'])
            processed_specs['motor_max_power_kw'] = float(raw_specs['motor_max_power_kw']); processed_specs['battery_max_discharge_power_kw'] = float(raw_specs['battery_max_discharge_power_kw'])
            processed_specs['final_gear_ratio'] = float(raw_specs['final_gear_ratio'])
            processed_specs['common_shift_rpm'] = int(raw_specs['common_shift_rpm']); processed_specs['shift_time_s'] = float(raw_specs['shift_time_s'])
            processed_specs['road_gradient_percent'] = float(raw_specs['road_gradient_percent']); processed_specs['additional_mass_kg'] = float(raw_specs['additional_mass_kg'])
            processed_specs['drivetrain_efficiency_percent'] = float(raw_specs['drivetrain_efficiency_percent']); processed_specs['max_total_torque_nm'] = float(raw_specs['max_total_torque_nm'])

            gear_ratios_str = raw_specs['gear_ratios'].split(','); processed_specs['gear_ratios'] = []
            for gr_str in gear_ratios_str: gr_str = gr_str.strip();
                if not gr_str: continue; gr_val = float(gr_str)
                if gr_val <= 0: raise ValueError("개별 기어비는 0보다 커야 합니다.");
                processed_specs['gear_ratios'].append(gr_val)
            if not processed_specs['gear_ratios']: raise ValueError("기어비 목록이 비어있습니다.")

            processed_specs['engine_torque_csv'] = raw_specs['engine_torque_csv'].strip(); processed_specs['motor_torque_csv'] = raw_specs['motor_torque_csv'].strip()

            for key, rule in self.spec_validation_rules.items():
                if key in processed_specs:
                    value = processed_specs[key]
                    if rule is True and value <= 0: raise ValueError(f"'{key}' 값은 0보다 커야 합니다.")
                    elif rule is False and value < 0: raise ValueError(f"'{key}' 값은 0 이상이어야 합니다.")
                    elif isinstance(rule, tuple) and not (rule[0] <= value <= rule[1]): raise ValueError(f"'{key}' 값은 {rule[0]}와 {rule[1]} 사이여야 합니다.")

            if processed_specs.get('motor_max_power_kw', 0) > 0:
                if processed_specs.get('motor_redline_rpm', 0) <= 0: raise ValueError("모터 사용 시 '모터 레드라인 RPM'은 0보다 커야 합니다.")
                if processed_specs.get('battery_capacity_kwh', 0) <= 0: raise ValueError("모터 사용 시 '배터리 용량 (kWh)'은 0보다 커야 합니다.")
                if processed_specs.get('battery_max_discharge_power_kw', 0) < 0: raise ValueError("'최대 배터리 방전 파워 (kW)'는 0 이상이어야 합니다.")
            return processed_specs
        except ValueError as e: messagebox.showerror("입력 오류", f"제원 값 오류: {e}"); raise
        except Exception as e: messagebox.showerror("입력 오류", f"제원 처리 중 알 수 없는 오류: {e}"); raise

    def run_simulation_sequence(self):
        self.results_text.config(state=tk.NORMAL)
        self.results_text.delete('1.0', tk.END)
        self.results_text.insert(tk.END, "시뮬레이션을 준비 중입니다...\n"); self.root.update_idletasks()
        try:
            specs = self._get_and_validate_specs()
            self.results_text.insert(tk.END, "차량 객체를 생성 중입니다...\n"); self.root.update_idletasks()
            engine = Engine(torque_curve={}, redline_rpm=specs['engine_redline_rpm'])
            motor = None
            if specs.get('motor_max_power_kw', 0) > 0 and specs.get('motor_redline_rpm', 0) > 0 :
                 motor = Motor(torque_curve={}, redline_rpm=specs['motor_redline_rpm'], max_power_kw=specs['motor_max_power_kw'])
            battery = None
            if motor and specs.get('battery_capacity_kwh', 0) > 0:
                 battery = Battery(capacity_kwh=specs['battery_capacity_kwh'], max_discharge_power_kw=specs['battery_max_discharge_power_kw'])
            elif motor and specs.get('battery_capacity_kwh', 0) <= 0:
                 messagebox.showwarning("배터리 경고", "모터가 정의되었으나 배터리 용량이 0 이하입니다. 모터 성능이 제한될 수 있습니다.")

            transmission = Transmission(
                gear_ratios=specs['gear_ratios'],
                final_gear_ratio=specs['final_gear_ratio'],
                efficiency=specs['drivetrain_efficiency_percent'] / 100.0
            )
            self.vehicle_obj_for_plotting = Vehicle(
                base_mass_kg=specs['mass_kg'],
                frontal_area_m2=specs['frontal_area_m2'], drag_coefficient=specs['drag_coefficient'],
                rolling_resistance_coefficient=specs['rolling_resistance_coefficient'], tire_radius_m=specs['tire_radius_m'],
                transmission=transmission, engine=engine, motor=motor, battery=battery,
                additional_mass_kg=specs['additional_mass_kg'],
                max_total_torque_nm=specs.get('max_total_torque_nm')
            )
            self.results_text.insert(tk.END, "토크 커브를 로드 중입니다...\n"); self.root.update_idletasks()
            if specs.get('engine_torque_csv'): self.vehicle_obj_for_plotting.load_torque_curves_from_csv(engine_csv_path=specs['engine_torque_csv'])
            if motor and specs.get('motor_torque_csv'): self.vehicle_obj_for_plotting.load_torque_curves_from_csv(motor_csv_path=specs['motor_torque_csv'])

            self.results_text.insert(tk.END, "가속 시뮬레이션을 실행 중입니다 (현재 경사도 적용)...\n"); self.root.update_idletasks()
            accel_targets_kmh = [60.0, 80.0, 100.0, 120.0]
            self.accel_results, self.simulation_results_df = simulate_acceleration(
                self.vehicle_obj_for_plotting, target_speeds_kmh=accel_targets_kmh, simulation_mode="hybrid",
                road_gradient_percent=specs['road_gradient_percent'],
                shift_time_s=specs['shift_time_s'],
                common_shift_rpm=specs['common_shift_rpm']
            )

            self.results_text.insert(tk.END, "최고 속도 시뮬레이션을 실행 중입니다 (평지)...\n"); self.root.update_idletasks()
            ts_flat_hybrid, _ = estimate_top_speed(self.vehicle_obj_for_plotting, mode="hybrid", road_gradient_percent=0.0, common_shift_rpm=specs['common_shift_rpm'])
            ts_flat_engine, _ = estimate_top_speed(self.vehicle_obj_for_plotting, mode="engine_only", road_gradient_percent=0.0, common_shift_rpm=specs['common_shift_rpm'])
            self.top_speed_results = {"평지 하이브리드": ts_flat_hybrid, "평지 엔진 단독": ts_flat_engine}

            self.results_text.insert(tk.END, "등판 성능을 계산 중입니다...\n"); self.root.update_idletasks()
            # Use a lowish RPM for gradeability torque, e.g. 15% of engine redline or a fixed min like 1000
            ref_rpm_for_grade = specs.get('engine_redline_rpm', 6000) * 0.15 if specs.get('engine_redline_rpm', 0) > 0 else 1000
            ref_rpm_for_grade = max(ref_rpm_for_grade, 800) # Ensure it's not too low
            self.max_gradeability_result = calculate_max_gradeability(self.vehicle_obj_for_plotting, reference_rpm_for_torque=ref_rpm_for_grade)

            self.uphill_top_speed_results = {}
            current_gradient = specs['road_gradient_percent']
            if abs(current_gradient) > 1e-3 :
                self.results_text.insert(tk.END, f"{current_gradient}% 경사로 등판/강판 최고 속도를 계산 중입니다...\n"); self.root.update_idletasks()
                uphill_ts_hybrid, _ = estimate_top_speed(self.vehicle_obj_for_plotting, mode="hybrid", road_gradient_percent=current_gradient, common_shift_rpm=specs['common_shift_rpm'])
                self.uphill_top_speed_results[f"{current_gradient}% 하이브리드"] = uphill_ts_hybrid
                uphill_ts_engine, _ = estimate_top_speed(self.vehicle_obj_for_plotting, mode="engine_only", road_gradient_percent=current_gradient, common_shift_rpm=specs['common_shift_rpm'])
                self.uphill_top_speed_results[f"{current_gradient}% 엔진 단독"] = uphill_ts_engine

            self.display_results_in_gui()
            messagebox.showinfo("시뮬레이션 완료", "모든 시뮬레이션 및 계산이 완료되었습니다.")
        except FileNotFoundError as e: messagebox.showerror("파일 오류", f"파일 찾기 실패: {e.filename}"); self.results_text.insert(tk.END, f"\n오류: 파일 찾기 실패 - {e.filename}\n"); self.results_text.config(state=tk.DISABLED)
        except ValueError: self.results_text.insert(tk.END, f"\n입력 값 오류 발생. 메시지를 확인하세요.\n"); self.results_text.config(state=tk.DISABLED)
        except Exception as e: messagebox.showerror("시뮬레이션 오류", f"예상치 못한 오류: {e}"); self.results_text.insert(tk.END, f"\n시뮬레이션 오류: {e}\n"); self.results_text.config(state=tk.DISABLED)

    def display_results_in_gui(self):
        self.results_text.config(state=tk.NORMAL)
        self.results_text.delete('1.0', tk.END)
        results_string = "--- 차량 성능 요약 ---\n\n"

        current_gradient_for_accel_str = self.spec_entries['road_gradient_percent'].get()
        try: current_gradient_for_accel = float(current_gradient_for_accel_str)
        except: current_gradient_for_accel = 0.0

        if self.accel_results:
            results_string += f"가속 성능 (현재 경사도: {current_gradient_for_accel:.1f}% 적용):\n"
            sorted_targets = sorted([float(k) for k in self.accel_results.keys()])
            for target_kmh in sorted_targets:
                time_val = self.accel_results[target_kmh] # Key is already float
                if time_val == float('inf'): results_string += f"  0-{int(target_kmh)} km/h: 목표 도달 실패\n"
                else: results_string += f"  0-{int(target_kmh)} km/h: {time_val:.2f} 초\n"

            if 80.0 in self.accel_results and 120.0 in self.accel_results:
                time_to_80 = self.accel_results[80.0]; time_to_120 = self.accel_results[120.0]
                if time_to_80!=float('inf') and time_to_120!=float('inf') and time_to_120 >= time_to_80:
                    results_string += f"  80-120 km/h: {time_to_120 - time_to_80:.2f} 초\n"
                else: results_string += "  80-120 km/h: 해당 구간 계산 불가\n"
        else: results_string += "가속 성능: 데이터 없음\n"

        results_string += "\n최고 속도 (평지):\n"
        if self.top_speed_results:
            for mode, speed_kmh in self.top_speed_results.items():
                if speed_kmh <= 0.01: results_string += f"  {mode}: {speed_kmh:.2f} km/h (거의 정지 또는 계산 불가)\n"
                else: results_string += f"  {mode}: {speed_kmh:.2f} km/h\n"
        else: results_string += "평지 최고 속도: 데이터 없음\n"

        results_string += "\n등판 성능:\n"
        if self.max_gradeability_result is not None:
            if math.isinf(self.max_gradeability_result) and self.max_gradeability_result > 0: results_string += "  발진 가능 최대 구배: 거의 수직 또는 그 이상 (이론상)\n"
            elif self.max_gradeability_result == -999.0: results_string += "  발진 가능 최대 구배: 계산 오류 (기어 또는 토크 설정 확인)\n"
            else: results_string += f"  발진 가능 최대 구배: {self.max_gradeability_result:.2f} %\n"
        else: results_string += "  발진 가능 최대 구배: 계산되지 않음\n"

        if self.uphill_top_speed_results and abs(current_gradient_for_accel) > 1e-3 :
            results_string += f"\n입력 경사도 ({current_gradient_for_accel_str}%) 등판/강판 최고 속도:\n"
            for mode, speed_kmh in self.uphill_top_speed_results.items():
                mode_name = mode.replace(f"{current_gradient_for_accel_str}% ", "")
                if speed_kmh <= 0.01: results_string += f"  {mode_name}: {speed_kmh:.2f} km/h (주행 불가 또는 정지)\n"
                else: results_string += f"  {mode_name}: {speed_kmh:.2f} km/h\n"
        elif abs(current_gradient_for_accel) <= 1e-3:
            results_string += f"  (평지에서의 최고 속도는 위 '최고 속도 (평지)' 섹션 참조)\n"

        results_string += "\n연비:\n  기본 연비 예측: 제공되지 않음.\n\n-----------------------------------\n"
        self.results_text.insert(tk.END, results_string)
        self.results_text.config(state=tk.DISABLED)

    def show_speed_time_plot(self):
        if self.simulation_results_df is not None and not self.simulation_results_df.empty: plot_speed_vs_time(self.simulation_results_df, title="속도 vs 시간 (시뮬레이션)")
        else: messagebox.showinfo("데이터 없음", "그래프를 표시할 시뮬레이션 데이터가 없습니다. 먼저 시뮬레이션을 실행하세요.")
    def show_torque_rpm_plot(self):
        if self.vehicle_obj_for_plotting: plot_torque_rpm_curves(self.vehicle_obj_for_plotting, simulation_df=self.simulation_results_df, title="토크 vs RPM")
        else: messagebox.showinfo("데이터 없음", "차량 객체가 생성되지 않았습니다. 먼저 시뮬레이션을 실행하세요.")
    def show_power_rpm_plot(self):
        if self.vehicle_obj_for_plotting: plot_power_rpm_curves(self.vehicle_obj_for_plotting, simulation_df=self.simulation_results_df, title="파워 vs RPM")
        else: messagebox.showinfo("데이터 없음", "차량 객체가 생성되지 않았습니다. 먼저 시뮬레이션을 실행하세요.")
    def show_accel_map_plot(self):
        if self.vehicle_obj_for_plotting:
            accel_map_data = get_gear_dependent_acceleration_map_data(self.vehicle_obj_for_plotting)
            if not accel_map_data.empty: plot_acceleration_map(accel_map_data, title="기어별 최대 가속도")
            else: messagebox.showwarning("데이터 없음", "가속도 맵 데이터를 생성할 수 없습니다.")
        else: messagebox.showinfo("데이터 없음", "차량 객체가 생성되지 않았습니다. 먼저 시뮬레이션을 실행하세요.")

if __name__ == '__main__':
    root = tk.Tk()
    app = VehicleSimApp(root)
    root.mainloop()
