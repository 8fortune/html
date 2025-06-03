# hybrid_vehicle_simulation/main_gui.py
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import json
import os
import sys
import pandas as pd

# Adjust path to import from src directory
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'src'))

try:
    from core import Engine, Motor, Transmission, Battery, Vehicle
    from simulation import simulate_acceleration, estimate_top_speed
    from plotting import plot_speed_vs_time, plot_torque_rpm_curves, plot_power_rpm_curves, plot_acceleration_map
    from outputs import get_gear_dependent_acceleration_map_data
except ImportError as e:
    print(f"모듈 임포트 중 오류 발생: {e}. 'src' 디렉토리가 PYTHONPATH에 있는지 확인하거나, 프로젝트 루트에서 스크립트를 실행하세요.")
    sys.exit(1)

class VehicleSimApp:
    def __init__(self, root):
        self.root = root
        root.title("하이브리드 차량 성능 시뮬레이터 GUI")
        root.geometry("1000x800")

        self.spec_entries = {}
        self.simulation_results_df = None
        self.vehicle_obj_for_plotting = None
        self.accel_results = None
        self.top_speed_results = None

        # Define which spec keys must be positive numbers
        self.positive_numeric_specs = [
            "mass_kg", "frontal_area_m2", "tire_radius_m",
            "engine_redline_rpm",
            "final_gear_ratio"
        ]
        # motor/battery related specs can be zero if no motor/battery is intended.
        # drag_coefficient and rolling_resistance_coefficient can also be zero for ideal cases.

        main_container = ttk.Frame(root, padding="10 10 10 10")
        main_container.pack(expand=True, fill=tk.BOTH)

        top_pane = ttk.Frame(main_container)
        top_pane.pack(side=tk.TOP, fill=tk.X, expand=False, pady=5)

        self.input_frame = ttk.LabelFrame(top_pane, text="차량 제원 입력", padding="10 10")
        self.input_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        input_params = [
            ("차량 질량 (kg):", "mass_kg", "1500.0"), ("전면 면적 (m²):", "frontal_area_m2", "2.2"),
            ("항력 계수 (Cd):", "drag_coefficient", "0.28"), ("구름 저항 계수 (μ):", "rolling_resistance_coefficient", "0.01"),
            ("타이어 반경 (m):", "tire_radius_m", "0.32"), ("엔진 레드라인 RPM:", "engine_redline_rpm", "6500"),
            ("모터 레드라인 RPM:", "motor_redline_rpm", "8000"), ("배터리 용량 (kWh):", "battery_capacity_kwh", "10.0"),
            ("최대 모터 파워 (kW):", "motor_max_power_kw", "75.0"), ("최대 배터리 방전 파워 (kW):", "battery_max_discharge_power_kw", "80.0"),
            ("기어비 (쉼표 구분):", "gear_ratios", "3.6,2.1,1.4,1.0,0.75"), ("최종 감속비 (FGR):", "final_gear_ratio", "3.5"),
            ("엔진 토크 커브 CSV:", "engine_torque_csv", os.path.join(current_dir, 'data', 'sample_engine_torque.csv')),
            ("모터 토크 커브 CSV:", "motor_torque_csv", os.path.join(current_dir, 'data', 'sample_motor_torque.csv')),
        ]
        for i, (label_text, key, default_val) in enumerate(input_params):
            label = ttk.Label(self.input_frame, text=label_text)
            label.grid(row=i, column=0, sticky=tk.W, padx=5, pady=2)
            entry = ttk.Entry(self.input_frame, width=40)
            entry.grid(row=i, column=1, sticky=tk.EW, padx=5, pady=2)
            entry.insert(0, default_val)
            self.spec_entries[key] = entry
            if "_csv" in key:
                browse_button = ttk.Button(self.input_frame, text="찾아보기", command=lambda k=key: self.browse_file(k))
                browse_button.grid(row=i, column=2, sticky=tk.W, padx=5, pady=2)
        self.input_frame.columnconfigure(1, weight=1)

        self.control_frame = ttk.LabelFrame(top_pane, text="제어", padding="10 10")
        self.control_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        self.save_button = ttk.Button(self.control_frame, text="제원 저장", command=self.save_specifications)
        self.save_button.pack(fill=tk.X, padx=5, pady=5)
        self.load_button = ttk.Button(self.control_frame, text="제원 불러오기", command=self.load_specifications)
        self.load_button.pack(fill=tk.X, padx=5, pady=5)
        self.run_sim_button = ttk.Button(self.control_frame, text="시뮬레이션 실행", command=self.run_simulation_sequence)
        self.run_sim_button.pack(fill=tk.X, padx=5, pady=5, side=tk.BOTTOM)

        self.results_frame = ttk.LabelFrame(main_container, text="시뮬레이션 결과", padding="10 10")
        self.results_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.results_text = tk.Text(self.results_frame, wrap=tk.WORD, height=15)
        self.results_text.pack(expand=True, fill=tk.BOTH, padx=5, pady=5)
        self.results_text.insert(tk.END, "시뮬레이션 결과가 여기에 표시됩니다.")
        self.results_text.config(state=tk.DISABLED)

        self.graph_buttons_frame = ttk.LabelFrame(main_container, text="그래프 생성", padding="10 10")
        self.graph_buttons_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5, expand=False)
        btn_plot_speed_time = ttk.Button(self.graph_buttons_frame, text="속도-시간 그래프", command=self.show_speed_time_plot)
        btn_plot_speed_time.pack(side=tk.LEFT, padx=5, pady=5)
        btn_plot_torque_rpm = ttk.Button(self.graph_buttons_frame, text="토크-RPM 그래프", command=self.show_torque_rpm_plot)
        btn_plot_torque_rpm.pack(side=tk.LEFT, padx=5, pady=5)
        btn_plot_power_rpm = ttk.Button(self.graph_buttons_frame, text="파워-RPM 그래프", command=self.show_power_rpm_plot)
        btn_plot_power_rpm.pack(side=tk.LEFT, padx=5, pady=5)
        btn_plot_accel_map = ttk.Button(self.graph_buttons_frame, text="가속도 맵", command=self.show_accel_map_plot)
        btn_plot_accel_map.pack(side=tk.LEFT, padx=5, pady=5)

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
            # Convert numeric types
            processed_specs['mass_kg'] = float(raw_specs['mass_kg'])
            processed_specs['frontal_area_m2'] = float(raw_specs['frontal_area_m2'])
            processed_specs['drag_coefficient'] = float(raw_specs['drag_coefficient'])
            processed_specs['rolling_resistance_coefficient'] = float(raw_specs['rolling_resistance_coefficient'])
            processed_specs['tire_radius_m'] = float(raw_specs['tire_radius_m'])
            processed_specs['engine_redline_rpm'] = int(raw_specs['engine_redline_rpm'])
            processed_specs['motor_redline_rpm'] = int(raw_specs['motor_redline_rpm'])
            processed_specs['battery_capacity_kwh'] = float(raw_specs['battery_capacity_kwh'])
            processed_specs['motor_max_power_kw'] = float(raw_specs['motor_max_power_kw'])
            processed_specs['battery_max_discharge_power_kw'] = float(raw_specs['battery_max_discharge_power_kw'])
            processed_specs['final_gear_ratio'] = float(raw_specs['final_gear_ratio'])

            gear_ratios_str = raw_specs['gear_ratios'].split(',')
            processed_specs['gear_ratios'] = []
            for gr_str in gear_ratios_str:
                gr_str = gr_str.strip()
                if not gr_str: continue
                gr_val = float(gr_str)
                processed_specs['gear_ratios'].append(gr_val)
            if not processed_specs['gear_ratios']: raise ValueError("기어비 목록이 비어있습니다.")

            processed_specs['engine_torque_csv'] = raw_specs['engine_torque_csv'].strip()
            processed_specs['motor_torque_csv'] = raw_specs['motor_torque_csv'].strip()

            # Validate positive values for fields that must be positive
            for key_check in self.positive_numeric_specs:
                if key_check in processed_specs and processed_specs[key_check] <= 0:
                    raise ValueError(f"'{key_check}' 값은 0보다 커야 합니다.") # Using key_check directly for error

            # Validate individual gear ratios must be positive
            for gr_val in processed_specs.get('gear_ratios', []):
                if gr_val <= 0:
                    raise ValueError("개별 기어비는 0보다 커야 합니다.")

            # Validate motor/battery specs if motor power > 0
            if processed_specs['motor_max_power_kw'] > 0:
                if processed_specs['motor_redline_rpm'] <= 0:
                    raise ValueError("'모터 레드라인 RPM'은 모터 파워가 0보다 클 경우 0보다 커야 합니다.")
                if processed_specs['battery_capacity_kwh'] <= 0:
                     raise ValueError("'배터리 용량 (kWh)'은 모터 파워가 0보다 클 경우 0보다 커야 합니다.")
                if processed_specs['battery_max_discharge_power_kw'] <= 0:
                     raise ValueError("'최대 배터리 방전 파워 (kW)'는 모터 파워가 0보다 클 경우 0보다 커야 합니다.")

            # Check for CSV file paths (optional, can be empty if no engine/motor)
            # if not processed_specs['engine_torque_csv'] and processed_specs['engine_redline_rpm'] > 0 : # If engine exists
            #    messagebox.showwarning("입력 경고", "엔진 토크 커브 CSV 경로가 지정되지 않았습니다. 엔진은 토크 없이 작동합니다.")
            # if not processed_specs['motor_torque_csv'] and processed_specs['motor_max_power_kw'] > 0: # If motor exists
            #    messagebox.showwarning("입력 경고", "모터 토크 커브 CSV 경로가 지정되지 않았습니다. 모터는 토크 없이 작동합니다.")


            return processed_specs
        except ValueError as e:
            messagebox.showerror("입력 오류", f"제원 값 변환 또는 유효성 검사 중 오류: {e}\n입력된 숫자와 형식을 확인해주세요.")
            raise
        except Exception as e:
            messagebox.showerror("입력 오류", f"알 수 없는 제원 처리 오류: {e}")
            raise

    def display_results_in_gui(self):
        self.results_text.config(state=tk.NORMAL)
        self.results_text.delete('1.0', tk.END)
        results_string = "--- 차량 성능 요약 ---\n\n"
        if self.accel_results:
            results_string += "가속 성능:\n"
            sorted_targets = sorted(self.accel_results.keys())
            for target_kmh in sorted_targets:
                time_val = self.accel_results[target_kmh]
                if time_val == float('inf'): results_string += f"  0-{int(target_kmh)} km/h: 목표 도달 실패\n"
                else: results_string += f"  0-{int(target_kmh)} km/h: {time_val:.2f} 초\n"
            if 80.0 in self.accel_results and 120.0 in self.accel_results:
                time_to_80 = self.accel_results[80.0]; time_to_120 = self.accel_results[120.0]
                if time_to_80!=float('inf') and time_to_120!=float('inf'): results_string += f"  80-120 km/h: {time_to_120 - time_to_80:.2f} 초\n"
                else: results_string += "  80-120 km/h: 해당 구간 계산 불가 (목표 도달 실패 등)\n"
        else: results_string += "가속 성능: 시뮬레이션 데이터 없음\n"
        results_string += "\n최고 속도:\n"
        if self.top_speed_results:
            for mode, speed_kmh in self.top_speed_results.items():
                if speed_kmh <= 0: results_string += f"  {mode}: 계산 불가 또는 0 km/h\n"
                else: results_string += f"  {mode}: {speed_kmh:.2f} km/h\n"
        else: results_string += "최고 속도: 시뮬레이션 데이터 없음\n"
        results_string += "\n연비:\n  기본 연비 예측: 제공되지 않음 (기본 시뮬레이션).\n"
        results_string += "\n-----------------------------------\n"
        self.results_text.insert(tk.END, results_string)
        self.results_text.config(state=tk.DISABLED)

    def run_simulation_sequence(self):
        self.results_text.config(state=tk.NORMAL)
        self.results_text.delete('1.0', tk.END)
        self.results_text.insert(tk.END, "시뮬레이션을 준비 중입니다...\n")
        self.root.update_idletasks()

        try:
            specs = self._get_and_validate_specs()

            self.results_text.insert(tk.END, "차량 객체를 생성 중입니다...\n")
            self.root.update_idletasks()

            engine = Engine(torque_curve={}, redline_rpm=specs['engine_redline_rpm'])
            motor = None
            if specs.get('motor_max_power_kw', 0) > 0 and specs.get('motor_redline_rpm', 0) > 0:
                 motor = Motor(torque_curve={}, redline_rpm=specs['motor_redline_rpm'], max_power_kw=specs['motor_max_power_kw'])

            battery = None
            if motor : # Battery only makes sense if there's a motor with capacity
                if specs.get('battery_capacity_kwh', 0) > 0 :
                    battery = Battery(capacity_kwh=specs['battery_capacity_kwh'], max_discharge_power_kw=specs['battery_max_discharge_power_kw'])
                else: # Motor present but no battery capacity, effectively means motor cannot be used.
                    messagebox.showwarning("입력 경고", "모터가 있으나 배터리 용량이 0입니다. 모터는 작동하지 않습니다.")
                    motor = None # Disable motor if no battery capacity

            transmission = Transmission(gear_ratios=specs['gear_ratios'], final_gear_ratio=specs['final_gear_ratio'], efficiency=0.92)

            self.vehicle_obj_for_plotting = Vehicle(
                mass_kg=specs['mass_kg'], frontal_area_m2=specs['frontal_area_m2'], drag_coefficient=specs['drag_coefficient'],
                rolling_resistance_coefficient=specs['rolling_resistance_coefficient'], tire_radius_m=specs['tire_radius_m'],
                transmission=transmission, engine=engine, motor=motor, battery=battery
            )

            self.results_text.insert(tk.END, "토크 커브를 로드 중입니다...\n")
            self.root.update_idletasks()

            if specs.get('engine_torque_csv'):
                self.vehicle_obj_for_plotting.load_torque_curves_from_csv(engine_csv_path=specs['engine_torque_csv'])
                if not isinstance(self.vehicle_obj_for_plotting.engine.torque_curve, pd.Series) or self.vehicle_obj_for_plotting.engine.torque_curve.empty:
                     messagebox.showwarning("토크 커브 경고", f"엔진 토크 커브 로드 실패 또는 데이터 없음: {specs['engine_torque_csv']}")

            if motor and specs.get('motor_torque_csv'):
                self.vehicle_obj_for_plotting.load_torque_curves_from_csv(motor_csv_path=specs['motor_torque_csv'])
                if not isinstance(self.vehicle_obj_for_plotting.motor.torque_curve, pd.Series) or self.vehicle_obj_for_plotting.motor.torque_curve.empty:
                     messagebox.showwarning("토크 커브 경고", f"모터 토크 커브 로드 실패 또는 데이터 없음: {specs['motor_torque_csv']}")

            self.results_text.insert(tk.END, "가속 시뮬레이션을 실행 중입니다...\n")
            self.root.update_idletasks()

            accel_targets_kmh = [60.0, 80.0, 100.0, 120.0]
            self.accel_results, self.simulation_results_df = simulate_acceleration(self.vehicle_obj_for_plotting, target_speeds_kmh=accel_targets_kmh, simulation_mode="hybrid")

            self.results_text.insert(tk.END, "최고 속도 시뮬레이션을 실행 중입니다 (하이브리드 모드)...\n")
            self.root.update_idletasks()
            top_speed_hybrid, _ = estimate_top_speed(self.vehicle_obj_for_plotting, mode="hybrid")

            self.results_text.insert(tk.END, "최고 속도 시뮬레이션을 실행 중입니다 (엔진 단독 모드)...\n")
            self.root.update_idletasks()
            top_speed_engine_only, _ = estimate_top_speed(self.vehicle_obj_for_plotting, mode="engine_only")

            self.top_speed_results = {"하이브리드 모드": top_speed_hybrid, "엔진 단독 모드": top_speed_engine_only}

            self.display_results_in_gui()
            messagebox.showinfo("시뮬레이션 완료", "시뮬레이션이 성공적으로 완료되었습니다.")

        except FileNotFoundError as e:
            messagebox.showerror("파일 오류", f"필요한 파일을 찾을 수 없습니다: {e.filename}")
            self.results_text.insert(tk.END, f"\n오류: 파일을 찾을 수 없습니다 - {e.filename}\n")
            self.results_text.config(state=tk.DISABLED)
        except ValueError:
            self.results_text.insert(tk.END, f"\n입력 값 오류가 발생했습니다. 메시지를 확인하세요.\n")
            self.results_text.config(state=tk.DISABLED)
        except Exception as e:
            messagebox.showerror("시뮬레이션 오류", f"시뮬레이션 중 예상치 못한 오류 발생: {e}")
            self.results_text.insert(tk.END, f"\n시뮬레이션 오류: {e}\n")
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
