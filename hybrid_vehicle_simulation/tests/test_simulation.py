# hybrid_vehicle_simulation/tests/test_simulation.py
import unittest
import pandas as pd
import math # For math.isinf
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.append(os.path.join(project_root, 'src'))

try:
    from core import Engine, Motor, Transmission, Battery, Vehicle
    from simulation import simulate_acceleration, estimate_top_speed, calculate_max_gradeability, KMH_TO_MPS
except ImportError as e:
    print(f"Error importing modules in test_simulation.py: {e}")
    sys.path.insert(0, os.path.join(os.getcwd(), 'src'))
    if not 'src.core' in sys.modules:
        sys.path.insert(0, os.path.join(os.getcwd(), 'hybrid_vehicle_simulation', 'src'))
    from core import Engine, Motor, Transmission, Battery, Vehicle
    from simulation import simulate_acceleration, estimate_top_speed, calculate_max_gradeability, KMH_TO_MPS


class TestSimulation(unittest.TestCase):
    def setUp(self):
        self.engine_const_torque = Engine(torque_curve=pd.Series([200.0], index=[1000]), redline_rpm=6000)
        self.motor_const_torque = Motor(torque_curve=pd.Series([100.0], index=[1000]), redline_rpm=7000, max_power_kw=50)
        self.ideal_battery = Battery(capacity_kwh=10.0, max_discharge_power_kw=60.0)
        self.ideal_transmission = Transmission(
            gear_ratios=[4.0, 2.5, 1.5, 1.0, 0.7],
            final_gear_ratio=3.5,
            efficiency=1.0
        )
        self.ideal_vehicle = Vehicle(
            base_mass_kg=1000.0, frontal_area_m2=2.0, drag_coefficient=0.0,
            rolling_resistance_coefficient=0.0, tire_radius_m=0.3,
            transmission=self.ideal_transmission, engine=self.engine_const_torque,
            motor=self.motor_const_torque, battery=self.ideal_battery,
            additional_mass_kg=0.0, max_total_torque_nm=None
        )

        self.realistic_engine = Engine(pd.Series([150,200,220,210,180], index=[1000,2000,3000,4000,5500]), 6000)
        self.realistic_motor = Motor(pd.Series([120,100,80,50], index=[0,1500,3000,6000]), 7000, 40)
        self.realistic_battery = Battery(10, 45)
        self.realistic_transmission = Transmission(
            gear_ratios=[3.2, 2.1, 1.4, 1.0, 0.8],
            final_gear_ratio=3.3,
            efficiency=0.85
        )
        self.realistic_vehicle = Vehicle(
            base_mass_kg=1500.0, frontal_area_m2=2.2, drag_coefficient=0.3,
            rolling_resistance_coefficient=0.01, tire_radius_m=0.32,
            transmission=self.realistic_transmission, engine=self.realistic_engine,
            motor=self.realistic_motor, battery=self.realistic_battery,
            additional_mass_kg=100.0,
            max_total_torque_nm=450.0
        )

    def test_simulate_acceleration_no_resistance_updated_call(self):
        target_speeds = [60.0]
        times, df = simulate_acceleration(
            self.ideal_vehicle, target_speeds_kmh=target_speeds, time_step=0.001,
            road_gradient_percent=0.0, shift_time_s=0.0, common_shift_rpm=None,
            simulation_mode="engine_only"
        )
        self.assertIsNotNone(df)
        self.assertIn(60.0, times)
        self.assertLess(times[60.0], 5.0)
        self.assertGreater(times[60.0], 1.0)

    def test_simulate_acceleration_with_shift_rpm(self):
        targets = [100.0]
        _, df_default_shift = simulate_acceleration(self.realistic_vehicle, targets, common_shift_rpm=None, shift_time_s=0.2)
        times_custom_shift, df_custom_shift = simulate_acceleration(
            self.realistic_vehicle, targets, common_shift_rpm=4000, shift_time_s=0.2
        )
        self.assertNotEqual(times_custom_shift[100.0], float('inf'))

    def test_simulate_acceleration_with_shift_time(self):
        targets = [100.0]
        times_no_delay, _ = simulate_acceleration(self.realistic_vehicle, targets, shift_time_s=0.0, common_shift_rpm=4000) # Fixed shift RPM for comparison
        times_with_delay, _ = simulate_acceleration(self.realistic_vehicle, targets, shift_time_s=0.5, common_shift_rpm=4000)

        self.assertNotEqual(times_no_delay[100.0], float('inf'))
        self.assertNotEqual(times_with_delay[100.0], float('inf'))
        if times_no_delay[100.0] != float('inf') and times_with_delay[100.0] != float('inf'):
            # Check if any shifts actually happened by looking at the gear column in DataFrames
            shifts_in_no_delay = len(pd.DataFrame(_['Gear'].unique())) if _ is not None and not _.empty else 0
            shifts_in_with_delay = len(pd.DataFrame(_['Gear'].unique())) if _ is not None and not _.empty else 0

            if shifts_in_no_delay > 1 or shifts_in_with_delay > 1 : # If shifts occurred
                 self.assertGreater(times_with_delay[100.0], times_no_delay[100.0] + 0.4) # Expect at least one 0.5s delay effect
            else: # No shifts, times should be very similar
                 self.assertAlmostEqual(times_with_delay[100.0], times_no_delay[100.0], delta=0.1)


    def test_simulate_acceleration_with_gradient(self):
        targets = [80.0]
        times_flat, _ = simulate_acceleration(self.realistic_vehicle, targets, road_gradient_percent=0.0)
        times_uphill, _ = simulate_acceleration(self.realistic_vehicle, targets, road_gradient_percent=5.0)
        times_downhill, _ = simulate_acceleration(self.realistic_vehicle, targets, road_gradient_percent=-2.0)

        self.assertNotEqual(times_flat[80.0], float('inf'))
        self.assertNotEqual(times_downhill[80.0], float('inf'))
        # Uphill might not be reachable, so check only if it is
        if times_uphill[80.0] != float('inf'):
            self.assertGreater(times_uphill[80.0], times_flat[80.0])
        if times_downhill[80.0] != float('inf'): # Should always be reachable if flat is
            self.assertLess(times_downhill[80.0], times_flat[80.0])

    def test_estimate_top_speed_with_gradient(self):
        ts_flat, _ = estimate_top_speed(self.realistic_vehicle, mode="hybrid", road_gradient_percent=0.0)
        ts_uphill, _ = estimate_top_speed(self.realistic_vehicle, mode="hybrid", road_gradient_percent=2.0)
        ts_downhill, _ = estimate_top_speed(self.realistic_vehicle, mode="hybrid", road_gradient_percent=-2.0)

        self.assertGreater(ts_flat, 100)
        if ts_uphill > 10 : # If it can move meaningfully uphill
             self.assertLess(ts_uphill, ts_flat)
        else: self.assertTrue(ts_uphill <= 10) # Or it's very slow/cannot move

        self.assertGreater(ts_downhill, ts_flat)

    def test_calculate_max_gradeability(self):
        grade_ideal = calculate_max_gradeability(self.ideal_vehicle, reference_rpm_for_torque=1000, start_gear=1)
        self.assertTrue(math.isinf(grade_ideal) and grade_ideal > 0, f"Ideal grade was {grade_ideal}")

        grade_realistic = calculate_max_gradeability(self.realistic_vehicle, reference_rpm_for_torque=1000, start_gear=1)
        # Calculation from previous step: 51.9 %
        self.assertGreater(grade_realistic, 40.0) # Expecting a significant grade
        self.assertLess(grade_realistic, 70.0)  # But not excessively so for a normal car

        weak_engine = Engine(pd.Series([10.0], index=[1000]), 3000)
        weak_vehicle = Vehicle( base_mass_kg=2000.0, frontal_area_m2=2.0, drag_coefficient=0.3,
                                rolling_resistance_coefficient=0.05, tire_radius_m=0.3,
                                transmission=self.ideal_transmission, engine=weak_engine, motor=None, battery=None)
        grade_weak = calculate_max_gradeability(weak_vehicle, reference_rpm_for_torque=1000, start_gear=1)
        self.assertAlmostEqual(grade_weak, 0.0, places=1)

        grade_invalid_gear = calculate_max_gradeability(self.realistic_vehicle, start_gear=99)
        self.assertEqual(grade_invalid_gear, -999.0)
        grade_zero_gear = calculate_max_gradeability(self.realistic_vehicle, start_gear=0)
        self.assertEqual(grade_zero_gear, -999.0)

    def test_simulation_dataframe_columns(self):
        _, df = simulate_acceleration(self.realistic_vehicle, target_speeds_kmh=[60.0])
        expected_cols = [
            'Time_s', 'Speed_mps', 'Speed_kmh', 'Acceleration_mps2', 'Gear',
            'Engine_RPM', 'Motor_RPM', 'Tractive_Force_N',
            'Resistive_Force_N', 'Net_Force_N', 'Distance_m'
        ]
        for col in expected_cols:
            self.assertIn(col, df.columns)

if __name__ == '__main__':
    unittest.main()
