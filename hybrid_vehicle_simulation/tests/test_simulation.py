# hybrid_vehicle_simulation/tests/test_simulation.py
import unittest
import pandas as pd
import sys
import os

# Adjust path to import from src directory
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.append(os.path.join(project_root, 'src'))

try:
    from core import Engine, Motor, Transmission, Battery, Vehicle
    from simulation import simulate_acceleration, estimate_top_speed, KMH_TO_MPS
except ImportError as e:
    print(f"Error importing modules in test_simulation.py: {e}")
    # Attempting a common alternative if the above fails
    sys.path.insert(0, os.path.join(os.getcwd(), 'hybrid_vehicle_simulation', 'src'))
    sys.path.insert(0, os.path.join(os.getcwd(), 'src'))
    from core import Engine, Motor, Transmission, Battery, Vehicle
    from simulation import simulate_acceleration, estimate_top_speed, KMH_TO_MPS


class TestSimulation(unittest.TestCase):
    def setUp(self):
        # Setup a simple vehicle that should be predictable
        # Engine: Constant torque for simplicity in some tests
        self.engine_const_torque = Engine(torque_curve=pd.Series([200.0], index=[1000]), redline_rpm=6000)
        # Motor: Constant torque
        self.motor_const_torque = Motor(torque_curve=pd.Series([100.0], index=[1000]), redline_rpm=7000, max_power_kw=50)

        self.battery = Battery(capacity_kwh=10.0, max_discharge_power_kw=60.0)
        self.transmission = Transmission(
            gear_ratios=[4.0, 2.5, 1.5, 1.0, 0.7], # Fairly standard ratios
            final_gear_ratio=3.5,
            efficiency=1.0 # Ideal efficiency for simpler test math
        )

        # Vehicle for no-resistance tests (drag/rolling coeff = 0)
        self.ideal_vehicle = Vehicle(
            mass_kg=1000.0,
            frontal_area_m2=2.0,
            drag_coefficient=0.0, # No air drag
            rolling_resistance_coefficient=0.0, # No rolling resistance
            tire_radius_m=0.3,
            transmission=self.transmission,
            engine=self.engine_const_torque,
            motor=self.motor_const_torque,
            battery=self.battery
        )

        # More realistic vehicle
        self.realistic_engine = Engine(pd.Series([150,200,180], index=[1000,3000,5500]), 6000)
        self.realistic_motor = Motor(pd.Series([100,80,50], index=[0,3000,6000]), 7000, 40)
        self.realistic_battery = Battery(10, 45)
        self.realistic_transmission = Transmission([3.0,2.0,1.3,1.0,0.8], 3.2, 0.9)
        self.realistic_vehicle = Vehicle(
            mass_kg=1500.0, frontal_area_m2=2.2, drag_coefficient=0.3,
            rolling_resistance_coefficient=0.01, tire_radius_m=0.32,
            transmission=self.realistic_transmission, engine=self.realistic_engine,
            motor=self.realistic_motor, battery=self.realistic_battery
        )


    def test_simulate_acceleration_no_resistance(self):
        # Test with ideal_vehicle (no resistance, constant torque from engine only)
        # Engine torque = 200 Nm. Motor not used in "engine_only"
        # Gear 1 total ratio = 4.0 * 3.5 * 1.0 (eff) = 14.0
        # Torque at wheels = 200 Nm * 14.0 = 2800 Nm
        # Tractive force = 2800 Nm / 0.3 m = 9333.33 N
        # Acceleration = Force / mass = 9333.33 N / 1000 kg = 9.333 m/s^2 (constant in gear 1 if RPM is in range)

        # Target: 0-60 km/h (16.667 m/s)
        # Ideal Time (if no shifts and const accel) = Speed / Acceleration = 16.667 m/s / 9.333 m/s^2 = 1.785 s
        target_speeds = [60.0]
        times, df = simulate_acceleration(
            self.ideal_vehicle,
            target_speeds_kmh=target_speeds,
            time_step=0.001, # Small time step for accuracy
            simulation_mode="engine_only"
        )

        self.assertIsNotNone(df)
        self.assertIn(60.0, times)
        # print(f"Test ideal 0-60 (engine_only): {times[60.0]:.3f}s (Expected ~1.785s without shifts)")
        # The simulation includes gear shifts and RPM dependencies for torque,
        # so it will be slower than the simplified constant acceleration calculation.
        # For this test, let's check if it's in a plausible range, e.g. < 5s for 0-60 with no resistance.
        self.assertLess(times[60.0], 5.0)
        self.assertGreater(times[60.0], 1.0) # Should take some time


    def test_simulate_acceleration_multi_segment_realistic(self):
        # Test with a more realistic vehicle and multiple segments
        targets = [60.0, 100.0, 120.0]
        times, df = simulate_acceleration(
            self.realistic_vehicle,
            target_speeds_kmh=targets,
            time_step=0.01,
            simulation_mode="hybrid"
        )
        self.assertIsNotNone(df)
        for target_speed in targets:
            self.assertIn(target_speed, times)
            # Basic sanity checks for the times
            if times[target_speed] != float('inf'):
                 self.assertLess(times[target_speed], 60.0) # Should reach 120kmh in less than 60s for a car
                 if target_speed > 60.0 and 60.0 in times and times[60.0] != float('inf'):
                     self.assertGreater(times[target_speed], times[60.0])
            else:
                print(f"Warning: Target {target_speed} km/h not reached in realistic accel test.")

        self.assertTrue(all(t != float('inf') for t in times.values()),
                        f"Not all targets reached in realistic accel test. Times: {times}")


    def test_estimate_top_speed_modes(self):
        # Hybrid mode should be faster than engine_only mode if motor contributes meaningfully
        top_speed_hybrid, df_hybrid = estimate_top_speed(self.realistic_vehicle, mode="hybrid", time_step=0.01)
        top_speed_engine_only, df_engine = estimate_top_speed(self.realistic_vehicle, mode="engine_only", time_step=0.01)

        self.assertGreater(top_speed_hybrid, 100) # Should be a reasonable speed for a car
        self.assertGreater(top_speed_engine_only, 80)

        # print(f"Top Speed Hybrid: {top_speed_hybrid:.2f} km/h")
        # print(f"Top Speed Engine Only: {top_speed_engine_only:.2f} km/h")

        if self.realistic_vehicle.motor and self.realistic_vehicle.motor.max_power_kw > 5: # Motor has some power
            self.assertGreaterEqual(top_speed_hybrid, top_speed_engine_only - 5) # Allow small delta for simulation variance

        top_speed_motor_only, df_motor = estimate_top_speed(self.realistic_vehicle, mode="motor_only", time_step=0.01)
        # print(f"Top Speed Motor Only: {top_speed_motor_only:.2f} km/h")
        self.assertGreater(top_speed_motor_only, 10) # Should be able to move, maybe low speed

        # Motor only top speed is often lower than engine only, unless it's a powerful EV motor
        if top_speed_engine_only > 30 : # if engine can achieve some meaningful speed
             self.assertLess(top_speed_motor_only, top_speed_engine_only + 20) # motor only is likely slower, allow some margin


    def test_simulation_dataframe_columns(self):
        # Check if the output DataFrame has the expected columns
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
