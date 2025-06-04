# hybrid_vehicle_simulation/tests/test_core.py
import unittest
import pandas as pd
import math
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.append(os.path.join(project_root, 'src'))

try:
    from core import Engine, Motor, Transmission, Battery, Vehicle
except ImportError as e:
    print(f"Error importing core modules in test_core.py: {e}")
    # Fallback for different execution contexts if needed
    # This assumes 'src' is a subdirectory of the current working directory OR
    # 'hybrid_vehicle_simulation/src' is.
    sys.path.insert(0, os.path.join(os.getcwd(), 'src'))
    # If the above doesn't work, try assuming tests are run from one level above 'hybrid_vehicle_simulation'
    if not 'src.core' in sys.modules: # Check if previous attempt failed
        sys.path.insert(0, os.path.join(os.getcwd(), 'hybrid_vehicle_simulation', 'src'))
    from core import Engine, Motor, Transmission, Battery, Vehicle


class TestEngine(unittest.TestCase):
    def setUp(self):
        self.engine_torque_curve = pd.Series([100, 200, 180], index=[1000, 3000, 5000])
        self.engine = Engine(torque_curve=self.engine_torque_curve, redline_rpm=6000)
    def test_engine_get_torque(self):
        self.assertEqual(self.engine.get_torque(3000), 200)
        self.assertEqual(self.engine.get_torque(500), 100) # Closest RPM is 1000
        self.assertEqual(self.engine.get_torque(7000), 0)  # Above redline
        self.assertEqual(self.engine.get_torque(0), 0) # At 0 RPM
    def test_engine_get_power(self):
        rpm = 3000; torque = self.engine.get_torque(rpm); expected_power = (torque * rpm) / 9549.3
        self.assertAlmostEqual(self.engine.get_power(rpm), expected_power)
        self.assertEqual(self.engine.get_power(0), 0)
        self.assertEqual(self.engine.get_power(7000), 0)

class TestMotor(unittest.TestCase):
    def setUp(self):
        self.motor_torque_curve = pd.Series([150, 150, 100, 50], index=[0, 2000, 4000, 6000])
        self.motor = Motor(torque_curve=self.motor_torque_curve, redline_rpm=7000, max_power_kw=50)
    def test_motor_get_torque(self):
        self.assertEqual(self.motor.get_torque(1000), 150) # Closest to 0 or 2000, picks first if equidistant logic not specific
                                                       # With current core.py: min([0,2k,4k,6k], key=lambda x:abs(x-1000)) is 0 or 2000
                                                       # if 0 is picked, torque is 150. If 2000, torque is 150. So 150 is correct.
        self.assertEqual(self.motor.get_torque(4000), 100) # Exact match
        self.assertEqual(self.motor.get_torque(3000), 150) # Closest to 2000 (150Nm) vs 4000 (100Nm)
                                                       # abs(3000-2000)=1000, abs(3000-4000)=1000.
                                                       # If 2000 is earlier in sorted unique index, it's chosen by min().
        self.assertEqual(self.motor.get_torque(0), 150) # Exact match
        self.assertEqual(self.motor.get_torque(8000), 0) # Above redline

class TestTransmission(unittest.TestCase):
    def setUp(self):
        self.transmission_default_eff = Transmission(gear_ratios=[3.0, 2.0, 1.0], final_gear_ratio=3.5) # Uses default eff (e.g. 0.85)
        self.transmission_custom_eff = Transmission(gear_ratios=[3.0, 2.0, 1.0], final_gear_ratio=3.5, efficiency=0.9)

    def test_get_torque_multiplier(self):
        # Test with default efficiency (assuming it's 0.85 from core.py)
        # Gear 1: 3.0 * 3.5 * 0.85 = 8.925
        self.assertAlmostEqual(self.transmission_default_eff.get_torque_multiplier(1), 3.0 * 3.5 * 0.85)

        # Test with custom efficiency
        # Gear 1: 3.0 * 3.5 * 0.9 = 9.45
        self.assertAlmostEqual(self.transmission_custom_eff.get_torque_multiplier(1), 9.45)
        self.assertEqual(self.transmission_custom_eff.get_torque_multiplier(0), 0)
        self.assertEqual(self.transmission_custom_eff.get_torque_multiplier(4), 0)

    def test_transmission_efficiency_validation(self):
        with self.assertRaisesRegex(ValueError, "효율은 0보다 크고 1 이하여야 합니다."):
            Transmission(gear_ratios=[1.0], final_gear_ratio=1.0, efficiency=0.0)
        with self.assertRaisesRegex(ValueError, "효율은 0보다 크고 1 이하여야 합니다."):
            Transmission(gear_ratios=[1.0], final_gear_ratio=1.0, efficiency=1.1)
        try: # Valid
            Transmission(gear_ratios=[1.0], final_gear_ratio=1.0, efficiency=1.0)
            Transmission(gear_ratios=[1.0], final_gear_ratio=1.0, efficiency=0.1)
        except ValueError:
            self.fail("ValueError raised unexpectedly for valid efficiency.")

class TestBattery(unittest.TestCase):
    def setUp(self): self.battery = Battery(capacity_kwh=10.0, max_discharge_power_kw=50.0)
    def test_battery_initialization(self):
        self.assertEqual(self.battery.capacity_kwh, 10.0)
        self.assertEqual(self.battery.max_discharge_power_kw, 50.0)

class TestVehicle(unittest.TestCase):
    def setUp(self):
        self.engine = Engine(pd.Series([200], index=[3000]), 6000)
        self.motor = Motor(pd.Series([100], index=[2000]), 5000, 30)
        self.battery = Battery(10, 35)
        self.transmission = Transmission([3.0, 2.0], 3.0, efficiency=0.9)

        self.vehicle_base_params = {
            "base_mass_kg": 1000.0, "frontal_area_m2": 2.0, "drag_coefficient": 0.3,
            "rolling_resistance_coefficient": 0.01, "tire_radius_m": 0.3,
            "transmission": self.transmission, "engine": self.engine,
            "motor": self.motor, "battery": self.battery
        }
        self.vehicle = Vehicle(**self.vehicle_base_params)

    def test_vehicle_mass_initialization(self):
        self.assertEqual(self.vehicle.mass_kg, 1000.0)
        vehicle_with_add_mass = Vehicle(**self.vehicle_base_params, additional_mass_kg=200.0)
        self.assertEqual(vehicle_with_add_mass.mass_kg, 1200.0)

    def test_vehicle_max_total_torque_init(self):
        self.assertIsNone(self.vehicle.max_total_torque_nm)
        vehicle_with_torque_cap = Vehicle(**self.vehicle_base_params, max_total_torque_nm=250.0)
        self.assertEqual(vehicle_with_torque_cap.max_total_torque_nm, 250.0)

    def test_get_combined_torque_with_max_cap(self):
        engine_rpm = 3000; motor_rpm = 2000
        # Base engine torque = 200 Nm, Base motor torque = 100 Nm. Potential combined = 300 Nm.

        # No cap
        self.assertAlmostEqual(self.vehicle.get_combined_torque(engine_rpm, motor_rpm, mode="hybrid"), 300)

        # Cap above potential
        vehicle_high_cap = Vehicle(**self.vehicle_base_params, max_total_torque_nm=350.0)
        self.assertAlmostEqual(vehicle_high_cap.get_combined_torque(engine_rpm, motor_rpm, mode="hybrid"), 300)

        # Cap below potential
        vehicle_low_cap = Vehicle(**self.vehicle_base_params, max_total_torque_nm=250.0)
        self.assertAlmostEqual(vehicle_low_cap.get_combined_torque(engine_rpm, motor_rpm, mode="hybrid"), 250.0)

        # Cap with engine_only mode
        vehicle_engine_cap = Vehicle(**self.vehicle_base_params, max_total_torque_nm=150.0)
        self.assertAlmostEqual(vehicle_engine_cap.get_combined_torque(engine_rpm, motor_rpm, mode="engine_only"), 150.0)
        vehicle_engine_no_cap_needed = Vehicle(**self.vehicle_base_params, max_total_torque_nm=220.0)
        self.assertAlmostEqual(vehicle_engine_no_cap_needed.get_combined_torque(engine_rpm, motor_rpm, mode="engine_only"), 200.0)

    def test_calculate_resistive_forces_with_gradient(self):
        mass = self.vehicle_base_params["base_mass_kg"]
        rr_coeff = self.vehicle_base_params["rolling_resistance_coefficient"]
        gravity = self.vehicle.gravity # Should be 9.81

        # Zero gradient, zero speed
        expected_roll_res = rr_coeff * mass * gravity
        self.assertAlmostEqual(self.vehicle.calculate_resistive_forces(0, road_gradient_percent=0.0), expected_roll_res)

        # Zero gradient, 20 m/s speed
        drag_force_at_20mps = 0.5 * self.vehicle.air_density * self.vehicle.frontal_area_m2 * self.vehicle.drag_coefficient * (20**2)
        self.assertAlmostEqual(self.vehicle.calculate_resistive_forces(20, road_gradient_percent=0.0), expected_roll_res + drag_force_at_20mps)

        # 5% gradient, zero speed
        gradient_5_rad = math.atan(5.0 / 100.0)
        grade_force_5_percent = mass * gravity * math.sin(gradient_5_rad)
        self.assertAlmostEqual(self.vehicle.calculate_resistive_forces(0, road_gradient_percent=5.0), expected_roll_res + grade_force_5_percent)

        # -2% gradient, 20 m/s speed
        gradient_neg_2_rad = math.atan(-2.0 / 100.0)
        grade_force_neg_2_percent = mass * gravity * math.sin(gradient_neg_2_rad)
        self.assertAlmostEqual(self.vehicle.calculate_resistive_forces(20, road_gradient_percent=-2.0), expected_roll_res + drag_force_at_20mps + grade_force_neg_2_percent)

        # With additional mass
        vehicle_heavy = Vehicle(**self.vehicle_base_params, additional_mass_kg=500.0) # Total mass = 1500kg
        heavy_mass = 1500.0
        expected_roll_res_heavy = rr_coeff * heavy_mass * gravity
        grade_force_5_percent_heavy = heavy_mass * gravity * math.sin(gradient_5_rad)
        self.assertAlmostEqual(vehicle_heavy.calculate_resistive_forces(0, road_gradient_percent=5.0), expected_roll_res_heavy + grade_force_5_percent_heavy)

    def test_calculate_rpm_methods(self): # Should still pass
        self.assertAlmostEqual(self.vehicle.calculate_wheel_rpm(20), 636.61977, places=4)
        self.assertAlmostEqual(self.vehicle.calculate_engine_rpm(20, 1), 5729.5779, places=3)

    def test_calculate_tractive_force(self): # Should still pass
        self.assertAlmostEqual(self.vehicle.calculate_tractive_force(300, 1), 8100)

if __name__ == '__main__':
    unittest.main()
