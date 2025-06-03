# hybrid_vehicle_simulation/tests/test_core.py
import unittest
import pandas as pd
import math
import sys
import os

# Adjust path to import from src directory
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir) # This should be hybrid_vehicle_simulation/
sys.path.append(os.path.join(project_root, 'src'))

try:
    from core import Engine, Motor, Transmission, Battery, Vehicle
except ImportError as e:
    print(f"Error importing core modules in test_core.py: {e}")
    # Attempting a common alternative if the above fails (e.g. running from root of a larger project)
    # This assumes 'hybrid_vehicle_simulation' is a subdir of the CWD.
    sys.path.insert(0, os.path.join(os.getcwd(), 'hybrid_vehicle_simulation', 'src'))
    sys.path.insert(0, os.path.join(os.getcwd(), 'src')) # If src is directly under CWD
    from core import Engine, Motor, Transmission, Battery, Vehicle


class TestEngine(unittest.TestCase):
    def setUp(self):
        self.engine_torque_curve = pd.Series([100, 200, 180], index=[1000, 3000, 5000])
        self.engine = Engine(torque_curve=self.engine_torque_curve, redline_rpm=6000)

    def test_engine_get_torque(self):
        self.assertEqual(self.engine.get_torque(3000), 200)
        # Test below min RPM in curve: core.py's Engine.get_torque finds closest RPM, which is 1000 for 500.
        self.assertEqual(self.engine.get_torque(500), 100)
        self.assertEqual(self.engine.get_torque(7000), 0)  # Above redline
        self.assertEqual(self.engine.get_torque(0), 0) # At 0 RPM

    def test_engine_get_power(self):
        # Power (kW) = Torque (Nm) * RPM / 9549.3
        rpm = 3000
        torque = self.engine.get_torque(rpm) # 200 Nm
        expected_power = (torque * rpm) / 9549.3
        self.assertAlmostEqual(self.engine.get_power(rpm), expected_power)
        self.assertEqual(self.engine.get_power(0), 0)
        self.assertEqual(self.engine.get_power(7000), 0) # Above redline


class TestMotor(unittest.TestCase):
    def setUp(self):
        self.motor_torque_curve = pd.Series([150, 150, 100, 50], index=[0, 2000, 4000, 6000])
        self.motor = Motor(torque_curve=self.motor_torque_curve, redline_rpm=7000, max_power_kw=50)

    def test_motor_get_torque(self):
        # Test within flat torque, before power limit hits
        self.assertEqual(self.motor.get_torque(1000), 150)

        # Test where power limit might apply
        # Torque_max = Power_max_kW * 9549.3 / RPM
        # At 4000 RPM, torque from curve is 100 Nm. Power = 100 * 4000 / 9549.3 = 41.88 kW ( < 50kW)
        self.assertEqual(self.motor.get_torque(4000), 100)

        # At 6000 RPM, torque from curve is 50 Nm. Power = 50 * 6000 / 9549.3 = 31.41 kW ( < 50kW)
        self.assertEqual(self.motor.get_torque(6000), 50)

        # Test RPM where power limit dictates torque:
        # RPM where 150Nm (base torque up to 2000rpm) would exceed 50kW:
        # Power = 150 * RPM / 9549.3 = 50kW => RPM = 50 * 9549.3 / 150 = 3183.1 RPM
        # So, for RPM > 3183.1, if curve torque is higher than power limited torque, power limit applies.
        # At 3500 RPM:
        # Curve torque (interpolated between 150@2k and 100@4k for 3500rpm):
        # Slope = (100-150)/(4000-2000) = -50/2000 = -0.025
        # Torque_3500 = 150 + (-0.025 * (3500-2000)) = 150 - 0.025*1500 = 150 - 37.5 = 112.5 Nm
        # Power limited torque at 3500 RPM = 50 * 9549.3 / 3500 = 136.41 Nm
        # Since 112.5 Nm < 136.41 Nm, the curve torque (112.5 Nm) should be returned.
        self.assertAlmostEqual(self.motor.get_torque(3500), 112.5, places=1)

        # Test torque at 0 RPM
        self.assertEqual(self.motor.get_torque(0), 150)
        self.assertEqual(self.motor.get_torque(8000), 0) # Above redline


class TestTransmission(unittest.TestCase):
    def setUp(self):
        self.transmission = Transmission(gear_ratios=[3.0, 2.0, 1.0], final_gear_ratio=3.5, efficiency=0.9)

    def test_get_torque_multiplier(self):
        # Gear 1: 3.0 * 3.5 * 0.9 = 9.45
        self.assertAlmostEqual(self.transmission.get_torque_multiplier(1), 9.45)
        # Gear 3: 1.0 * 3.5 * 0.9 = 3.15
        self.assertAlmostEqual(self.transmission.get_torque_multiplier(3), 3.15)
        self.assertEqual(self.transmission.get_torque_multiplier(0), 0) # Invalid gear
        self.assertEqual(self.transmission.get_torque_multiplier(4), 0) # Invalid gear


class TestBattery(unittest.TestCase):
    def setUp(self):
        self.battery = Battery(capacity_kwh=10.0, max_discharge_power_kw=50.0)

    def test_battery_initialization(self):
        self.assertEqual(self.battery.capacity_kwh, 10.0)
        self.assertEqual(self.battery.max_discharge_power_kw, 50.0)


class TestVehicle(unittest.TestCase):
    def setUp(self):
        # Basic components for vehicle tests
        self.engine = Engine(pd.Series([200], index=[3000]), 6000)
        self.motor = Motor(pd.Series([100,80], index=[2000,4000]), 5000, 30) # max_power_kw=30
        self.battery = Battery(10, 35) # max_discharge_power_kw=35
        self.transmission = Transmission([3.0, 2.0], 3.0, 0.9) # efficiency = 0.9

        self.vehicle = Vehicle(
            mass_kg=1000.0,
            frontal_area_m2=2.0,
            drag_coefficient=0.3,
            rolling_resistance_coefficient=0.01,
            tire_radius_m=0.3,
            transmission=self.transmission,
            engine=self.engine,
            motor=self.motor,
            battery=self.battery
        )

    def test_calculate_resistive_forces(self):
        # F_drag = 0.5 * rho * A * Cd * v^2
        # F_roll = mu * m * g
        # At 0 m/s, drag is 0. Roll_res = 0.01 * 1000 * 9.81 = 98.1 N
        self.assertAlmostEqual(self.vehicle.calculate_resistive_forces(0), 98.1)
        # At 20 m/s (72 km/h)
        # Drag = 0.5 * 1.225 * 2.0 * 0.3 * (20**2) = 0.5 * 1.225 * 2.0 * 0.3 * 400 = 147 N
        # Total = 147 + 98.1 = 245.1 N
        self.assertAlmostEqual(self.vehicle.calculate_resistive_forces(20), 245.1)

    def test_calculate_rpm_methods(self):
        # Vehicle speed = 20 m/s, Gear = 1 (overall ratio 3.0 * 3.0 = 9.0)
        # Wheel RPM = (20 / (2 * pi * 0.3)) * 60 = (20 / 1.884955...) * 60 = 10.610329... * 60 = 636.6197... RPM
        # Source RPM = Wheel RPM * Gear_Ratio = 636.6197... * 9.0 = 5729.5779... RPM
        self.assertAlmostEqual(self.vehicle.calculate_wheel_rpm(20), 636.61977, places=4)
        self.assertAlmostEqual(self.vehicle.calculate_engine_rpm(20, 1), 5729.5779, places=3)
        self.assertAlmostEqual(self.vehicle.calculate_motor_rpm(20, 1), 5729.5779, places=3)


    def test_get_combined_torque_modes(self):
        engine_rpm = 3000 # Engine torque = 200 Nm (from self.engine setup)
        motor_rpm = 2000  # Motor torque from curve = 100 Nm (from self.motor setup)
                          # Power for this motor torque = 100Nm * 2000RPM / 9549.3 = 20.94 kW
                          # This is < motor.max_power_kw (30kW) and < battery.max_discharge_power_kw (35kW)

        # Hybrid mode
        # Expected: 200 (engine) + 100 (motor) = 300 Nm
        self.assertAlmostEqual(self.vehicle.get_combined_torque(engine_rpm, motor_rpm, 1.0, mode="hybrid"), 300)

        # Engine only mode
        # Expected: 200 (engine)
        self.assertAlmostEqual(self.vehicle.get_combined_torque(engine_rpm, motor_rpm, 1.0, mode="engine_only"), 200)

        # Motor only mode
        # Expected: 100 (motor)
        self.assertAlmostEqual(self.vehicle.get_combined_torque(engine_rpm, motor_rpm, 1.0, mode="motor_only"), 100)

        # Test motor torque limited by motor's max_power_kw (30kW)
        # Motor curve: 100Nm @ 2000rpm, 80Nm @ 4000rpm. max_power_kw=30kW for motor.
        # At 4000 RPM, curve torque is 80Nm. Potential power = 80Nm * 4000RPM / 9549.3 = 33.51 kW.
        # This exceeds motor.max_power_kw (30kW).
        # So, motor torque should be limited to: 30kW * 9549.3 / 4000RPM = 71.61975 Nm.
        # This is also below battery.max_discharge_power_kw (35kW), so motor limit applies.
        self.assertAlmostEqual(self.vehicle.get_combined_torque(engine_rpm, 4000, 1.0, mode="motor_only"), 71.61975, places=4)

        # Test motor torque limited by battery's max_discharge_power_kw (25kW for this test case)
        self.vehicle.battery.max_discharge_power_kw = 25 # Update battery limit for this specific test
        # Motor's own max_power_kw is 30kW.
        # At 4000 RPM, curve torque is 80Nm. Potential power from curve = 33.51kW.
        # Motor can deliver up to 30kW. Battery can deliver up to 25kW.
        # So, the effective power limit for the motor is min(30kW, 25kW) = 25kW.
        # Expected torque = 25kW * 9549.3 / 4000RPM = 59.683125 Nm.
        self.assertAlmostEqual(self.vehicle.get_combined_torque(engine_rpm, 4000, 1.0, mode="motor_only"), 59.683125, places=4)

        # Hybrid mode should also reflect this battery limit for the motor part
        # Expected: 200 (engine) + 59.683125 (motor_limited_by_battery) = 259.683125
        self.assertAlmostEqual(self.vehicle.get_combined_torque(engine_rpm, 4000, 1.0, mode="hybrid"), 200 + 59.683125, places=4)


    def test_calculate_tractive_force(self):
        # Combined torque = 300 Nm (from previous test, hybrid mode, low RPMs)
        # Gear 1 multiplier = 3.0 (gear) * 3.0 (final) * 0.9 (eff) = 8.1
        # Torque at wheels = 300 * 8.1 = 2430 Nm
        # Tractive force = Torque at wheels / tire_radius = 2430 / 0.3 = 8100 N
        self.assertAlmostEqual(self.vehicle.calculate_tractive_force(300, 1), 8100)


if __name__ == '__main__':
    unittest.main()
