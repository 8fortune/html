# hybrid_vehicle_simulation/src/plotting.py
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from typing import Optional

# Assuming core.py, outputs.py might be needed for type hints or data structures
# For direct execution, ensure these are discoverable or use relative imports in a package structure.
try:
    from outputs import get_torque_curves_data, get_power_curves_data # For standalone testing if needed
    from core import Vehicle # For type hinting
except ImportError:
    print("Attempting to import components for plotting.py")
    # Add path adjustments if necessary for direct execution
    # import sys
    # import os
    # sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    # from outputs import get_torque_curves_data, get_power_curves_data
    # from core import Vehicle


def plot_speed_vs_time(simulation_df: pd.DataFrame, title: str = "Speed vs. Time"):
    """
    Plots vehicle speed (km/h) against time (s).

    Args:
        simulation_df: DataFrame from `simulate_acceleration` or `estimate_top_speed`
                       (must contain 'Time_s' and 'Speed_kmh' columns).
        title: The title for the plot.
    """
    if not all(col in simulation_df.columns for col in ['Time_s', 'Speed_kmh']):
        print("Error: simulation_df must contain 'Time_s' and 'Speed_kmh' columns.")
        return

    plt.figure(figsize=(10, 6))
    plt.plot(simulation_df['Time_s'], simulation_df['Speed_kmh'], label="Speed")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (km/h)")
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.show()


def plot_torque_rpm_curves(
    vehicle_obj: Vehicle, # Pass the vehicle object to generate fresh data
    rpm_points_for_curve: Optional[np.ndarray] = None,
    simulation_df: Optional[pd.DataFrame] = None,
    title: str = "Torque vs. RPM"
):
    """
    Plots engine, motor, and combined torque curves against RPM.
    Optionally overlays actual operating points and gear shifts from a simulation run.

    Args:
        vehicle_obj: The vehicle object to generate torque curve data.
        rpm_points_for_curve: Optional np.array of RPMs for the theoretical curves.
        simulation_df: Optional DataFrame from simulation (must contain 'Engine_RPM',
                       'Motor_RPM', 'Gear', 'Tractive_Force_N', 'Time_s', 'Speed_kmh').
                       Used to overlay operating points and shift points.
        title: The title for the plot.
    """
    torque_curves_df = get_torque_curves_data(vehicle_obj, rpm_points=rpm_points_for_curve)

    plt.figure(figsize=(12, 7))

    if vehicle_obj.engine:
        plt.plot(torque_curves_df['RPM'], torque_curves_df['EngineTorque_Nm'], label="Engine Torque (potential)", linestyle='--')
    if vehicle_obj.motor:
        plt.plot(torque_curves_df['RPM'], torque_curves_df['MotorTorque_Nm'], label="Motor Torque (potential)", linestyle='--')

    # Plot combined potential torque
    plt.plot(torque_curves_df['RPM'], torque_curves_df['CombinedTorque_Nm'], label="Combined Torque (potential)", color='black', linewidth=2)

    if simulation_df is not None and not simulation_df.empty:
        # Overlay actual operating points (engine RPM vs combined torque *at the engine/motor*)
        # This requires calculating the combined torque at source from simulation data
        # The simulation_df has tractive force, need to work backwards or use source RPMs and get_combined_torque again
        # For simplicity, let's plot engine RPM vs its torque, and motor RPM vs its torque from the sim.
        # And show gear shifts.

        if vehicle_obj.engine and 'Engine_RPM' in simulation_df.columns and 'Tractive_Force_N' in simulation_df.columns:
            # This is tricky: simulation_df has Engine_RPM. We need the engine's contribution to combined torque during sim.
            # Let's plot the Engine_RPM from simulation and the torque it was producing at that time.
            # This requires storing individual torques in simulation_df or re-calculating.
            # For now, let's focus on gear shifts.
            pass # Actual operating torque points can be added if sim_df stores individual torques

        # Overlay gear shift points
        # A shift occurs when simulation_df['Gear'] changes
        if 'Gear' in simulation_df.columns and 'Engine_RPM' in simulation_df.columns and vehicle_obj.engine:
            shift_points_rpm = []
            shift_points_torque = [] # We'd need the torque at shift point

            gears = simulation_df['Gear'].to_numpy()
            rpms = simulation_df['Engine_RPM'].to_numpy() # Assuming shifts based on engine RPM

            for i in range(1, len(simulation_df)):
                if gears[i] != gears[i-1] and gears[i-1] !=0 : # Gear changed from a valid gear
                    shift_rpm = rpms[i-1] # RPM just before shift
                    shift_points_rpm.append(shift_rpm)
                    # To get torque at this point, we'd need to know combined input torque to transmission
                    # For now, just mark RPM. We can get the torque from the *potential* curve at that RPM.
                    # This is an approximation of where on the potential curve the shift happened.
                    shift_torque = np.interp(shift_rpm, torque_curves_df['RPM'], torque_curves_df['CombinedTorque_Nm'])
                    shift_points_torque.append(shift_torque)

            if shift_points_rpm:
                plt.scatter(shift_points_rpm, shift_points_torque, color='red', s=100, zorder=5, label="Gear Shifts (approx. on combined curve)")

    plt.xlabel("RPM")
    plt.ylabel("Torque (Nm)")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.ylim(bottom=0) # Torque is usually positive for propulsion
    if vehicle_obj.engine or vehicle_obj.motor: # Set xlim based on max redline
        max_rpm_plot = 0
        if vehicle_obj.engine: max_rpm_plot = max(max_rpm_plot, vehicle_obj.engine.redline_rpm)
        if vehicle_obj.motor: max_rpm_plot = max(max_rpm_plot, vehicle_obj.motor.redline_rpm)
        if torque_curves_df['RPM'].max() > max_rpm_plot : max_rpm_plot = torque_curves_df['RPM'].max()
        plt.xlim(left=0, right=max_rpm_plot * 1.05)

    plt.show()


def plot_power_rpm_curves(
    vehicle_obj: Vehicle, # Pass the vehicle object
    rpm_points_for_curve: Optional[np.ndarray] = None,
    simulation_df: Optional[pd.DataFrame] = None, # For gear shifts
    title: str = "Power vs. RPM"
):
    """
    Plots engine, motor, and combined power curves against RPM.
    Optionally overlays gear shifts from a simulation run.

    Args:
        vehicle_obj: The vehicle object to generate power curve data.
        rpm_points_for_curve: Optional np.array of RPMs for the theoretical curves.
        simulation_df: Optional DataFrame from simulation for gear shift points.
        title: The title for the plot.
    """
    power_curves_df = get_power_curves_data(vehicle_obj, rpm_points=rpm_points_for_curve)

    plt.figure(figsize=(12, 7))

    if vehicle_obj.engine:
        plt.plot(power_curves_df['RPM'], power_curves_df['EnginePower_kW'], label="Engine Power (potential)", linestyle='--')
    if vehicle_obj.motor:
        plt.plot(power_curves_df['RPM'], power_curves_df['MotorPower_kW'], label="Motor Power (potential)", linestyle='--')

    plt.plot(power_curves_df['RPM'], power_curves_df['CombinedPower_kW'], label="Combined Power (potential)", color='black', linewidth=2)

    # Overlay gear shift points (similar to torque plot)
    if simulation_df is not None and not simulation_df.empty and \
       'Gear' in simulation_df.columns and 'Engine_RPM' in simulation_df.columns and vehicle_obj.engine:
        shift_points_rpm = []
        shift_points_power = []

        gears = simulation_df['Gear'].to_numpy()
        rpms = simulation_df['Engine_RPM'].to_numpy()

        for i in range(1, len(simulation_df)):
            if gears[i] != gears[i-1] and gears[i-1] !=0:
                shift_rpm = rpms[i-1]
                shift_points_rpm.append(shift_rpm)
                shift_power = np.interp(shift_rpm, power_curves_df['RPM'], power_curves_df['CombinedPower_kW'])
                shift_points_power.append(shift_power)

        if shift_points_rpm:
            plt.scatter(shift_points_rpm, shift_points_power, color='red', s=100, zorder=5, label="Gear Shifts (approx. on combined curve)")

    plt.xlabel("RPM")
    plt.ylabel("Power (kW)")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.ylim(bottom=0)
    if vehicle_obj.engine or vehicle_obj.motor: # Set xlim based on max redline
        max_rpm_plot = 0
        if vehicle_obj.engine: max_rpm_plot = max(max_rpm_plot, vehicle_obj.engine.redline_rpm)
        if vehicle_obj.motor: max_rpm_plot = max(max_rpm_plot, vehicle_obj.motor.redline_rpm)
        if power_curves_df['RPM'].max() > max_rpm_plot : max_rpm_plot = power_curves_df['RPM'].max()
        plt.xlim(left=0, right=max_rpm_plot*1.05)

    plt.show()


def plot_acceleration_map(
    accel_map_df: pd.DataFrame, # From get_gear_dependent_acceleration_map_data
    title: str = "Gear-Dependent Acceleration Map"
):
    """
    Plots maximum acceleration (m/s^2) vs. speed (km/h) for each gear.

    Args:
        accel_map_df: DataFrame from `get_gear_dependent_acceleration_map_data`.
                      Expected columns: 'Speed_kmh', 'Gear_1_Accel_mps2', ...
        title: The title for the plot.
    """
    if 'Speed_kmh' not in accel_map_df.columns:
        print("Error: accel_map_df must contain 'Speed_kmh' column.")
        return

    plt.figure(figsize=(12, 7))

    for col in accel_map_df.columns:
        if col.startswith('Gear_') and col.endswith('_Accel_mps2'):
            gear_label = col.replace('_Accel_mps2', '').replace('_', ' ')
            plt.plot(accel_map_df['Speed_kmh'], accel_map_df[col], label=gear_label)

    plt.xlabel("Speed (km/h)")
    plt.ylabel("Maximum Acceleration (m/s^2)")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.ylim(bottom=0)
    plt.show()

# Example of how to use (for testing, typically called from main script)
if __name__ == '__main__':
    # This section is for testing the plotting functions with dummy data.
    # In actual use, these functions will be called from the main script with real simulation data.
    print("Testing plotting functions with dummy data...")

    # Dummy simulation_df for speed vs time
    dummy_sim_data = pd.DataFrame({
        'Time_s': np.linspace(0, 10, 100),
        'Speed_kmh': np.linspace(0, 100, 100) * (1 - np.exp(-0.3 * np.linspace(0, 10, 100))),
        'Gear': np.concatenate([np.ones(30), np.ones(30)*2, np.ones(40)*3]), # Dummy gears
        'Engine_RPM': np.linspace(800, 6000, 100) # Dummy RPM
    })
    plot_speed_vs_time(dummy_sim_data)

    # Dummy Vehicle object and data for torque/power plots
    class DummyEngine:
        def __init__(self):
            self.redline_rpm = 6500
            # simple linear torque curve for testing
            self.torque_curve = pd.Series([100, 150, 140], index=[1000, 3000, 6000])
        def get_torque(self, rpm): # Simplified
            return np.interp(rpm, self.torque_curve.index, self.torque_curve.values, left=0, right=0)

    class DummyMotor:
        def __init__(self):
            self.redline_rpm = 8000
            self.max_power_kw = 50
            self.torque_curve = pd.Series([80, 80, 60], index=[0, 4000, 7500])
        def get_torque(self, rpm): # Simplified
            base_tq = np.interp(rpm, self.torque_curve.index, self.torque_curve.values, left=0, right=0)
            if rpm > 0:
                power_limit_tq = (self.max_power_kw * 9549.3) / rpm
                return min(base_tq, power_limit_tq)
            return base_tq

    class DummyBattery:
        def __init__(self):
            self.max_discharge_power_kw = 40 # Lower than motor's max

    class DummyTransmission:
         def __init__(self):
            self.gear_ratios = [3.0, 2.0, 1.0] # Dummy
            self.final_gear_ratio = 3.5


    # Need a full Vehicle object for get_torque_curves_data and get_power_curves_data
    # For simplicity, we'll assume get_torque_curves_data and get_power_curves_data are available
    # and can be tested separately if outputs.py is also present.
    # Here, we'll manually create dummy data for the plots if Vehicle cannot be instantiated.

    # Dummy torque_curves_df
    rpm_range = np.linspace(0, 7000, 100)
    dummy_torque_data = pd.DataFrame({
        'RPM': rpm_range,
        'EngineTorque_Nm': 150 * np.sin(rpm_range/7000 * np.pi/2) + 20,
        'MotorTorque_Nm': 80 * (1 - rpm_range/8000) + 10,
        'CombinedTorque_Nm': (150 * np.sin(rpm_range/7000 * np.pi/2) + 20) + (80 * (1 - rpm_range/8000) + 10)
    })
    # This manual creation is not ideal. Better to instantiate a dummy Vehicle.
    # Let's try to make a dummy vehicle instance for testing
    try:
        test_engine = DummyEngine()
        test_motor = DummyMotor()
        test_battery = DummyBattery()
        test_transmission = DummyTransmission()

        # A simplified dummy vehicle for plotting.
        # The actual Vehicle class has more params, but for plotting we mainly need engine/motor/battery.
        class TestVehicleForPlotting:
            def __init__(self):
                self.engine = test_engine
                self.motor = test_motor
                self.battery = test_battery
                # The get_torque_curves_data in outputs.py needs a full Vehicle, so this is still a compromise
                # This __main__ block is primarily for quick visual checks of plot functions.

        dummy_vehicle_for_plot = TestVehicleForPlotting()

        # We'd call plot_torque_rpm_curves(dummy_vehicle_for_plot, simulation_df=dummy_sim_data)
        # but get_torque_curves_data expects a full Vehicle.
        # So, we'll plot with manually created dummy_torque_data for now.

        # Re-defining plot functions locally to use dummy_torque_data for this test block
        # This is NOT how it should be in production.

        print("Visualizing torque curves with dummy data (actual functions use Vehicle object).")
        plt.figure(figsize=(12,7))
        plt.plot(dummy_torque_data['RPM'], dummy_torque_data['EngineTorque_Nm'], label="Engine Torque (dummy)", linestyle='--')
        plt.plot(dummy_torque_data['RPM'], dummy_torque_data['MotorTorque_Nm'], label="Motor Torque (dummy)", linestyle='--')
        plt.plot(dummy_torque_data['RPM'], dummy_torque_data['CombinedTorque_Nm'], label="Combined Torque (dummy)", color='black')
        # Add shift points from dummy_sim_data to this dummy plot
        if 'Gear' in dummy_sim_data.columns and 'Engine_RPM' in dummy_sim_data.columns:
            shift_points_rpm = []
            shift_points_torque = []
            gears = dummy_sim_data['Gear'].to_numpy()
            rpms = dummy_sim_data['Engine_RPM'].to_numpy()
            for i in range(1, len(dummy_sim_data)):
                if gears[i] != gears[i-1]:
                    shift_rpm = rpms[i-1]
                    shift_points_rpm.append(shift_rpm)
                    shift_torque_val = np.interp(shift_rpm, dummy_torque_data['RPM'], dummy_torque_data['CombinedTorque_Nm'])
                    shift_points_torque.append(shift_torque_val)
            if shift_points_rpm:
                plt.scatter(shift_points_rpm, shift_points_torque, color='red', s=100, zorder=5, label="Gear Shifts (dummy)")
        plt.xlabel("RPM"); plt.ylabel("Torque (Nm)"); plt.title("Torque vs. RPM (Dummy Data Test)"); plt.legend(); plt.grid(True); plt.show()

        # Dummy power_curves_df
        dummy_power_data = pd.DataFrame({
            'RPM': rpm_range,
            'EnginePower_kW': (dummy_torque_data['EngineTorque_Nm'] * rpm_range / 9549.3).fillna(0),
            'MotorPower_kW': (dummy_torque_data['MotorTorque_Nm'] * rpm_range / 9549.3).fillna(0),
            'CombinedPower_kW': (dummy_torque_data['CombinedTorque_Nm'] * rpm_range / 9549.3).fillna(0)
        })
        print("Visualizing power curves with dummy data.")
        plt.figure(figsize=(12,7))
        plt.plot(dummy_power_data['RPM'], dummy_power_data['EnginePower_kW'], label="Engine Power (dummy)", linestyle='--')
        plt.plot(dummy_power_data['RPM'], dummy_power_data['MotorPower_kW'], label="Motor Power (dummy)", linestyle='--')
        plt.plot(dummy_power_data['RPM'], dummy_power_data['CombinedPower_kW'], label="Combined Power (dummy)", color='black')
        plt.xlabel("RPM"); plt.ylabel("Power (kW)"); plt.title("Power vs. RPM (Dummy Data Test)"); plt.legend(); plt.grid(True); plt.show()


    except NameError as e:
        print(f"Skipping some dummy data plots due to missing dependencies for dummy objects: {e}")
    except ImportError:
        print(f"Skipping some dummy data plots due to ImportError for core/outputs for dummy objects.")


    # Dummy accel_map_df
    dummy_accel_data = pd.DataFrame({
        'Speed_kmh': np.linspace(0, 150, 15),
        'Gear_1_Accel_mps2': 5 * np.exp(-0.02 * np.linspace(0, 150, 15)),
        'Gear_2_Accel_mps2': 3 * np.exp(-0.015 * np.linspace(0, 150, 15)),
        'Gear_3_Accel_mps2': 1.5 * np.exp(-0.01 * np.linspace(0, 150, 15))
    })
    plot_acceleration_map(dummy_accel_data)

    print("Plotting tests complete. Close plot windows to continue.")
