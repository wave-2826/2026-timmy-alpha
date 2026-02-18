# Load `fast.csv` and `slow.csv`

import math
import os
import pandas as pd
from enum import Enum

analysis_data = {}

class MotorType(Enum):
    FLYWHEEL_TOP = "flywheel_top"
    FLYWHEEL_BOTTOM = "flywheel_bottom"
    AZIMUTH = "azimuth"
    HOOD = "hood"
class Ring(Enum):
    TOP = "top", [MotorType.AZIMUTH]
    MIDDLE = "middle", [MotorType.HOOD]
    BOTTOM = "bottom", [MotorType.FLYWHEEL_TOP, MotorType.FLYWHEEL_BOTTOM]
    def __init__(self, ring_name, motors):
        self.ring_name = ring_name
        self.motors = motors

motor_identification_columns = {
    MotorType.FLYWHEEL_TOP: "Brownout Sticky Warning",
    MotorType.FLYWHEEL_BOTTOM: "Brownout Warning",
    MotorType.AZIMUTH: "Ext EEPROM Sticky Warning",
    MotorType.HOOD: "Ext EEPROM Warning",
}

def column_name_to_motor_name(column_name, uuid_map):
    for uuid, motor_name in uuid_map.items():
        if uuid in column_name:
            return motor_name
    return None

def filter_dataframe(df, rings) -> (pd.DataFrame, dict):
    # The exported data doesn't identify which motor is which, so each has a different warning enabled that only it has.
    # Columns are in the format "SparkMax/////2a3d907b-9f5b-492b-a206-44f9987c907f Current",
    # where the UUID is the same for all columns of a given motor, so we can identify the motor by looking for the warning
    # column and then filtering to only columns with the same UUID.

    uuids = {}
    for motor in motor_identification_columns.keys():
        warning_column = motor_identification_columns[motor]
        matching_columns = [col for col in df.columns if warning_column in col]
        if not matching_columns:
            print(f"Warning column '{warning_column}' not found for motor '{motor.value}'")
            continue
        if len(matching_columns) > 1:
            print(f"Multiple columns found for warning '{warning_column}' of motor '{motor.value}': {matching_columns}")
            continue
        uuid = matching_columns[0].split("/////")[1].split()[0]
        uuids[uuid] = motor.value
    
    motors = []
    for ring in rings:
        motors.extend(ring.motors)
    
    matching_uuids = [uuid for uuid, motor_name in uuids.items() if MotorType(motor_name) in motors]

    filtered_columns = [col for col in df.columns if any(uuid in col for uuid in matching_uuids)]
    # Remove warning columns
    filtered_columns = [col for col in filtered_columns if not any(warning in col for warning in motor_identification_columns.values())]
    return (df[filtered_columns], uuids)

def analyze_motor(test_name, *rings):
    print(f"Analysis for {test_name}:")

    data_file = test_name
    
    try:
        full_fast_df = pd.read_csv(os.path.join(os.path.dirname(__file__), 'data', f'{data_file}_fast.csv'))
        full_slow_df = pd.read_csv(os.path.join(os.path.dirname(__file__), 'data', f'{data_file}_slow.csv'))
    except FileNotFoundError:
        print(f"  CSV files for {data_file} not found.")
        print(f"  ")

        analysis_data[test_name] = {
            "static_loss": 0.0,
            "dynamic_loss": 0.0,
        }
        return
    
    (fast_df, uuid_map_fast) = filter_dataframe(full_fast_df, rings)
    (slow_df, uuid_map_slow) = filter_dataframe(full_slow_df, rings)

    # Average all nonempty cells of each column whose name contains "Current"
    def average_current(df, uuid_map, name):
        current_columns = [col for col in df.columns if 'Current' in col]
        # Average by column and sum
        total_current = 0
        for col in current_columns:
            motor_name = column_name_to_motor_name(col, uuid_map)
            if motor_name is None:
                print(f"Could not identify motor for column '{col}'")
                continue
            total_current += df[col].mean()
        # Print a summary
        if len(current_columns) > 1:
            print(f'  Average current for {name} by motor:')
            for uuid, motor_name in uuid_map.items():
                if MotorType(motor_name) not in [motor for ring in rings for motor in ring.motors]:
                    continue
                motor_columns = [col for col in current_columns if uuid in col]
                if not motor_columns:
                    print(f"    No current columns found for motor '{motor_name}' with UUID '{uuid}'")
                    continue
                motor_current = sum(df[col].mean() for col in motor_columns)
                print(f'    {motor_name}: {motor_current:.4f} Amps')
        return total_current

    fast_avg_current = average_current(full_fast_df, uuid_map_fast, "fast")
    slow_avg_current = average_current(full_slow_df, uuid_map_slow, "slow")

    print(f'  Average current for fast: {fast_avg_current:.4f} Amps')
    print(f'  Average current for slow: {slow_avg_current:.4f} Amps')
    print(f'  Total dynamic loss: {fast_avg_current - slow_avg_current:.4f} Amps')

    # Average the absolute value of the velocity for nonempty cells of velocity columns
    def average_velocity(df):
        velocity_columns = [col for col in df.columns if 'Velocity' in col]
        velocity_values = df[velocity_columns].values.flatten()
        velocity_values = velocity_values[~pd.isnull(velocity_values)]
        return abs(velocity_values).mean()

    fast_avg_velocity = average_velocity(fast_df)
    slow_avg_velocity = average_velocity(slow_df)

    print(f'  ')
    print(f'  Average velocity for fast: {fast_avg_velocity:.4f} rad/s')
    print(f'  Average velocity for slow: {slow_avg_velocity:.4f} rad/s')

    print(f'  ')
    print(f"  Static losses: {slow_avg_current:.4f} Amps")
    currentDiff = fast_avg_current - slow_avg_current
    velDiff = fast_avg_velocity - slow_avg_velocity
    print(f"  Dynamic losses: {currentDiff / velDiff:.4f} Amps/(rad/s)")
    print(f"    or {(currentDiff * 1000) / (velDiff / 2 / math.pi * 60):.4f} mA/rpm")

    print(f"  ")

    analysis_data[test_name] = {
        "static_loss": slow_avg_current,
        "current_fast": fast_avg_current,
        "dynamic_loss": currentDiff / velDiff,
        "per_motor_current": {column_name_to_motor_name(col, uuid_map_fast): full_fast_df[col].mean() for col in full_fast_df.columns if 'Current' in col},
    }


analyze_motor("flywheel", Ring.BOTTOM)
analyze_motor("azimuth", Ring.TOP)
analyze_motor("hood", Ring.MIDDLE)

analyze_motor("all_rings", Ring.TOP, Ring.MIDDLE, Ring.BOTTOM)
analyze_motor("bottom_static", Ring.TOP, Ring.MIDDLE)
analyze_motor("top_static", Ring.MIDDLE, Ring.BOTTOM)

# Generate a table of the results
print("Motor summary:")
print(f"  {'Test':<25} {'Static Loss (A)':<20} {'Dynamic Loss (A/(rad/s))':<30}")
for motor, data in analysis_data.items():
    print(f"  {motor:<25} {data['static_loss']:<20.4f} {data['dynamic_loss']:<30.6f} {data.get('comment', '')}")
print("  ")


import numpy as np

flywheel_static = analysis_data["flywheel"]["static_loss"]
azimuth_static = analysis_data["azimuth"]["static_loss"]
hood_static = analysis_data["hood"]["static_loss"]
all_rings_static = analysis_data["all_rings"]["static_loss"]
top_two_static = analysis_data["bottom_static"]["static_loss"]
bottom_two_static = analysis_data["top_static"]["static_loss"]



# Variables: [metal_bearing_loss, plastic_bearing_loss, internal_flywheel_loss, internal_hood_loss, external_azimuth_loss]
A = np.array([
    [2, 0, 0, 0, 1],  # all_rings_static = 2*metal + external_azimuth
    [1, 1, 1, 0, 1],  # top_two_static = metal + plastic + internal_flywheel + external_azimuth
    [1, 1, 1, 1, 0],  # bottom_two_static = metal + plastic + internal_flywheel + internal_hood
    # [1, 1, 1, 0, 0],  # flywheel_static = metal + plastic + internal_flywheel
    [1, 1, 1, 1, 1],  # azimuth_static = metal + plastic + internal_flywheel + internal_hood + external_azimuth
    [0, 2, 0, 1, 0],  # hood_static = 2*plastic + internal_hood
])
b = np.array([
    all_rings_static,
    top_two_static,
    bottom_two_static,
    # flywheel_static,
    azimuth_static,
    hood_static,
])

solution = np.linalg.solve(A, b)

print(solution)

print("Estimated losses:")
print(f"  Metal bearing loss: {solution[0]:.4f} Amps")
print(f"  Plastic bearing loss: {solution[1]:.4f} Amps")
print(f"  Internal flywheel loss: {solution[2]:.4f} Amps")
print(f"  Internal hood loss: {solution[3]:.4f} Amps")
print(f"  External azimuth loss: {solution[4]:.4f} Amps")
print("  ")

generated_file_path = os.path.join(os.path.dirname(__file__), "..", "..", "src/main/java/frc/robot/generated", "TurretTuningData.java")
with open(generated_file_path, "w") as f:
    f.write("""package frc.robot.generated;

/**
 * Generated by `just tune_turret` - edits will be overwritten!  
 * Contains data tuned by `misc/turretTuning/analyze_data.py`.
 */
public class TurretTuningData {
""")
    for motor, data in analysis_data.items():
        motor_name_camel_case = ''.join(word.capitalize() for word in motor.split('_'))
        motor_name_camel_case = motor_name_camel_case[0].lower() + motor_name_camel_case[1:]
        f.write(f"    public static final double {motor_name_camel_case}StaticLoss = {data['static_loss']:.6f};\n")
        f.write(f"    public static final double {motor_name_camel_case}DynamicLoss = {data['dynamic_loss']:.6f};\n")
    f.write("}\n")
print("Results saved to code.")

# Export the saved data to a csv so we can load it into a spreadsheet
export_file_path = os.path.join(os.path.dirname(__file__), "tuning_results.csv")
with open(export_file_path, "w") as f:
    motor_columns = ''.join([f"Motor {ty.value} Current (A)," for ty in MotorType])
    f.write(f"Test,Static Loss (A),Loss when fast (A),Dynamic Loss (A/(rad/s)),{motor_columns}\n")
    for motor, data in analysis_data.items():
        f.write(f"{motor},{data['static_loss']:.6f},{data['current_fast']:.6f},{data['dynamic_loss']:.6f},")
        for ty in MotorType:
            f.write(f"{data['per_motor_current'].get(ty.value, 0.0):.6f},")
        f.write("\n")