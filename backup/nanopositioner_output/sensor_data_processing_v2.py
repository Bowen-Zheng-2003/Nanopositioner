import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# ============================================================================
# CONFIGURATION
# ============================================================================
# Set this to choose which extreme point to use as the turning point:
# - "first": Always use the first occurring extreme (min or max)
# - "algorithm": Use the intelligent selection algorithm to choose between first and second
TURNING_POINT_METHOD = "algorithm"  # Options: "first" or "algorithm"
# ============================================================================

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

version = 18
graphs_dir = os.path.join(script_dir, 'graphs')

# Build the full path to the CSV files
keyence_file_path = os.path.join(script_dir, f"keyence_output{version}.csv")
arduino_file_path = os.path.join(script_dir, f"arduino_output{version}.txt")

# Import Arduino TXT file first to get the time reference
df_arduino = pd.read_csv(arduino_file_path, header=None, delimiter=', ', 
                        names=['time_ms', 'displacement_mm_raw', 'displacement_mm_filtered'], 
                        engine='python')

# Strip whitespace from all columns
df_arduino['time_ms'] = df_arduino['time_ms'].astype(str).str.strip()
df_arduino['displacement_mm_raw'] = df_arduino['displacement_mm_raw'].astype(str).str.strip()
df_arduino['displacement_mm_filtered'] = df_arduino['displacement_mm_filtered'].astype(str).str.strip()

# Convert to numeric, treating '-' as NaN
df_arduino['time_ms'] = pd.to_numeric(df_arduino['time_ms'], errors='coerce')
df_arduino['displacement_mm_raw'] = pd.to_numeric(df_arduino['displacement_mm_raw'], errors='coerce') / 1000
df_arduino['displacement_mm_filtered'] = pd.to_numeric(df_arduino['displacement_mm_filtered'], errors='coerce') / 1000

# Remove rows where either displacement column is NaN
df_arduino = df_arduino.dropna(subset=['displacement_mm_raw', 'displacement_mm_filtered'])

# Convert bits to mm (with scaling) for both columns
df_arduino['displacement_mm_raw'] = df_arduino['displacement_mm_raw'] * 0.48828125 * 0.9335003416
df_arduino['displacement_mm_filtered'] = df_arduino['displacement_mm_filtered'] * 0.48828125 * 0.9335003416

# Create time_s column by calculating cumulative time differences in seconds
df_arduino['time_s'] = (df_arduino['time_ms'] - df_arduino['time_ms'].iloc[0]) / 1000.0

# Import Keyence CSV file
df_keyence = pd.read_csv(keyence_file_path, header=None)

# Find the column with the most non-null values
column_lengths = df_keyence.count()
longest_column_index = column_lengths.idxmax()

# Create a new DataFrame with only the longest column
df_keyence = pd.DataFrame({
    'displacement_mm': df_keyence[longest_column_index]
})

# Remove any NaN values
df_keyence = df_keyence.dropna()

# Use Arduino time values for Keyence (align datasets by length)
min_length = min(len(df_keyence), len(df_arduino))
df_keyence = df_keyence.iloc[:min_length].copy()
df_arduino = df_arduino.iloc[:min_length].copy()

# Reset indices to ensure alignment
df_keyence.reset_index(drop=True, inplace=True)
df_arduino.reset_index(drop=True, inplace=True)

# Assign Arduino time values to Keyence
df_keyence.insert(0, 'time_s', df_arduino['time_s'].values)

print(f"Aligned datasets to {min_length} data points")
print(f"Time range: {df_keyence['time_s'].iloc[0]:.4f} to {df_keyence['time_s'].iloc[-1]:.4f} seconds")

# Find global max and min for Keyence
keyence_max_idx = df_keyence['displacement_mm'].idxmax()
keyence_min_idx = df_keyence['displacement_mm'].idxmin()

print(f"\nKeyence - Global Maximum:")
print(f"Index: {keyence_max_idx}")
print(f"Time: {df_keyence.loc[keyence_max_idx, 'time_s']:.4f} s")
print(f"Displacement: {df_keyence.loc[keyence_max_idx, 'displacement_mm']:.4f} mm")

print(f"\nKeyence - Global Minimum:")
print(f"Index: {keyence_min_idx}")
print(f"Time: {df_keyence.loc[keyence_min_idx, 'time_s']:.4f} s")
print(f"Displacement: {df_keyence.loc[keyence_min_idx, 'displacement_mm']:.4f} mm")

# Determine which occurs first and second (max or min) for Keyence
keyence_first_extreme_idx = min(keyence_max_idx, keyence_min_idx)
keyence_second_extreme_idx = max(keyence_max_idx, keyence_min_idx)
keyence_first_extreme_type = "Max" if keyence_first_extreme_idx == keyence_max_idx else "Min"
keyence_second_extreme_type = "Max" if keyence_second_extreme_idx == keyence_max_idx else "Min"

print(f"\nKeyence - First extreme point is {keyence_first_extreme_type} at index {keyence_first_extreme_idx}")
print(f"Keyence - Second extreme point is {keyence_second_extreme_type} at index {keyence_second_extreme_idx}")

# Function to determine which extreme is the true turning point
def select_turning_point(df, first_idx, second_idx, first_type, column_name='displacement_mm', method="algorithm"):
    """
    Determines which extreme point (first or second) represents the turning point.
    
    Parameters:
    -----------
    df : DataFrame
        The dataframe containing the sensor data
    first_idx : int
        Index of the first extreme point
    second_idx : int
        Index of the second extreme point
    first_type : str
        Type of first extreme ("Max" or "Min")
    column_name : str
        Name of the column to analyze
    method : str
        Method to use: "first" or "algorithm"
        - "first": Always return the first extreme
        - "algorithm": Use intelligent selection based on data analysis
    
    Returns:
    --------
    tuple: (selected_index, selected_type)
    """
    if method == "first":
        print(f"\n  Using FIRST extreme point (method='first')")
        return first_idx, first_type
    
    # Algorithm method: analyze which extreme is the true turning point
    print(f"\n  Using ALGORITHM to select turning point")
    
    data_after_first = len(df) - first_idx
    data_after_second = len(df) - second_idx
    
    # Calculate the slope/trend after each extreme (looking at next 10% of remaining data)
    window_first = max(10, int(0.1 * data_after_first))
    window_second = max(10, int(0.1 * data_after_second))
    
    # Get data windows after each extreme
    first_window = df.loc[first_idx:min(first_idx + window_first, len(df)-1), column_name]
    second_window = df.loc[second_idx:min(second_idx + window_second, len(df)-1), column_name]
    
    # Calculate if there's a consistent trend after each point
    first_has_trend = len(first_window) > 1 and abs(first_window.iloc[-1] - first_window.iloc[0]) > 0.1
    second_has_trend = len(second_window) > 1 and abs(second_window.iloc[-1] - second_window.iloc[0]) > 0.1
    
    print(f"  Data points after first extreme: {data_after_first}")
    print(f"  Data points after second extreme: {data_after_second}")
    print(f"  First extreme has trend: {first_has_trend}")
    print(f"  Second extreme has trend: {second_has_trend}")
    
    # Decision logic:
    # 1. If second extreme has very little data after it (<5% of total), use first
    # 2. If second extreme has significant data and shows a trend, use second
    # 3. Otherwise, use the one with more data after it
    
    total_points = len(df)
    
    if data_after_second < 0.05 * total_points:
        print(f"  -> Selected FIRST extreme (second has insufficient data)")
        return first_idx, first_type
    elif data_after_second > 0.2 * total_points and second_has_trend:
        print(f"  -> Selected SECOND extreme (has significant data and trend)")
        return second_idx, "Max" if first_type == "Min" else "Min"
    elif data_after_first > data_after_second:
        print(f"  -> Selected FIRST extreme (has more data)")
        return first_idx, first_type
    else:
        print(f"  -> Selected SECOND extreme (has more data)")
        return second_idx, "Max" if first_type == "Min" else "Min"

# Select the turning point for Keyence
print(f"\n{'='*60}")
print(f"TURNING POINT METHOD: {TURNING_POINT_METHOD.upper()}")
print(f"{'='*60}")
print("\n=== Keyence Turning Point Selection ===")
keyence_turning_idx, keyence_turning_type = select_turning_point(
    df_keyence, keyence_first_extreme_idx, keyence_second_extreme_idx, 
    keyence_first_extreme_type, column_name='displacement_mm', method=TURNING_POINT_METHOD
)

# Create dataframe starting from turning point
df_keyence_after_extreme = df_keyence.loc[keyence_turning_idx:].copy()
df_keyence_after_extreme.reset_index(drop=True, inplace=True)

# Shift time to start at 0
df_keyence_after_extreme['time_s'] = df_keyence_after_extreme['time_s'] - df_keyence_after_extreme['time_s'].iloc[0]

# Shift displacement to start at 0
df_keyence_after_extreme['displacement_mm'] = df_keyence_after_extreme['displacement_mm'] - df_keyence_after_extreme['displacement_mm'].iloc[0]

print(f"Keyence DataFrame after turning point:")
print(f"Number of rows: {len(df_keyence_after_extreme)}")

# Find global max and min for Arduino (using filtered data)
arduino_max_idx = df_arduino['displacement_mm_filtered'].idxmax()
arduino_min_idx = df_arduino['displacement_mm_filtered'].idxmin()

print(f"\n\nArduino - Global Maximum:")
print(f"Index: {arduino_max_idx}")
print(f"Time: {df_arduino.loc[arduino_max_idx, 'time_s']:.4f} s")
print(f"Displacement (filtered): {df_arduino.loc[arduino_max_idx, 'displacement_mm_filtered']:.4f} mm")

print(f"\nArduino - Global Minimum:")
print(f"Index: {arduino_min_idx}")
print(f"Time: {df_arduino.loc[arduino_min_idx, 'time_s']:.4f} s")
print(f"Displacement (filtered): {df_arduino.loc[arduino_min_idx, 'displacement_mm_filtered']:.4f} mm")

# Determine which occurs first and second (max or min) for Arduino
arduino_first_extreme_idx = min(arduino_max_idx, arduino_min_idx)
arduino_second_extreme_idx = max(arduino_max_idx, arduino_min_idx)
arduino_first_extreme_type = "Max" if arduino_first_extreme_idx == arduino_max_idx else "Min"
arduino_second_extreme_type = "Max" if arduino_second_extreme_idx == arduino_max_idx else "Min"

print(f"\nArduino - First extreme point is {arduino_first_extreme_type} at index {arduino_first_extreme_idx}")
print(f"Arduino - Second extreme point is {arduino_second_extreme_type} at index {arduino_second_extreme_idx}")

# Select the turning point for Arduino
print("\n=== Arduino Turning Point Selection ===")
arduino_turning_idx, arduino_turning_type = select_turning_point(
    df_arduino, arduino_first_extreme_idx, arduino_second_extreme_idx, 
    arduino_first_extreme_type, column_name='displacement_mm_filtered', method=TURNING_POINT_METHOD
)

# Create dataframe starting from turning point
df_arduino_after_extreme = df_arduino.loc[arduino_turning_idx:].copy()
df_arduino_after_extreme.reset_index(drop=True, inplace=True)

# Shift time to start at 0
df_arduino_after_extreme['time_s'] = df_arduino_after_extreme['time_s'] - df_arduino_after_extreme['time_s'].iloc[0]

# Shift displacement to start at 0 for both raw and filtered
df_arduino_after_extreme['displacement_mm_raw'] = df_arduino_after_extreme['displacement_mm_raw'] - df_arduino_after_extreme['displacement_mm_raw'].iloc[0]
df_arduino_after_extreme['displacement_mm_filtered'] = df_arduino_after_extreme['displacement_mm_filtered'] - df_arduino_after_extreme['displacement_mm_filtered'].iloc[0]

# Rename columns to distinguish between sensors
df_keyence_after_extreme_renamed = df_keyence_after_extreme.rename(columns={
    'time_s': 'keyence_time_s',
    'displacement_mm': 'keyence_displacement_mm'
})

df_arduino_after_extreme_renamed = df_arduino_after_extreme.rename(columns={
    'time_s': 'arduino_time_s',
    'displacement_mm_raw': 'arduino_displacement_mm_raw',
    'displacement_mm_filtered': 'arduino_displacement_mm_filtered'
})

# Combine both dataframes side by side
df_combined = pd.concat([df_keyence_after_extreme_renamed, df_arduino_after_extreme_renamed], axis=1)

# Export to CSV with method indicator in filename
method_suffix = "_first" if TURNING_POINT_METHOD == "first" else "_algo"
output_file_path = os.path.join(script_dir, f"processed_data{version}.csv")
df_combined.to_csv(output_file_path, index=False)
print(f"\nData exported to: {output_file_path}")

# Create the plot for Keyence data with max and min marked
plt.figure(figsize=(10, 6))
plt.plot(df_keyence['time_s'], df_keyence['displacement_mm'], linewidth=0.8, label='Full Data')
plt.scatter(df_keyence.loc[keyence_max_idx, 'time_s'], 
           df_keyence.loc[keyence_max_idx, 'displacement_mm'],
           color='red', s=100, zorder=5, label=f'Max ({df_keyence.loc[keyence_max_idx, "displacement_mm"]:.4f} mm)')
plt.scatter(df_keyence.loc[keyence_min_idx, 'time_s'], 
           df_keyence.loc[keyence_min_idx, 'displacement_mm'],
           color='blue', s=100, zorder=5, label=f'Min ({df_keyence.loc[keyence_min_idx, "displacement_mm"]:.4f} mm)')
plt.axvline(x=df_keyence.loc[keyence_turning_idx, 'time_s'], 
            color='green', linestyle='--', linewidth=2, 
            label=f'Turning Point ({keyence_turning_type}) [{TURNING_POINT_METHOD}]')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (mm)')
plt.title(f'Keyence Sensor - Global Extremes and Turning Point (Method: {TURNING_POINT_METHOD})')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(graphs_dir, f'keyence_extremes{version}{method_suffix}.png'), dpi=300, bbox_inches='tight')

# Create the plot for Arduino data with max and min marked (using filtered data)
plt.figure(figsize=(10, 6))
plt.plot(df_arduino['time_s'], df_arduino['displacement_mm_filtered'], linewidth=0.8, color='orange', label='Filtered Data')
plt.plot(df_arduino['time_s'], df_arduino['displacement_mm_raw'], linewidth=0.8, color='lightcoral', alpha=0.6, label='Raw Data')
plt.scatter(df_arduino.loc[arduino_max_idx, 'time_s'], 
           df_arduino.loc[arduino_max_idx, 'displacement_mm_filtered'],
           color='red', s=100, zorder=5, label=f'Max ({df_arduino.loc[arduino_max_idx, "displacement_mm_filtered"]:.4f} mm)')
plt.scatter(df_arduino.loc[arduino_min_idx, 'time_s'], 
           df_arduino.loc[arduino_min_idx, 'displacement_mm_filtered'],
           color='blue', s=100, zorder=5, label=f'Min ({df_arduino.loc[arduino_min_idx, "displacement_mm_filtered"]:.4f} mm)')
plt.axvline(x=df_arduino.loc[arduino_turning_idx, 'time_s'], 
            color='green', linestyle='--', linewidth=2, 
            label=f'Turning Point ({arduino_turning_type}) [{TURNING_POINT_METHOD}]')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (mm)')
plt.title(f'Arduino Sensor - Global Extremes and Turning Point (Method: {TURNING_POINT_METHOD})')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(graphs_dir, f'arduino_extremes{version}{method_suffix}.png'), dpi=300, bbox_inches='tight')

# Create combined plot with both sensors after extreme point (including raw and filtered Arduino data)
plt.figure(figsize=(12, 7))
plt.plot(df_keyence_after_extreme['time_s'], df_keyence_after_extreme['displacement_mm'], 
         linewidth=1.5, label='Keyence Sensor', alpha=0.8, color='blue')
plt.plot(df_arduino_after_extreme['time_s'], df_arduino_after_extreme['displacement_mm_filtered'], 
         linewidth=1.5, label='Arduino Sensor (Filtered)', alpha=0.8, color='orange')
plt.plot(df_arduino_after_extreme['time_s'], df_arduino_after_extreme['displacement_mm_raw'], 
         linewidth=1.0, label='Arduino Sensor (Raw)', alpha=0.6, color='lightcoral', linestyle='--')
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Displacement (mm)', fontsize=12)
plt.title(f'Keyence and Arduino Sensors - Data After Turning Point (Method: {TURNING_POINT_METHOD})', 
          fontsize=14, fontweight='bold')
plt.legend(fontsize=11)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(graphs_dir, f'combined_sensors{version}{method_suffix}.png'), dpi=300, bbox_inches='tight')

# Print some summary statistics
print("\n\n=== Summary Statistics ===")
print(f"Method used: {TURNING_POINT_METHOD.upper()}")
print(f"\nKeyence - Turning point: {keyence_turning_type} at time {df_keyence.loc[keyence_turning_idx, 'time_s']:.4f} s")
print(f"Keyence - Time range: 0 to {df_keyence_after_extreme['time_s'].iloc[-1]:.4f} s")
print(f"Keyence - Displacement range: {df_keyence_after_extreme['displacement_mm'].min():.4f} to {df_keyence_after_extreme['displacement_mm'].max():.4f} mm")
print(f"\nArduino - Turning point: {arduino_turning_type} at time {df_arduino.loc[arduino_turning_idx, 'time_s']:.4f} s")
print(f"Arduino - Time range: 0 to {df_arduino_after_extreme['time_s'].iloc[-1]:.4f} s")
print(f"Arduino - Displacement range (filtered): {df_arduino_after_extreme['displacement_mm_filtered'].min():.4f} to {df_arduino_after_extreme['displacement_mm_filtered'].max():.4f} mm")
print(f"Arduino - Displacement range (raw): {df_arduino_after_extreme['displacement_mm_raw'].min():.4f} to {df_arduino_after_extreme['displacement_mm_raw'].max():.4f} mm")
print(f"\nGraphs saved to: {graphs_dir}")

plt.show()