import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.signal import argrelextrema

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

version = 29
graphs_dir = os.path.join(script_dir, 'graphs')

# Build the full path to the CSV files
arduino_file_path = os.path.join(script_dir, f"arduino_output{version}.txt")
keyence_file_path = os.path.join(script_dir, f"keyence_output{version}.csv")

time_shift = -0.028
displacement_shift = -0.00

# Import Arduino TXT file with 4 columns: time, magnetic_raw, magnetic_filtered, posic_encoder
df_arduino = pd.read_csv(
    arduino_file_path, 
    header=None, 
    delimiter=', ', 
    names=['time_ms', 'magnetic_raw_mm', 'magnetic_filtered_mm', 'posic_encoder_mm'], 
    engine='python'
)

# Strip whitespace from all columns
for col in df_arduino.columns:
    df_arduino[col] = df_arduino[col].astype(str).str.strip()

# Convert all columns to numeric, treating '-' as NaN
for col in df_arduino.columns:
    df_arduino[col] = pd.to_numeric(df_arduino[col], errors='coerce')

# Remove rows where any measurement column is NaN
df_arduino = df_arduino.dropna(subset=['magnetic_raw_mm', 'magnetic_filtered_mm', 'posic_encoder_mm'])

# Convert posic_encoder from bits to mm (with scaling)
df_arduino['posic_encoder_mm'] = df_arduino['posic_encoder_mm'] / 1000 * (-1) * 0.01953125

# Create time_s column by calculating cumulative time differences in seconds
df_arduino['time_s'] = (df_arduino['time_ms'] - df_arduino['time_ms'].iloc[0]) / 1000.0

df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] * 0.00048828125 * 0.9459754389
df_arduino['magnetic_filtered_mm'] = df_arduino['magnetic_filtered_mm'] * 0.00048828125 * 0.9335003416

# Zero all displacement values
df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] - df_arduino['magnetic_raw_mm'].iloc[0]
df_arduino['magnetic_filtered_mm'] = df_arduino['magnetic_filtered_mm'] - df_arduino['magnetic_filtered_mm'].iloc[0]
df_arduino['posic_encoder_mm'] = df_arduino['posic_encoder_mm'] - df_arduino['posic_encoder_mm'].iloc[0]
# Shift Arduino raw magnetic data vertically by 0.01 mm
df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] + displacement_shift

print("Arduino Data:")
print(df_arduino)

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
df_keyence["time_s"] = df_arduino['time_s'].iloc[:len(df_keyence)].values

# Shift Keyence time by 0.03 seconds (making it slower/later)
df_keyence['time_s'] = df_keyence['time_s'] + time_shift

# Zero the Keyence displacement values
df_keyence['displacement_mm'] = df_keyence['displacement_mm'] - df_keyence['displacement_mm'].iloc[0]

print("\nKeyence Data:")
print(df_keyence)

# Find turning points (local minima and maxima) for magnetic sensor
magnetic_data = df_arduino['magnetic_raw_mm'].values
time_data = df_arduino['time_s'].values

# Find local minima and maxima (order parameter controls sensitivity)
# Higher order = fewer, more prominent peaks; lower order = more sensitive
local_min_indices = argrelextrema(magnetic_data, np.less, order=5)[0]
local_max_indices = argrelextrema(magnetic_data, np.greater, order=5)[0]

# Get the actual values at these points
min_times = time_data[local_min_indices]
min_values = magnetic_data[local_min_indices]
max_times = time_data[local_max_indices]
max_values = magnetic_data[local_max_indices]

print(f"\nFound {len(local_min_indices)} local minima and {len(local_max_indices)} local maxima")
print("\nLocal Minima:")
for t, v in zip(min_times, min_values):
    print(f"  Time: {t:.3f}s, Displacement: {v:.4f}mm")
print("\nLocal Maxima:")
for t, v in zip(max_times, max_values):
    print(f"  Time: {t:.3f}s, Displacement: {v:.4f}mm")

# Create the plot with all sensors
plt.figure(figsize=(14, 8))

# Plot all datasets
plt.plot(df_arduino['time_s'], df_arduino['magnetic_raw_mm'], 
         label='Magnetic Sensor (Raw)', linewidth=1.5, alpha=0.7)
plt.plot(df_arduino['time_s'], df_arduino['magnetic_filtered_mm'], 
         label='Magnetic Sensor (Filtered)', linewidth=1.5, alpha=0.8)
plt.plot(df_arduino['time_s'], df_arduino['posic_encoder_mm'], 
         label='POSIC Encoder', linewidth=1.5, alpha=0.8)
plt.plot(df_keyence['time_s'], df_keyence['displacement_mm'], 
         label='Keyence', linewidth=1.5, alpha=0.8)

# Plot turning points as simple dots
plt.scatter(min_times, min_values, color='red', s=50, zorder=5, label='Local Minima')
plt.scatter(max_times, max_values, color='green', s=50, zorder=5, label='Local Maxima')

# Calculate slopes between turning points
# Combine min and max points and sort by time
all_turning_points = []
for t, v in zip(min_times, min_values):
    all_turning_points.append((t, v, 'min'))
for t, v in zip(max_times, max_values):
    all_turning_points.append((t, v, 'max'))
all_turning_points.sort(key=lambda x: x[0])

print("\n" + "="*80)
print("SLOPE ANALYSIS BETWEEN TURNING POINTS")
print("="*80)

arduino_slopes = []
keyence_slopes = []

# For each consecutive pair of turning points
for i in range(len(all_turning_points) - 1):
    t1, v1, type1 = all_turning_points[i]
    t2, v2, type2 = all_turning_points[i + 1]
    
    print(f"\nSegment {i+1}: {type1.upper()} at t={t1:.3f}s to {type2.upper()} at t={t2:.3f}s")
    
    # Get Arduino data in this time range
    arduino_mask = (time_data >= t1) & (time_data <= t2)
    arduino_segment_times = time_data[arduino_mask]
    arduino_segment_values = magnetic_data[arduino_mask]
    
    if len(arduino_segment_times) > 1:
        # Calculate line of best fit (polyfit degree 1)
        arduino_coeffs = np.polyfit(arduino_segment_times, arduino_segment_values, 1)
        arduino_slope = arduino_coeffs[0]
        # After:
        
        if abs(arduino_slope) >= 0.10:
            print(arduino_slope)
            arduino_slopes.append(abs(arduino_slope))
        print(f"  Arduino slope: {arduino_slope:.6f} mm/s")
    
    # Get Keyence data in this time range
    keyence_times = df_keyence['time_s'].values
    keyence_values = df_keyence['displacement_mm'].values
    keyence_mask = (keyence_times >= t1) & (keyence_times <= t2)
    keyence_segment_times = keyence_times[keyence_mask]
    keyence_segment_values = keyence_values[keyence_mask]
    
    if len(keyence_segment_times) > 1:
        # Calculate line of best fit
        keyence_coeffs = np.polyfit(keyence_segment_times, keyence_segment_values, 1)
        keyence_slope = keyence_coeffs[0]
        keyence_slopes.append(abs(keyence_slope))
        print(f"  Keyence slope: {keyence_slope:.6f} mm/s")

# Calculate averages
print("\n" + "="*80)
print("AVERAGE SLOPES")
print("="*80)
if arduino_slopes:
    avg_arduino_slope = np.mean(arduino_slopes)
    print(f"Arduino average slope: {avg_arduino_slope:.6f} mm/s")
    print(f"  (based on {len(arduino_slopes)} segments)")
if keyence_slopes:
    avg_keyence_slope = np.mean(keyence_slopes)
    print(f"Keyence average slope: {avg_keyence_slope:.6f} mm/s")
    print(f"  (based on {len(keyence_slopes)} segments)")
print("="*80 + "\n")

# Add labels and title
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Displacement (mm)', fontsize=12)
plt.title(f'Displacement Comparison - All Sensors with Turning Points (Version {version})', fontsize=14)
plt.legend(fontsize=10)
plt.grid(True, alpha=0.3)

# Tight layout for better spacing
plt.tight_layout()

# Save the figure
os.makedirs(graphs_dir, exist_ok=True)
output_path = os.path.join(graphs_dir, f'all_sensors_plot{version}_turning_points.png')
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"\nPlot saved to: {output_path}")

# Display the plot
plt.show()

# Export combined dataframe to CSV
# Merge Arduino and Keyence data based on time alignment
df_combined = df_arduino[['time_s', 'magnetic_raw_mm', 'magnetic_filtered_mm', 'posic_encoder_mm']].copy()

# Add Keyence data (will be NaN for times beyond Keyence data length)
df_combined['keyence_mm'] = np.nan
keyence_length = len(df_keyence)
df_combined.loc[:keyence_length-1, 'keyence_mm'] = df_keyence['displacement_mm'].values

# Export to CSV
csv_output_path = os.path.join(script_dir, f'combined_sensor_data_v{version}.csv')
df_combined.to_csv(csv_output_path, index=False)
print(f"\nCombined data exported to: {csv_output_path}")
print(f"Columns: {list(df_combined.columns)}")
print(f"Total rows: {len(df_combined)}")