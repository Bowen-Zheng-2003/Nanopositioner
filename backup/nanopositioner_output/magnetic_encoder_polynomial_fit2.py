import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.signal import argrelextrema
from scipy import signal

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

version = 26
graphs_dir = os.path.join(script_dir, 'graphs')

# Build the full path to the CSV files
arduino_file_path = os.path.join(script_dir, f"arduino_output{version}.txt")
keyence_file_path = os.path.join(script_dir, f"keyence_output{version}.csv")

time_shift = -0
time_shift_filtered = -0
displacement_shift = -0.00
error_correction_function = [1.56349485e-02, 3.29054137e-01, 2.87548000e+00, 1.28833164e+01, 2.60340140e+01, -1.65938741e+01, -2.04232084e+02, -3.01833443e+02, 7.01568061e+02, 3.85068074e+03, 8.07724810e+03, 9.95394613e+03, 7.48092099e+03, 2.92764433e+03, -8.35559913e+01, -6.91437546e+02, -3.21169108e+02, -5.83443744e+01, -3.38699946e+00, -1.03061261e+00, 2.62987204e-03]
# Outlier removal threshold for consecutive magnetic_raw differences (mm)
max_consecutive_diff = 0.05

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

# df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] * 0.00048828125 * 0.9406912098
# df_arduino['magnetic_filtered_mm'] = df_arduino['magnetic_filtered_mm'] * 0.00048828125 * 0.9335003416
# scale_factor = 3
df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] * 0.00048828125 * 1.694
df_arduino['magnetic_filtered_mm'] = df_arduino['magnetic_filtered_mm'] * 0.00048828125 * 1.694

# Zero all displacement values
df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] - df_arduino['magnetic_raw_mm'].iloc[0]
df_arduino['magnetic_filtered_mm'] = df_arduino['magnetic_filtered_mm'] - df_arduino['magnetic_filtered_mm'].iloc[0]
df_arduino['posic_encoder_mm'] = df_arduino['posic_encoder_mm'] - df_arduino['posic_encoder_mm'].iloc[0]
# Shift Arduino raw magnetic data vertically
df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] + displacement_shift

# ============================================================================
# REMOVE OUTLIERS BASED ON CONSECUTIVE DIFFERENCES
# ============================================================================
print("\n" + "="*80)
print("REMOVING OUTLIERS BASED ON CONSECUTIVE DIFFERENCES")
print("="*80)

initial_rows = len(df_arduino)

# Calculate absolute difference between consecutive magnetic_raw values
df_arduino['consecutive_diff'] = df_arduino['magnetic_raw_mm'].diff().abs()

# Keep first row (has NaN diff) and rows where diff is below threshold
rows_to_keep = (df_arduino['consecutive_diff'].isna()) | (df_arduino['consecutive_diff'] <= max_consecutive_diff)
df_arduino = df_arduino[rows_to_keep].copy()

# Drop the temporary column
df_arduino = df_arduino.drop(columns=['consecutive_diff'])

# Reset index
df_arduino = df_arduino.reset_index(drop=True)

removed_rows = initial_rows - len(df_arduino)
print(f"Initial rows: {initial_rows}")
print(f"Removed rows: {removed_rows}")
print(f"Remaining rows: {len(df_arduino)}")
print(f"Threshold used: {max_consecutive_diff} mm")
print("="*80 + "\n")

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

# ============================================================================
# APPLY LOW-PASS FILTER TO MAGNETIC_RAW DATA
# ============================================================================
# print("\n" + "="*80)
# print("APPLYING LOW-PASS FILTER")
# print("="*80)

samplingFreq = 200  # matching sampling frequency of arduino itself
cutoffFreq = 7

# Low-pass filter design
w0 = 2*np.pi*cutoffFreq  # pole frequency (rad/s) (cutoff frequency)
num = w0        # transfer function numerator coefficients
den = [1, w0]   # transfer function denominator coefficients
lowPass = signal.TransferFunction(num, den)

# Convert to discrete time
dt = 1.0/samplingFreq
discreteLowPass = lowPass.to_discrete(dt, method='gbt', alpha=0.5)

# Get filter coefficients
b = discreteLowPass.num
a = -discreteLowPass.den

# print(f"Discrete Low-Pass Filter: {discreteLowPass}")
# print(f"Filter coefficients b_i: {b}")
# print(f"Filter coefficients a_i: {a[1:]}")

# Apply filter to magnetic_raw data
y = df_arduino['magnetic_raw_mm'].values
yfilt = np.zeros(len(y))

# Apply the filter equation: yfilt[i] = a[1]*yfilt[i-1] + b[0]*y[i] + b[1]*y[i-1]
for i in range(1, len(y)):
    yfilt[i] = a[1]*yfilt[i-1] + b[0]*y[i] + b[1]*y[i-1]

# Add filtered data to dataframe
df_arduino['magnetic_raw_filtered'] = yfilt

# Time shift the filtered data
df_arduino['magnetic_raw_filtered_time'] = df_arduino['time_s'] + time_shift_filtered

# print(f"\nFilter applied successfully to {len(y)} data points")
# print("="*80 + "\n")

# ============================================================================
# FIND TURNING POINTS FOR ORIGINAL MAGNETIC_RAW DATA
# ============================================================================
magnetic_data = df_keyence['displacement_mm'].values
time_data = df_arduino['time_s'].values

# Find local minima and maxima
local_min_indices = argrelextrema(magnetic_data, np.less, order=5)[0]
local_max_indices = argrelextrema(magnetic_data, np.greater, order=5)[0]

min_times = time_data[local_min_indices]
min_values = magnetic_data[local_min_indices]
max_times = time_data[local_max_indices]
max_values = magnetic_data[local_max_indices]

print(f"Found {len(local_min_indices)} local minima and {len(local_max_indices)} local maxima")

# ============================================================================
# APPLY ERROR CORRECTION POLYNOMIAL TO FILTERED DATA
# ============================================================================
print("\n" + "="*80)
print("APPLYING ERROR CORRECTION POLYNOMIAL")
print("="*80)

# The error_correction_function coefficients are ordered from highest degree to lowest
# For an 8-element array, this represents a 7th-degree polynomial
poly_order = len(error_correction_function) - 1
print(f"Polynomial order: {poly_order}")
print(f"Coefficients (highest to lowest degree): {error_correction_function}")

# Apply the polynomial using numpy's polyval function
# polyval expects coefficients in descending order (highest degree first)
error_correction_values = pd.DataFrame(
    np.polyval(error_correction_function, df_arduino['magnetic_raw_filtered']),
    columns=['corrected_values']
)

# Create new column with corrected data
df_arduino['magnetic_corrected_mm'] = (df_arduino['magnetic_raw_filtered'] + error_correction_values['corrected_values'])
print(df_arduino['magnetic_corrected_mm'])

# df_arduino['magnetic_corrected_mm'] = df_arduino['magnetic_corrected_mm'] * 1.69

# print(f"Error correction applied to {len(df_arduino)} data points")
# print(f"Error correction range: {error_correction_values.min():.6f} to {error_correction_values.max():.6f} mm")
print("="*80 + "\n")

# ============================================================================
# CREATE THE MAIN PLOT
# ============================================================================
plt.figure(figsize=(14, 8))

# Plot magnetic raw, filtered version, and keyence as scatter plots
plt.scatter(df_arduino['time_s'], df_arduino['magnetic_raw_mm'], 
           label='Magnetic Sensor (Raw)', s=10, alpha=0.5, color='blue')
plt.scatter(df_arduino['magnetic_raw_filtered_time'], df_arduino['magnetic_raw_filtered'], 
           label='Magnetic Sensor (Low-Pass Filtered)', s=15, alpha=0.8, color='darkblue')
plt.scatter(df_keyence['time_s'], df_keyence['displacement_mm'], 
           label='Keyence', s=10, alpha=0.8, color='orange')
plt.scatter(df_arduino['time_s'], df_arduino['magnetic_corrected_mm'], 
           label='Magnetic Sensor (Corrected)', s=15, alpha=0.8, color='purple')

# Plot turning points
plt.scatter(min_times, min_values, color='red', s=50, zorder=5, label='Local Minima')
plt.scatter(max_times, max_values, color='green', s=50, zorder=5, label='Local Maxima')

# Add labels and title
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Displacement (mm)', fontsize=12)
plt.title(f'Displacement Comparison with Low-Pass Filter (Version {version})', fontsize=14)
plt.legend(fontsize=10, loc='best')
plt.grid(True, alpha=0.3)
plt.tight_layout()

# Save the figure
os.makedirs(graphs_dir, exist_ok=True)
output_path = os.path.join(graphs_dir, f'filtered_comparison_v{version}.png')
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"Plot saved to: {output_path}")

# Display the plot
plt.show()

# ============================================================================
# CALCULATE SLOPES BETWEEN TURNING POINTS
# ============================================================================
# all_turning_points = []
# for t, v in zip(min_times, min_values):
#     all_turning_points.append((t, v, 'min'))
# for t, v in zip(max_times, max_values):
#     all_turning_points.append((t, v, 'max'))
# all_turning_points.sort(key=lambda x: x[0])

# print("\n" + "="*80)
# print("SLOPE ANALYSIS BETWEEN TURNING POINTS")
# print("="*80)

# arduino_slopes = []
# keyence_slopes = []
# filtered_slopes = []

# filtered_data = df_arduino['magnetic_raw_filtered'].values

# for i in range(len(all_turning_points) - 1):
#     t1, v1, type1 = all_turning_points[i]
#     t2, v2, type2 = all_turning_points[i + 1]
    
#     print(f"\nSegment {i+1}: {type1.upper()} at t={t1:.3f}s to {type2.upper()} at t={t2:.3f}s")
    
#     # Arduino raw data
#     arduino_mask = (time_data >= t1) & (time_data <= t2)
#     arduino_segment_times = time_data[arduino_mask]
#     arduino_segment_values = magnetic_data[arduino_mask]
    
#     if len(arduino_segment_times) > 1:
#         arduino_coeffs = np.polyfit(arduino_segment_times, arduino_segment_values, 1)
#         arduino_slope = arduino_coeffs[0]
#         arduino_slopes.append(abs(arduino_slope))
#         print(f"  Arduino (raw) slope: {arduino_slope:.6f} mm/s")
    
#     # Arduino filtered data
#     filtered_segment_values = filtered_data[arduino_mask]
#     if len(arduino_segment_times) > 1:
#         filtered_coeffs = np.polyfit(arduino_segment_times, filtered_segment_values, 1)
#         filtered_slope = filtered_coeffs[0]
#         filtered_slopes.append(abs(filtered_slope))
#         print(f"  Arduino (filtered) slope: {filtered_slope:.6f} mm/s")
    
#     # Keyence data
#     keyence_times = df_keyence['time_s'].values
#     keyence_values = df_keyence['displacement_mm'].values
#     keyence_mask = (keyence_times >= t1) & (keyence_times <= t2)
#     keyence_segment_times = keyence_times[keyence_mask]
#     keyence_segment_values = keyence_values[keyence_mask]
    
#     if len(keyence_segment_times) > 1:
#         keyence_coeffs = np.polyfit(keyence_segment_times, keyence_segment_values, 1)
#         keyence_slope = keyence_coeffs[0]
#         keyence_slopes.append(abs(keyence_slope))
#         print(f"  Keyence slope: {keyence_slope:.6f} mm/s")

# # Calculate averages
# print("\n" + "="*80)
# print("AVERAGE SLOPES")
# print("="*80)
# if arduino_slopes:
#     avg_arduino_slope = np.mean(arduino_slopes)
#     print(f"Arduino (raw) average slope: {avg_arduino_slope:.6f} mm/s")
#     print(f"  (based on {len(arduino_slopes)} segments)")
# if filtered_slopes:
#     avg_filtered_slope = np.mean(filtered_slopes)
#     print(f"Arduino (filtered) average slope: {avg_filtered_slope:.6f} mm/s")
#     print(f"  (based on {len(filtered_slopes)} segments)")
# if keyence_slopes:
#     avg_keyence_slope = np.mean(keyence_slopes)
#     print(f"Keyence average slope: {avg_keyence_slope:.6f} mm/s")
#     print(f"  (based on {len(keyence_slopes)} segments)")
# print("="*80 + "\n")

# Export combined dataframe to CSV
df_combined = df_arduino[['time_s', 'magnetic_raw_mm', 'magnetic_raw_filtered', 
                           'magnetic_filtered_mm', 'posic_encoder_mm']].copy()

# Add Keyence data
df_combined['keyence_mm'] = np.nan
keyence_length = len(df_keyence)
df_combined.loc[:keyence_length-1, 'keyence_mm'] = df_keyence['displacement_mm'].values

# Export to CSV
csv_output_path = os.path.join(script_dir, f'combined_sensor_data_v{version}_filtered.csv')
df_combined.to_csv(csv_output_path, index=False)
print(f"Combined data exported to: {csv_output_path}")
# print(f"Columns: {list(df_combined.columns)}")
# print(f"Total rows: {len(df_combined)}")