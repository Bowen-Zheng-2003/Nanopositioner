import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.signal import argrelextrema
from scipy import signal

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

version = 28
graphs_dir = os.path.join(script_dir, 'graphs')

# Build the full path to the CSV files
arduino_file_path = os.path.join(script_dir, f"arduino_output{version}.txt")
keyence_file_path = os.path.join(script_dir, f"keyence_output{version}.csv")

time_shift = -0
time_shift_filtered = -0
displacement_shift = -0.00

# Outlier removal threshold for consecutive magnetic_raw differences (mm)
max_consecutive_diff = 25

def apply_lookup_table(magnetic_values, lookup_table_path, N):
    """
    Apply a lookup table to convert magnetic sensor readings to calibrated displacement values.
    Uses linear interpolation for non-integer indices.
    Automatically determines the magnetic range from the lookup table.
    
    Parameters:
    -----------
    magnetic_values : numpy.array or pandas.Series
        Raw magnetic sensor readings to be calibrated
    lookup_table_path : str
        Path to the lookup table txt file
    N : int
        Bit resolution used to create the lookup table (table size = 2^N)
    
    Returns:
    --------
    calibrated_values : numpy.array
        Calibrated displacement values after applying lookup table
    """
    
    # Load the lookup table
    with open(lookup_table_path, 'r') as f:
        content = f.read()
        lookup_table = np.array([float(x) for x in content.split(',')])
    
    table_size = 2**N
    
    # Convert magnetic values to numpy array if it's a pandas Series
    if isinstance(magnetic_values, pd.Series):
        magnetic_values = magnetic_values.values
    
    # Automatically determine magnetic_min and magnetic_max from the actual data
    valid_magnetic = magnetic_values[~np.isnan(magnetic_values)]
    magnetic_min = np.min(valid_magnetic)
    magnetic_max = np.max(valid_magnetic)
    
    print("\n" + "="*80)
    print("APPLYING LOOKUP TABLE CALIBRATION")
    print("="*80)
    print(f"Lookup table path: {lookup_table_path}")
    print(f"Lookup table size: {table_size} entries (N={N})")
    print(f"Auto-detected magnetic range from data:")
    print(f"  Magnetic min: {magnetic_min:.4f} mm")
    print(f"  Magnetic max: {magnetic_max:.4f} mm")
    print(f"Input data points: {len(magnetic_values)}")
    
    # Initialize output array
    calibrated_values = np.zeros_like(magnetic_values, dtype=float)
    
    # Track statistics
    clipped_low = 0
    clipped_high = 0
    interpolated = 0
    exact_matches = 0
    
    for i, mag_val in enumerate(magnetic_values):
        # Handle NaN values
        if np.isnan(mag_val):
            calibrated_values[i] = np.nan
            continue
        
        # Convert magnetic value to floating-point index
        # Map magnetic_min to index 0, magnetic_max to index (table_size - 1)
        float_index = (mag_val - magnetic_min) / (magnetic_max - magnetic_min) * (table_size - 1)
        
        # Clip to valid range and track out-of-bounds values
        if float_index < 0:
            float_index = 0
            clipped_low += 1
        elif float_index > table_size - 1:
            float_index = table_size - 1
            clipped_high += 1
        
        # Get the integer indices surrounding the float_index
        index_low = int(np.floor(float_index))
        index_high = int(np.ceil(float_index))
        
        # If float_index is exactly an integer, no interpolation needed
        if index_low == index_high:
            calibrated_values[i] = lookup_table[index_low]
            exact_matches += 1
        else:
            # Linear interpolation between the two surrounding values
            fraction = float_index - index_low
            calibrated_values[i] = (1 - fraction) * lookup_table[index_low] + fraction * lookup_table[index_high]
            interpolated += 1
    
    print(f"\nCalibration Statistics:")
    print(f"  Exact index matches: {exact_matches}")
    print(f"  Interpolated values: {interpolated}")
    print(f"  Clipped to lower bound: {clipped_low}")
    print(f"  Clipped to upper bound: {clipped_high}")
    print(f"  Output range: {np.nanmin(calibrated_values):.4f} to {np.nanmax(calibrated_values):.4f} mm")
    print("="*80 + "\n")
    
    return calibrated_values


def plot_lut_calibration_comparison(df_arduino, calibrated_values, version):
    """
    Plot comparison between original magnetic filtered data and LUT-calibrated data.
    
    Parameters:
    -----------
    df_arduino : pandas.DataFrame
        Arduino dataframe with time and sensor data
    calibrated_values : numpy.array
        Calibrated values from lookup table
    version : int
        Version number for plot title and filename
    """
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 16))
    
    time_data = df_arduino['time_s'].values
    
    # Plot 2: If Keyence data exists, compare LUT calibrated to Keyence
    if 'displacement_mm' in df_arduino.columns:
        keyence_mask = df_arduino['displacement_mm'].notna()
        
        ax1.scatter(time_data[keyence_mask], df_arduino.loc[keyence_mask, 'displacement_mm'], 
                   label='Keyence', s=10, alpha=0.5, color='orange')
        ax1.scatter(time_data, calibrated_values, 
                   label='Magnetic LUT Calibrated', s=10, alpha=0.5, color='red')
        
        # Calculate error between LUT calibrated and Keyence
        error = calibrated_values[keyence_mask] - df_arduino.loc[keyence_mask, 'displacement_mm'].values
        rmse = np.sqrt(np.mean(error**2))
        mean_error = np.mean(error)
        std_error = np.std(error)
        
        stats_text = f'RMSE: {rmse:.4f} mm\nMean Error: {mean_error:.4f} mm\nStd Dev: {std_error:.4f} mm'
        ax1.text(0.02, 0.98, stats_text, 
                transform=ax1.transAxes, 
                fontsize=11, 
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        ax1.set_xlabel('Time (s)', fontsize=12)
        ax1.set_ylabel('Displacement (mm)', fontsize=12)
        ax1.set_title(f'LUT Calibrated vs Keyence Reference (Version {version})', fontsize=14)
        ax1.legend(fontsize=10)
        ax1.grid(True, alpha=0.3)
        
        # Plot 3: Error plot (LUT Calibrated - Keyence)
        ax2.scatter(time_data[keyence_mask], error, 
                   s=10, alpha=0.6, color='purple', label='Error (LUT - Keyence)')
        ax2.axhline(y=0, color='black', linestyle='--', linewidth=1.5, alpha=0.7, label='Zero Error')
        ax2.axhline(y=mean_error, color='red', linestyle=':', linewidth=1.5, alpha=0.7, label=f'Mean Error: {mean_error:.4f} mm')
        
        # Add ±1 std deviation lines
        ax2.axhline(y=mean_error + std_error, color='orange', linestyle=':', linewidth=1, alpha=0.5)
        ax2.axhline(y=mean_error - std_error, color='orange', linestyle=':', linewidth=1, alpha=0.5)
        ax2.fill_between(time_data[keyence_mask], 
                         mean_error - std_error, 
                         mean_error + std_error, 
                         alpha=0.2, 
                         color='orange',
                         label=f'±1 Std Dev ({std_error:.4f} mm)')
        
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Error (mm)', fontsize=12)
        ax2.set_title(f'Error: LUT Calibrated - Keyence (Version {version})', fontsize=14)
        ax2.legend(fontsize=10, loc='best')
        ax2.grid(True, alpha=0.3)
        
    else:
        ax1.text(0.5, 0.5, 'Keyence data not available for comparison', 
                ha='center', va='center', transform=ax2.transAxes, fontsize=12)
        ax2.text(0.5, 0.5, 'Keyence data not available for error plot', 
                ha='center', va='center', transform=ax3.transAxes, fontsize=12)
    
    plt.tight_layout()
    
    # Save the figure
    output_path = os.path.join(graphs_dir, f'lut_calibration_comparison_v{version}.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"LUT calibration comparison plot saved to: {output_path}")
    
    plt.show()

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
scale_factor = 1.694
# df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] * 0.00048828125*scale_factor
# df_arduino['magnetic_filtered_mm'] = df_arduino['magnetic_filtered_mm'] * 0.00048828125*scale_factor
df_arduino['magnetic_raw_mm'] = (df_arduino['magnetic_raw_mm'] + 15000) * -1
df_arduino['magnetic_filtered_mm'] = (df_arduino['magnetic_filtered_mm']  + 15000) * -1

# # Zero all displacement values
# df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] - df_arduino['magnetic_raw_mm'].iloc[0]
# df_arduino['magnetic_filtered_mm'] = df_arduino['magnetic_filtered_mm'] - df_arduino['magnetic_filtered_mm'].iloc[0]
# df_arduino['posic_encoder_mm'] = df_arduino['posic_encoder_mm'] - df_arduino['posic_encoder_mm'].iloc[0]
# # Shift Arduino raw magnetic data vertically
# df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] + displacement_shift

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
df_keyence['displacement_mm'] = df_keyence['displacement_mm'] * -1

# ============================================================================
# MERGE KEYENCE DATA INTO DF_ARDUINO
# ============================================================================
# Add Keyence data to df_arduino (initialize with NaN)
df_arduino['displacement_mm'] = np.nan
keyence_length = len(df_keyence)
df_arduino.loc[:keyence_length-1, 'displacement_mm'] = df_keyence['displacement_mm'].values

print(f"\nMerged {keyence_length} Keyence data points into df_arduino")

# ============================================================================
# REMOVE OUTLIERS BASED ON CONSECUTIVE DIFFERENCES (AFTER MERGE)
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

# print(len(df_arduino))

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
# APPLY LOOKUP TABLE CALIBRATION
# ============================================================================
# Specify lookup table parameters
N = 12  # Must match the N used when creating the lookup table
lookup_table_path = os.path.join(script_dir, f'lookup_table_v26_N{N}.txt')
# lookup_table_path = os.path.join(script_dir, f'lookup_table_v{version}_N{N}.txt')

# Apply the lookup table to the magnetic filtered data
df_arduino['magnetic_lut_calibrated'] = apply_lookup_table(
    df_arduino['magnetic_raw_filtered'],
    lookup_table_path,
    N
)

# Plot the comparison
plot_lut_calibration_comparison(df_arduino, df_arduino['magnetic_lut_calibrated'].values, version)


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
# plt.show()

# ============================================================================
# CALCULATE SLOPES BETWEEN TURNING POINTS
# ============================================================================
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
filtered_slopes = []

filtered_data = df_arduino['magnetic_raw_filtered'].values

for i in range(len(all_turning_points) - 1):
    t1, v1, type1 = all_turning_points[i]
    t2, v2, type2 = all_turning_points[i + 1]
    
    print(f"\nSegment {i+1}: {type1.upper()} at t={t1:.3f}s to {type2.upper()} at t={t2:.3f}s")
    
    # Arduino raw data
    arduino_mask = (time_data >= t1) & (time_data <= t2)
    arduino_segment_times = time_data[arduino_mask]
    arduino_segment_values = magnetic_data[arduino_mask]
    
    if len(arduino_segment_times) > 1:
        arduino_coeffs = np.polyfit(arduino_segment_times, arduino_segment_values, 1)
        arduino_slope = arduino_coeffs[0]
        arduino_slopes.append(abs(arduino_slope))
        print(f"  Arduino (raw) slope: {arduino_slope:.6f} mm/s")
    
    # Arduino filtered data
    filtered_segment_values = filtered_data[arduino_mask]
    if len(arduino_segment_times) > 1:
        filtered_coeffs = np.polyfit(arduino_segment_times, filtered_segment_values, 1)
        filtered_slope = filtered_coeffs[0]
        filtered_slopes.append(abs(filtered_slope))
        print(f"  Arduino (filtered) slope: {filtered_slope:.6f} mm/s")
    
    # Keyence data
    keyence_times = df_keyence['time_s'].values
    keyence_values = df_keyence['displacement_mm'].values
    keyence_mask = (keyence_times >= t1) & (keyence_times <= t2)
    keyence_segment_times = keyence_times[keyence_mask]
    keyence_segment_values = keyence_values[keyence_mask]
    
    if len(keyence_segment_times) > 1:
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
    print(f"Arduino (raw) average slope: {avg_arduino_slope:.6f} mm/s")
    print(f"  (based on {len(arduino_slopes)} segments)")
if filtered_slopes:
    avg_filtered_slope = np.mean(filtered_slopes)
    print(f"Arduino (filtered) average slope: {avg_filtered_slope:.6f} mm/s")
    print(f"  (based on {len(filtered_slopes)} segments)")
if keyence_slopes:
    avg_keyence_slope = np.mean(keyence_slopes)
    print(f"Keyence average slope: {avg_keyence_slope:.6f} mm/s")
    print(f"  (based on {len(keyence_slopes)} segments)")
print("="*80 + "\n")

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