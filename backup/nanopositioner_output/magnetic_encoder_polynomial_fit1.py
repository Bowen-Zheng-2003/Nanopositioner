import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.signal import argrelextrema
from scipy import signal

def create_extrema_segments(df_merged, min_times, max_times):
    """
    Create columns for each segment between consecutive extrema.
    Each segment gets its own normalized time (starting at 0) and error column.
    
    Parameters:
    -----------
    df_merged : pandas.DataFrame
        The merged dataframe containing all sensor data
    min_times : numpy.array
        Array of times where local minima occur
    max_times : numpy.array
        Array of times where local maxima occur
    
    Returns:
    --------
    df_merged : pandas.DataFrame
        The input dataframe with new columns added for each segment
    segment_info : list of dict
        Information about each segment (start_time, end_time, segment_number)
    """
    
    # Combine and sort all extrema times
    all_extrema_times = sorted(list(min_times) + list(max_times))
    
    print(f"\nFound {len(all_extrema_times)} total extrema")
    print(f"Creating {len(all_extrema_times) - 1} segments")
    
    segment_info = []
    
    # Loop through consecutive pairs of extrema
    for i in range(len(all_extrema_times) - 1):
        t_start = all_extrema_times[i]
        t_end = all_extrema_times[i + 1]
        
        # Create mask for this time segment
        segment_mask = (df_merged['time_s'] >= t_start) & (df_merged['time_s'] <= t_end)
        
        # Create column names for this segment
        segment_num = i + 1
        time_col = f'time_segment_{segment_num}'
        magnetic_col = f'magnetic_segment_{segment_num}'
        keyence_col = f'keyence_segment_{segment_num}'
        error_col = f'error_segment_{segment_num}'
        
        # Initialize columns with NaN
        df_merged[time_col] = np.nan
        df_merged[magnetic_col] = np.nan
        df_merged[keyence_col] = np.nan
        df_merged[error_col] = np.nan
        
        # Fill in the data for this segment with normalized time (starting at 0)
        df_merged.loc[segment_mask, time_col] = df_merged.loc[segment_mask, 'time_s'] - t_start
        df_merged.loc[segment_mask, magnetic_col] = df_merged.loc[segment_mask, 'magnetic_raw_filtered']
        df_merged.loc[segment_mask, keyence_col] = df_merged.loc[segment_mask, 'displacement_mm']
        df_merged.loc[segment_mask, error_col] = (
            df_merged.loc[segment_mask, 'displacement_mm'] - 
            df_merged.loc[segment_mask, 'magnetic_raw_filtered']
        )
        
        # Store segment information
        segment_info.append({
            'segment_num': segment_num,
            'start_time': t_start,
            'end_time': t_end,
            'duration': t_end - t_start,
            'time_col': time_col,
            'magnetic_col': magnetic_col,
            'keyence_col': keyence_col,
            'error_col': error_col
        })
        
        print(f"Segment {segment_num}: {t_start:.3f}s to {t_end:.3f}s (duration: {t_end - t_start:.3f}s)")
    
    return df_merged, segment_info


def plot_overlapping_segments(df_merged, segment_info, version):
    """
    Plot all segments overlapping with normalized time.
    Odd segments on one subplot, even segments on another, and displacement data on a third.
    
    Parameters:
    -----------
    df_merged : pandas.DataFrame
        The dataframe with segment columns
    segment_info : list of dict
        Information about each segment
    version : int
        Version number for the plot title and filename
    """
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 16))
    
    # Separate odd and even segments
    odd_segments = [seg for seg in segment_info if seg['segment_num'] % 2 == 1]
    even_segments = [seg for seg in segment_info if seg['segment_num'] % 2 == 0]
    
    # Define colormaps for different segments
    colors_odd = plt.cm.viridis(np.linspace(0, 1, len(odd_segments)))
    colors_even = plt.cm.plasma(np.linspace(0, 1, len(even_segments)))
    
    # Plot odd segments - ERROR
    for i, segment in enumerate(odd_segments):
        segment_mask = df_merged[segment['error_col']].notna()
        
        ax1.plot(
            df_merged.loc[segment_mask, segment['time_col']],
            df_merged.loc[segment_mask, segment['error_col']],
            label=f"Segment {segment['segment_num']} ({segment['start_time']:.2f}s, Δt={segment['duration']:.2f}s)",
            linewidth=2,
            alpha=0.7,
            color=colors_odd[i]
        )
    
    ax1.set_xlabel('Normalized Time (s) - Starting from 0', fontsize=12)
    ax1.set_ylabel('Error (Keyence - Magnetic) (mm)', fontsize=12)
    ax1.set_title(f'Odd Segments - Error (Version {version})', fontsize=14)
    ax1.legend(fontsize=9, loc='best')
    ax1.grid(True, alpha=0.3)
    
    # Plot even segments - ERROR
    for i, segment in enumerate(even_segments):
        segment_mask = df_merged[segment['error_col']].notna()
        
        ax2.plot(
            df_merged.loc[segment_mask, segment['time_col']],
            df_merged.loc[segment_mask, segment['error_col']],
            label=f"Segment {segment['segment_num']} ({segment['start_time']:.2f}s, Δt={segment['duration']:.2f}s)",
            linewidth=2,
            alpha=0.7,
            color=colors_even[i]
        )
    
    ax2.set_xlabel('Normalized Time (s) - Starting from 0', fontsize=12)
    ax2.set_ylabel('Error (Keyence - Magnetic) (mm)', fontsize=12)
    ax2.set_title(f'Even Segments - Error (Version {version})', fontsize=14)
    ax2.legend(fontsize=9, loc='best')
    ax2.grid(True, alpha=0.3)
    
    # Plot displacement data - KEYENCE and MAGNETIC
    for i, segment in enumerate(segment_info):
        segment_mask = df_merged[segment['keyence_col']].notna()
        
        # Use consistent color for each segment type
        if segment['segment_num'] % 2 == 1:
            color_idx = odd_segments.index(segment)
            color = colors_odd[color_idx]
        else:
            color_idx = even_segments.index(segment)
            color = colors_even[color_idx]
        
        # Plot Keyence (solid line)
        ax3.plot(
            df_merged.loc[segment_mask, segment['time_col']],
            df_merged.loc[segment_mask, segment['keyence_col']],
            linewidth=2,
            alpha=0.7,
            color=color,
            linestyle='-',
            label=f"Seg {segment['segment_num']} Keyence"
        )
        
        # Plot Magnetic (dashed line)
        ax3.plot(
            df_merged.loc[segment_mask, segment['time_col']],
            df_merged.loc[segment_mask, segment['magnetic_col']],
            linewidth=2,
            alpha=0.7,
            color=color,
            linestyle='--',
            label=f"Seg {segment['segment_num']} Magnetic"
        )
    
    ax3.set_xlabel('Normalized Time (s) - Starting from 0', fontsize=12)
    ax3.set_ylabel('Displacement (mm)', fontsize=12)
    ax3.set_title(f'All Segments - Keyence (solid) vs Magnetic (dashed) (Version {version})', fontsize=14)
    ax3.legend(fontsize=8, loc='best', ncol=2)
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save the figure
    output_path = os.path.join(graphs_dir, f'error_segments_v{version}_overlapping.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\nOverlapping segments plot saved to: {output_path}")
    print(f"Odd segments: {len(odd_segments)}, Even segments: {len(even_segments)}")
    
    plt.show()

def plot_single_segment(df_merged, segment_info, segment_num, version):
    """
    Plot a single segment showing error, keyence, and magnetic data.
    
    Parameters:
    -----------
    df_merged : pandas.DataFrame
        The dataframe with segment columns
    segment_info : list of dict
        Information about each segment
    segment_num : int
        The segment number to plot (1-indexed)
    version : int
        Version number for the plot title and filename
    """
    
    # Find the segment info
    segment = None
    for seg in segment_info:
        if seg['segment_num'] == segment_num:
            segment = seg
            break
    
    if segment is None:
        print(f"Error: Segment {segment_num} not found!")
        return
    
    # Create figure with 2 subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Get the mask for this segment
    segment_mask = df_merged[segment['error_col']].notna()
    
    # Plot 1: Error
    ax1.plot(
        df_merged.loc[segment_mask, segment['time_col']],
        df_merged.loc[segment_mask, segment['error_col']],
        linewidth=2,
        color='red',
        label='Error (Keyence - Magnetic)'
    )
    
    ax1.set_xlabel('Normalized Time (s)', fontsize=12)
    ax1.set_ylabel('Error (mm)', fontsize=12)
    ax1.set_title(f'Segment {segment_num} - Error (Start: {segment["start_time"]:.2f}s, Duration: {segment["duration"]:.2f}s) - Version {version}', fontsize=14)
    ax1.legend(fontsize=10, loc='best')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Keyence vs Magnetic
    ax2.plot(
        df_merged.loc[segment_mask, segment['time_col']],
        df_merged.loc[segment_mask, segment['keyence_col']],
        linewidth=2,
        color='blue',
        linestyle='-',
        label='Keyence'
    )
    
    ax2.plot(
        df_merged.loc[segment_mask, segment['time_col']],
        df_merged.loc[segment_mask, segment['magnetic_col']],
        linewidth=2,
        color='green',
        linestyle='--',
        label='Magnetic (filtered)'
    )
    
    ax2.set_xlabel('Normalized Time (s)', fontsize=12)
    ax2.set_ylabel('Displacement (mm)', fontsize=12)
    ax2.set_title(f'Segment {segment_num} - Keyence vs Magnetic - Version {version}', fontsize=14)
    ax2.legend(fontsize=10, loc='best')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save the figure
    output_path = os.path.join(graphs_dir, f'segment_{segment_num}_v{version}.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\nSegment {segment_num} plot saved to: {output_path}")
    
    plt.show()

def plot_segment_with_polynomial_fit(df_merged, segment_info, segment_num, version, max_degree=10, use_degree=None):
    """
    Plot a single segment with polynomial curve fitting.
    Tests polynomial degrees from 1 to max_degree and selects the one with highest R².
    Optionally use a specific degree instead of the best one.
    
    Uses displacement (Keyence) as x and error as y.
    
    Parameters:
    -----------
    df_merged : pandas.DataFrame
        The dataframe with segment columns
    segment_info : list of dict
        Information about each segment
    segment_num : int
        The segment number to plot (1-indexed)
    version : int
        Version number for the plot title and filename
    max_degree : int
        Maximum polynomial degree to test (default: 10)
    use_degree : int or None
        Specific degree to use. If None, uses the degree with best R² (default: None)
    
    Returns:
    --------
    selected_degree : int
        The polynomial degree used
    r2_value : float
        The R² score for the selected degree
    coefficients : array
        Polynomial coefficients (highest degree first)
    """
    
    # Find the segment info
    segment = None
    for seg in segment_info:
        if seg['segment_num'] == segment_num:
            segment = seg
            break
    
    if segment is None:
        print(f"Error: Segment {segment_num} not found!")
        return None, None, None
    
    # Get the mask for this segment
    segment_mask = df_merged[segment['error_col']].notna()
    
    # Extract data - using Keyence displacement as x
    x_data = df_merged.loc[segment_mask, segment['keyence_col']].values
    y_error = df_merged.loc[segment_mask, segment['error_col']].values
    time_data = df_merged.loc[segment_mask, segment['time_col']].values
    y_magnetic = df_merged.loc[segment_mask, segment['magnetic_col']].values
    
    # Test different polynomial degrees
    r2_scores = []
    coefficients = []
    
    print(f"\nTesting polynomial fits for Segment {segment_num}:")
    print(f"{'Degree':<10} {'R² Score':<15}")
    print("-" * 25)
    
    for degree in range(1, max_degree + 1):
        # Fit polynomial
        poly_coeffs = np.polyfit(x_data, y_error, degree)
        poly_func = np.poly1d(poly_coeffs)
        y_fit = poly_func(x_data)
        
        # Calculate R² score
        ss_res = np.sum((y_error - y_fit) ** 2)
        ss_tot = np.sum((y_error - np.mean(y_error)) ** 2)
        r2 = 1 - (ss_res / ss_tot)
        
        r2_scores.append(r2)
        coefficients.append(poly_coeffs)
        
        print(f"{degree:<10} {r2:<15.6f}")
    
    # Determine which degree to use
    if use_degree is None:
        # Use best degree
        selected_degree = np.argmax(r2_scores) + 1
        print(f"\nAuto-selected best fit: Degree {selected_degree}")
    else:
        # Use specified degree
        if use_degree < 1 or use_degree > max_degree:
            print(f"Error: use_degree must be between 1 and {max_degree}")
            return None, None, None
        selected_degree = use_degree
        print(f"\nUsing specified degree: {selected_degree}")
    
    selected_r2 = r2_scores[selected_degree - 1]
    selected_coeffs = coefficients[selected_degree - 1]
    
    print(f"R² = {selected_r2:.6f}")
    
    # Create the polynomial function with selected coefficients
    selected_poly = np.poly1d(selected_coeffs)
    y_selected_fit = selected_poly(x_data)
    
    # Create figure with 3 subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 14))
    
    # Plot 1: Error vs Displacement with selected fit
    ax1.scatter(x_data, y_error, s=20, color='red', alpha=0.5, label='Error (Keyence - Magnetic)')
    # Sort x_data for smooth curve plotting
    sort_idx = np.argsort(x_data)
    ax1.plot(x_data[sort_idx], y_selected_fit[sort_idx], linewidth=2, color='blue', linestyle='--', 
             label=f'Polynomial Fit (degree={selected_degree}, R²={selected_r2:.4f})')
    
    ax1.set_xlabel('Displacement - Keyence (mm)', fontsize=12)
    ax1.set_ylabel('Error (mm)', fontsize=12)
    ax1.set_title(f'Segment {segment_num} - Error vs Displacement with Degree {selected_degree} Polynomial Fit - Version {version}', fontsize=14)
    ax1.legend(fontsize=10, loc='best')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Keyence vs Magnetic over time
    ax2.plot(time_data, x_data, linewidth=2, color='blue', linestyle='-', label='Keyence')
    ax2.plot(time_data, y_magnetic, linewidth=2, color='green', linestyle='--', label='Magnetic (filtered)')
    
    ax2.set_xlabel('Normalized Time (s)', fontsize=12)
    ax2.set_ylabel('Displacement (mm)', fontsize=12)
    ax2.set_title(f'Segment {segment_num} - Keyence vs Magnetic vs Time - Version {version}', fontsize=14)
    ax2.legend(fontsize=10, loc='best')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: R² scores for different degrees
    degrees = list(range(1, max_degree + 1))
    ax3.plot(degrees, r2_scores, marker='o', linewidth=2, markersize=8, color='purple')
    ax3.axvline(x=selected_degree, color='red', linestyle='--', linewidth=2, 
                label=f'Selected degree: {selected_degree}')
    ax3.axhline(y=selected_r2, color='red', linestyle=':', linewidth=1, alpha=0.5)
    
    ax3.set_xlabel('Polynomial Degree', fontsize=12)
    ax3.set_ylabel('R² Score', fontsize=12)
    ax3.set_title(f'R² Scores vs Polynomial Degree - Version {version}', fontsize=14)
    ax3.legend(fontsize=10, loc='best')
    ax3.grid(True, alpha=0.3)
    ax3.set_xticks(degrees)
    
    plt.tight_layout()
    
    # Save the figure
    output_path = os.path.join(graphs_dir, f'segment_{segment_num}_v{version}_degree{selected_degree}_fit_vs_displacement.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\nSegment {segment_num} polynomial fit plot saved to: {output_path}")
    
    # Print polynomial equation
    print(f"\nPolynomial equation (degree {selected_degree}):")
    print(f"error = ", end="")
    for i, coeff in enumerate(selected_coeffs):
        power = selected_degree - i
        if i > 0:
            print(f" + " if coeff >= 0 else f" - ", end="")
            print(f"{abs(coeff):.6e}*displacement^{power}", end="")
        else:
            print(f"{coeff:.6e}*displacement^{power}", end="")
    print("\n")
    
    plt.show()
    
    return selected_degree, selected_r2, selected_coeffs

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
df_arduino['posic_encoder_mm'] = df_arduino['posic_encoder_mm'] / 1000 * (-1) * 0.01953125 * 0.9392363058

# Create time_s column by calculating cumulative time differences in seconds
df_arduino['time_s'] = (df_arduino['time_ms'] - df_arduino['time_ms'].iloc[0]) / 1000.0

# df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] * 0.00048828125 * 0.9406912098
# df_arduino['magnetic_filtered_mm'] = df_arduino['magnetic_filtered_mm'] * 0.00048828125 * 0.9335003416
df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] * 0.00048828125 * 1.694
df_arduino['magnetic_filtered_mm'] = df_arduino['magnetic_filtered_mm'] * 0.00048828125 * 1.694

# Zero all displacement values
df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] - df_arduino['magnetic_raw_mm'].iloc[0]
df_arduino['magnetic_filtered_mm'] = df_arduino['magnetic_filtered_mm'] - df_arduino['magnetic_filtered_mm'].iloc[0]
df_arduino['posic_encoder_mm'] = df_arduino['posic_encoder_mm'] - df_arduino['posic_encoder_mm'].iloc[0]
# Shift Arduino raw magnetic data vertically
df_arduino['magnetic_raw_mm'] = df_arduino['magnetic_raw_mm'] + displacement_shift

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

# Shift Keyence time by time_shift seconds
df_keyence['time_s'] = df_keyence['time_s'] + time_shift

# Zero the Keyence displacement values
df_keyence['displacement_mm'] = df_keyence['displacement_mm'] - df_keyence['displacement_mm'].iloc[0]

# ============================================================================
# APPLY LOW-PASS FILTER TO MAGNETIC_RAW AND POSIC_ENCODER DATA
# ============================================================================

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

# Apply filter to magnetic_raw data
y = df_arduino['magnetic_raw_mm'].values
yfilt = np.zeros(len(y))

# Apply the filter equation: yfilt[i] = a[1]*yfilt[i-1] + b[0]*y[i] + b[1]*y[i-1]
for i in range(1, len(y)):
    yfilt[i] = a[1]*yfilt[i-1] + b[0]*y[i] + b[1]*y[i-1]

# Add filtered data to dataframe
df_arduino['magnetic_raw_filtered'] = yfilt

# Apply filter to posic_encoder data
y_posic = df_arduino['posic_encoder_mm'].values
yfilt_posic = np.zeros(len(y_posic))

# Apply the filter equation: yfilt[i] = a[1]*yfilt[i-1] + b[0]*y[i] + b[1]*y[i-1]
for i in range(1, len(y_posic)):
    yfilt_posic[i] = a[1]*yfilt_posic[i-1] + b[0]*y_posic[i] + b[1]*y_posic[i-1]

# Add filtered data to dataframe
df_arduino['posic_encoder_filtered'] = yfilt_posic

# ============================================================================
# MERGE DATAFRAMES ON TIME COLUMN
# ============================================================================
# Merge df_keyence into df_arduino based on time_s
df_merged = pd.merge(df_arduino, df_keyence, on='time_s', how='outer', suffixes=('', '_keyence'))
df_merged = df_merged.sort_values('time_s').reset_index(drop=True)

# Create time-shifted columns for filtered data
df_merged['magnetic_raw_filtered_time'] = df_merged['time_s'] + time_shift_filtered
df_merged['posic_encoder_filtered_time'] = df_merged['time_s'] + time_shift_filtered

# ============================================================================
# FIND TURNING POINTS FOR MAGNETIC_RAW_FILTERED DATA
# ============================================================================
# Use only rows where magnetic_raw_filtered is not NaN
valid_mask = df_merged['magnetic_raw_filtered'].notna()
magnetic_data = df_merged.loc[valid_mask, 'displacement_mm'].values
# magnetic_data = df_merged.loc[valid_mask, 'magnetic_raw_filtered'].values
time_data = df_merged.loc[valid_mask, 'time_s'].values

df_merged["magnetic_error"] = df_merged["displacement_mm"] - df_merged["magnetic_raw_filtered"]
df_merged["magnetic_linearized"] =  df_merged["magnetic_raw_filtered"] + df_merged["magnetic_error"]

# Find local minima and maxima
local_min_indices = argrelextrema(magnetic_data, np.less, order=5)[0]
local_max_indices = argrelextrema(magnetic_data, np.greater, order=5)[0]

min_times = time_data[local_min_indices] # List of local mins
min_values = magnetic_data[local_min_indices]
max_times = time_data[local_max_indices]
max_values = magnetic_data[local_max_indices] # List of local maxes

print(f"Found {len(local_min_indices)} local minima and {len(local_max_indices)} local maxima")
print(local_min_indices)

# ============================================================================
# CREATE THE PLOTS
# ============================================================================

# After you've calculated min_times and max_times, add:
df_merged, segment_info = create_extrema_segments(df_merged, min_times, max_times)

# # Plot the overlapping segments
plot_overlapping_segments(df_merged, segment_info, version)

# Plot a single segment (e.g., segment 1)
plot_single_segment(df_merged, segment_info, segment_num=1, version=version)

# Option 2: Specify a particular degree (e.g., degree 3)
degree, r2, coeffs = plot_segment_with_polynomial_fit(
    df_merged, segment_info, segment_num=5, version=version, max_degree=20, use_degree=10
)

print(coeffs)

# ============================================================================
# EXPORT COMBINED DATAFRAME TO CSV
# ============================================================================
# Select columns for export
df_export = df_merged[['time_s', 'magnetic_raw_mm', 'magnetic_raw_filtered', 
                        'magnetic_filtered_mm', 'posic_encoder_mm', 
                        'posic_encoder_filtered', 'displacement_mm']].copy()

# Rename keyence column for clarity
df_export = df_export.rename(columns={'displacement_mm': 'keyence_mm'})

# Export to CSV
csv_output_path = os.path.join(script_dir, f'combined_sensor_data_v{version}_filtered.csv')
df_export.to_csv(csv_output_path, index=False)
print(f"\nCombined data exported to: {csv_output_path}")
print(f"Columns: {list(df_export.columns)}")
print(f"Total rows: {len(df_export)}")