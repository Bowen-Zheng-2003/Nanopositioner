import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.signal import argrelextrema
from scipy import signal

#### CONTROLS
version = 31
# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
graphs_dir = os.path.join(script_dir, 'graphs')

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
    if 'keyence_mm' in df_arduino.columns:
        keyence_mask = df_arduino['keyence_mm'].notna()
        
        ax1.scatter(time_data[keyence_mask], df_arduino.loc[keyence_mask, 'keyence_mm'], 
                   label='Keyence', s=10, alpha=0.5, color='orange')
        ax1.scatter(time_data, calibrated_values, 
                   label='Posic (LUT)', s=10, alpha=0.5, color='red')
        # ax1.scatter(time_data, df_arduino["magnetic_raw_mm"], 
        #            label='Magnetic', s=10, alpha=0.5, color='blue')
        ax1.scatter(time_data, df_arduino["magnetic_raw_filtered"], 
                   label='Magnetic (LP Filter)', s=10, alpha=0.5, color='green')
        
        # Calculate error between LUT calibrated and Keyence
        error = calibrated_values[keyence_mask] - df_arduino.loc[keyence_mask, 'keyence_mm'].values
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
                ha='center', va='center', transform=ax1.transAxes, fontsize=12)
        ax2.text(0.5, 0.5, 'Keyence data not available for error plot', 
                ha='center', va='center', transform=ax2.transAxes, fontsize=12)
    
    plt.tight_layout()
    
    # Save the figure
    output_path = os.path.join(graphs_dir, f'lut_calibration_comparison_v{version}.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"LUT calibration comparison plot saved to: {output_path}")
    
    plt.show()

# ============================================================================
# IMPORT COMBINED SENSOR DATA FROM CSV
# ============================================================================
csv_input_path = os.path.join(script_dir, f'combined_sensor_data_v{version}_filtered.csv')
df_merged = pd.read_csv(csv_input_path)

# ============================================================================
# APPLY LOOKUP TABLE CALIBRATION
# ============================================================================
# Specify lookup table parameters
N = 12  # Must match the N used when creating the lookup table
lookup_table_path = os.path.join(script_dir, f'lookup_table_v{version}_N{N}.txt')
# lookup_table_path = os.path.join(script_dir, f'lookup_table_v26_N{N}.txt')
# lookup_table_path = os.path.join(script_dir, f'lookup_table_v{version}_N{N}.txt')

# Apply the lookup table to the magnetic filtered data
df_merged['posic_lut_calibrated'] = apply_lookup_table(
    df_merged['posic_encoder_filtered'],
    lookup_table_path,
    N
)

# Plot the comparison
plot_lut_calibration_comparison(df_merged, df_merged['posic_lut_calibrated'].values, version)
# plot_lut_calibration_comparison(df_merged, df_merged['magnetic_raw_filtered'].values, version)

df_merged.to_csv(csv_input_path, index=False)
print(f"Saved updated data to: {csv_input_path}")