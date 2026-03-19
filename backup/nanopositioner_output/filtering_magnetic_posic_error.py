import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# Configuration
version = 15
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_file_path = os.path.join(script_dir, f'combined_sensor_data_v{version}_filtered.csv')
graphs_dir = os.path.join(script_dir, 'graphs')

# Read the combined CSV file
df = pd.read_csv(csv_file_path)

print(f"Loaded data from: {csv_file_path}")
print(f"Columns: {list(df.columns)}")
print(f"Total rows: {len(df)}")

# Filter to only rows where Keyence data exists (not NaN)
df_valid = df[df['keyence_mm'].notna()].copy()

print(f"\nRows with valid Keyence data: {len(df_valid)}")

# Calculate errors (Magnetic - Keyence)
df_valid['error_raw'] = df_valid['magnetic_raw_mm'] - df_valid['keyence_mm']
df_valid['error_filtered'] = df_valid['magnetic_raw_filtered'] - df_valid['keyence_mm']

# Calculate error statistics
print("\n" + "="*80)
print("ERROR STATISTICS")
print("="*80)

print("\nMagnetic Raw vs Keyence:")
print(f"  Mean error: {df_valid['error_raw'].mean():.6f} mm")
print(f"  Std dev: {df_valid['error_raw'].std():.6f} mm")
print(f"  RMS error: {np.sqrt(np.mean(df_valid['error_raw']**2)):.6f} mm")
print(f"  Max error: {df_valid['error_raw'].max():.6f} mm")
print(f"  Min error: {df_valid['error_raw'].min():.6f} mm")

print("\nMagnetic Filtered vs Keyence:")
print(f"  Mean error: {df_valid['error_filtered'].mean():.6f} mm")
print(f"  Std dev: {df_valid['error_filtered'].std():.6f} mm")
print(f"  RMS error: {np.sqrt(np.mean(df_valid['error_filtered']**2)):.6f} mm")
print(f"  Max error: {df_valid['error_filtered'].max():.6f} mm")
print(f"  Min error: {df_valid['error_filtered'].min():.6f} mm")

print("="*80 + "\n")

# Create the error plot
plt.figure(figsize=(14, 8))

# Plot both errors
plt.plot(df_valid['time_s'], df_valid['error_raw'], 
         label='Error: Magnetic Raw - Keyence', linewidth=1.5, alpha=0.7, color='blue')
plt.plot(df_valid['time_s'], df_valid['error_filtered'], 
         label='Error: Magnetic Filtered - Keyence', linewidth=2, alpha=0.8, color='red')

# Add a zero reference line
plt.axhline(y=0, color='black', linestyle='--', linewidth=1, alpha=0.5, label='Zero Error')

# Add labels and title
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Error (mm)', fontsize=12)
plt.title(f'Measurement Error Comparison: Magnetic Sensor vs Keyence (Version {version})', fontsize=14)
plt.legend(fontsize=10, loc='best')
plt.grid(True, alpha=0.3)
plt.tight_layout()

# Save the figure
os.makedirs(graphs_dir, exist_ok=True)
output_path = os.path.join(graphs_dir, f'error_comparison_v{version}.png')
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"Error plot saved to: {output_path}")

# Display the plot
plt.show()

# Export error data to CSV
error_output_path = os.path.join(script_dir, f'error_analysis_v{version}.csv')
df_valid[['time_s', 'keyence_mm', 'magnetic_raw_mm', 'magnetic_raw_filtered', 
          'error_raw', 'error_filtered']].to_csv(error_output_path, index=False)
print(f"\nError analysis data exported to: {error_output_path}")

# ============================================================================
# FFT ANALYSIS OF ERRORS
# ============================================================================
print("\n" + "="*80)
print("FFT ANALYSIS OF ERRORS")
print("="*80)

# Calculate sampling rate from time data
time_diff = np.diff(df_valid['time_s'].values)
sampling_rate = 1.0 / np.mean(time_diff)
print(f"Average sampling rate: {sampling_rate:.2f} Hz")

# Perform FFT on error_raw
error_raw_values = df_valid['error_raw'].values
N_raw = len(error_raw_values)
fft_raw = np.fft.fft(error_raw_values)
freq_raw = np.fft.fftfreq(N_raw, d=1.0/sampling_rate)

# Take only positive frequencies
positive_freq_mask_raw = freq_raw > 0
freq_raw_pos = freq_raw[positive_freq_mask_raw]
fft_raw_magnitude = np.abs(fft_raw[positive_freq_mask_raw])

# Perform FFT on error_filtered
error_filtered_values = df_valid['error_filtered'].values
N_filtered = len(error_filtered_values)
fft_filtered = np.fft.fft(error_filtered_values)
freq_filtered = np.fft.fftfreq(N_filtered, d=1.0/sampling_rate)

# Take only positive frequencies
positive_freq_mask_filtered = freq_filtered > 0
freq_filtered_pos = freq_filtered[positive_freq_mask_filtered]
fft_filtered_magnitude = np.abs(fft_filtered[positive_freq_mask_filtered])

print(f"FFT of error_raw: {N_raw} points")
print(f"FFT of error_filtered: {N_filtered} points")
print("="*80 + "\n")

# Create FFT plot
plt.figure(figsize=(14, 8))

plt.plot(freq_raw_pos, fft_raw_magnitude, 
         label='FFT of Error: Magnetic Raw - Keyence', linewidth=1.5, alpha=0.7, color='blue')
plt.plot(freq_filtered_pos, fft_filtered_magnitude, 
         label='FFT of Error: Magnetic Filtered - Keyence', linewidth=2, alpha=0.8, color='red')

plt.xlabel('Frequency (Hz)', fontsize=12)
plt.ylabel('Magnitude', fontsize=12)
plt.title(f'FFT of Measurement Errors (Version {version})', fontsize=14)
plt.legend(fontsize=10, loc='best')
plt.grid(True, alpha=0.3)
plt.xlim(0, sampling_rate/2)  # Show up to Nyquist frequency
plt.tight_layout()

# Save FFT plot
fft_output_path = os.path.join(graphs_dir, f'fft_error_comparison_v{version}.png')
plt.savefig(fft_output_path, dpi=300, bbox_inches='tight')
print(f"FFT plot saved to: {fft_output_path}")

plt.show()