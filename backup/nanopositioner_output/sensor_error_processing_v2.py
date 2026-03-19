import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy import signal

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

version = 18
graphs_dir = os.path.join(script_dir, 'graphs')

# Build the full path to the processed CSV file
processed_file_path = os.path.join(script_dir, f'processed_data{version}.csv')

# Read the processed data CSV
df_combined = pd.read_csv(processed_file_path)

print("Combined DataFrame loaded:")
print(f"Shape: {df_combined.shape}")
print(f"\nColumn names: {df_combined.columns.tolist()}")

# Remove any rows with NaN values
df_combined = df_combined.dropna()

print(f"\n\nCleaned DataFrame:")
print(f"Shape: {df_combined.shape}")
print(f"Time range: {df_combined['keyence_time_s'].min():.4f} to {df_combined['keyence_time_s'].max():.4f} s")
print(f"\nFirst few rows:")
print(df_combined.head())
print(f"\nLast few rows:")
print(df_combined.tail())

# Calculate error (Keyence - Arduino) for both raw and filtered
df_combined['error_raw'] = df_combined['keyence_displacement_mm'] - df_combined['arduino_displacement_mm_raw']
df_combined['error_filtered'] = df_combined['keyence_displacement_mm'] - df_combined['arduino_displacement_mm_filtered']

print(f"\n\nError Statistics (Raw):")
print(f"Mean error: {df_combined['error_raw'].mean():.4f} mm")
print(f"Std deviation: {df_combined['error_raw'].std():.4f} mm")
print(f"Max error: {df_combined['error_raw'].max():.4f} mm")
print(f"Min error: {df_combined['error_raw'].min():.4f} mm")

print(f"\n\nError Statistics (Filtered):")
print(f"Mean error: {df_combined['error_filtered'].mean():.4f} mm")
print(f"Std deviation: {df_combined['error_filtered'].std():.4f} mm")
print(f"Max error: {df_combined['error_filtered'].max():.4f} mm")
print(f"Min error: {df_combined['error_filtered'].min():.4f} mm")

# Create first figure with three subplots (Time domain)
fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 14))
fig1.canvas.manager.set_window_title('Time Domain Analysis')

# Plot 1: Keyence and Arduino sensors with dual y-axis
color_keyence = 'tab:blue'
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Keyence Displacement (mm)', color=color_keyence)
line1 = ax1.plot(df_combined['keyence_time_s'], df_combined['keyence_displacement_mm'], 
                 color=color_keyence, alpha=0.7, linewidth=1.5, label='Keyence')
ax1.tick_params(axis='y', labelcolor=color_keyence)
ax1.grid(True, alpha=0.3)

# Create second y-axis for Arduino
ax1_twin = ax1.twinx()
color_arduino_filtered = 'tab:orange'
color_arduino_raw = 'lightcoral'
line2 = ax1_twin.plot(df_combined['arduino_time_s'], df_combined['arduino_displacement_mm_filtered'], 
                      color=color_arduino_filtered, alpha=0.7, linewidth=1.5, label='Arduino (Filtered)')
line3 = ax1_twin.plot(df_combined['arduino_time_s'], df_combined['arduino_displacement_mm_raw'], 
                      color=color_arduino_raw, alpha=0.5, linewidth=1.0, linestyle='--', label='Arduino (Raw)')
ax1_twin.set_ylabel('Arduino Displacement (mm)', color=color_arduino_filtered)
ax1_twin.tick_params(axis='y', labelcolor=color_arduino_filtered)

# Add title and legend
ax1.set_title('Keyence and Arduino Sensor Measurements vs Time (Dual Y-Axis)')
lines = line1 + line2 + line3
labels = [l.get_label() for l in lines]
ax1.legend(lines, labels, loc='upper left')

# Plot 2: Error for Filtered
ax2.scatter(df_combined['keyence_time_s'], df_combined['error_filtered'], 
           s=10, alpha=0.6, label='Error (Filtered)', color='tab:green')
ax2.axhline(y=0, color='r', linestyle='--', linewidth=1.5, label='Zero Error')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error (Keyence - Arduino Filtered) (mm)')
ax2.set_title('Error Between Keyence and Arduino (Filtered) Sensors vs Time')
ax2.legend()
ax2.grid(True, alpha=0.3)

# Plot 3: Error for Raw
ax3.scatter(df_combined['keyence_time_s'], df_combined['error_raw'], 
           s=10, alpha=0.6, label='Error (Raw)', color='tab:purple')
ax3.axhline(y=0, color='r', linestyle='--', linewidth=1.5, label='Zero Error')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Error (Keyence - Arduino Raw) (mm)')
ax3.set_title('Error Between Keyence and Arduino (Raw) Sensors vs Time')
ax3.legend()
ax3.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(graphs_dir, f'error_sensor{version}.png'), dpi=300, bbox_inches='tight')

# Create second figure for FFT Analysis (with two subplots for raw and filtered)
fig2, (ax4, ax5) = plt.subplots(2, 1, figsize=(12, 10))
fig2.canvas.manager.set_window_title('FFT Analysis')

# Calculate sampling rate
time_diff = np.diff(df_combined['keyence_time_s'])
avg_dt = np.mean(time_diff)
sampling_rate = 1.0 / avg_dt
n = len(df_combined)
print(f"\n\nFFT Analysis:")
print(time_diff[:10])
print(f"Average sampling interval: {avg_dt:.6f} s")
print(f"Average sampling rate: {sampling_rate:.2f} Hz")

# FFT for Error (Filtered)
fhat_filtered = np.fft.fft(df_combined['error_filtered'], n)
psd_filtered = fhat_filtered * np.conj(fhat_filtered) / n
freq = (1 / (avg_dt * n)) * np.arange(n)
L = np.arange(1, np.floor(n/2), dtype='int')  # Fixed: get first half of spectrum, excluding DC

# Plot FFT for Filtered
ax4.plot(freq[L], psd_filtered[L], color='c', linewidth=1.5)
ax4.set_xlabel('Frequency (Hz)')
ax4.set_ylabel('Power Spectral Density')
ax4.set_title('FFT Analysis of Error Signal (Filtered)')
ax4.grid(True, alpha=0.3)
ax4.set_xlim([0, 1/(2*avg_dt)])  # Nyquist frequency

# FFT for Error (Raw)
fhat_raw = np.fft.fft(df_combined['error_raw'], n)
psd_raw = fhat_raw * np.conj(fhat_raw) / n

# Plot FFT for Raw
ax5.plot(freq[L], psd_raw[L], color='m', linewidth=1.5)
ax5.set_xlabel('Frequency (Hz)')
ax5.set_ylabel('Power Spectral Density')
ax5.set_title('FFT Analysis of Error Signal (Raw)')
ax5.grid(True, alpha=0.3)
ax5.set_xlim([0, 1/(2*avg_dt)])  # Nyquist frequency

plt.tight_layout()
plt.savefig(os.path.join(graphs_dir, f'fft_sensor{version}.png'), dpi=300, bbox_inches='tight')

# Create third figure comparing both errors on same plot
fig3, (ax6, ax7) = plt.subplots(2, 1, figsize=(12, 10))
fig3.canvas.manager.set_window_title('Error Comparison')

# Plot both errors together in time domain
ax6.scatter(df_combined['keyence_time_s'], df_combined['error_filtered'], 
           s=10, alpha=0.5, label='Error (Filtered)', color='tab:green')
ax6.scatter(df_combined['keyence_time_s'], df_combined['error_raw'], 
           s=10, alpha=0.5, label='Error (Raw)', color='tab:purple')
ax6.axhline(y=0, color='r', linestyle='--', linewidth=1.5, label='Zero Error')
ax6.set_xlabel('Time (s)')
ax6.set_ylabel('Error (mm)')
ax6.set_title('Comparison of Errors: Filtered vs Raw')
ax6.legend()
ax6.grid(True, alpha=0.3)

# Plot both FFTs together
ax7.plot(freq[L], psd_filtered[L], color='c', linewidth=1.5, label='FFT (Filtered)', alpha=0.7)
ax7.plot(freq[L], psd_raw[L], color='m', linewidth=1.5, label='FFT (Raw)', alpha=0.7)
ax7.set_xlabel('Frequency (Hz)')
ax7.set_ylabel('Power Spectral Density')
ax7.set_title('FFT Comparison: Filtered vs Raw Error Signals')
ax7.legend()
ax7.grid(True, alpha=0.3)
ax7.set_xlim([0, 1/(2*avg_dt)])  # Nyquist frequency

plt.tight_layout()
plt.savefig(os.path.join(graphs_dir, f'error_comparison{version}.png'), dpi=300, bbox_inches='tight')

# Show all figures
plt.show()