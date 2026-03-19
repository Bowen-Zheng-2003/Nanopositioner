import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

version = 21

# Import the combined CSV file
csv_file_path = os.path.join(script_dir, f'combined_sensor_data_v{version}.csv')
df = pd.read_csv(csv_file_path)

print("Combined Sensor Data:")
print(df.head(10))
print(f"\nTotal rows: {len(df)}")
print(f"Columns: {list(df.columns)}")

# Calculate error between magnetic raw and keyence
# Only where both sensors have valid data (not NaN)
df_valid = df.dropna(subset=['magnetic_raw_mm', 'keyence_mm'])
df_valid['error_mm'] = df_valid['magnetic_raw_mm'] - df_valid['keyence_mm']

# Calculate statistics
mean_error = df_valid['error_mm'].mean()
std_error = df_valid['error_mm'].std()
max_error = df_valid['error_mm'].max()
min_error = df_valid['error_mm'].min()

print("\n" + "="*80)
print("ERROR STATISTICS (Magnetic Raw - Keyence)")
print("="*80)
print(f"Mean error: {mean_error:.6f} mm")
print(f"Std deviation: {std_error:.6f} mm")
print(f"Max error: {max_error:.6f} mm")
print(f"Min error: {min_error:.6f} mm")
print(f"Valid data points: {len(df_valid)}")
print("="*80 + "\n")

# Create the error plot
plt.figure(figsize=(14, 8))

# Plot the error
plt.plot(df_valid['time_s'], df_valid['error_mm'], 
         label='Error (Magnetic Raw - Keyence)', linewidth=1.5, alpha=0.8, color='red')

# Add a horizontal line at zero for reference
plt.axhline(y=0, color='black', linestyle='--', linewidth=1, alpha=0.5, label='Zero Error')

# Add horizontal lines for mean error
plt.axhline(y=mean_error, color='blue', linestyle='--', linewidth=1, alpha=0.7, 
            label=f'Mean Error ({mean_error:.4f} mm)')

# Add labels and title
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Error (mm)', fontsize=12)
plt.title(f'Error Between Magnetic Sensor (Raw) and Keyence (Version {version})', fontsize=14)
plt.legend(fontsize=10)
plt.grid(True, alpha=0.3)

# Tight layout for better spacing
plt.tight_layout()

# Save the figure
graphs_dir = os.path.join(script_dir, 'graphs')
os.makedirs(graphs_dir, exist_ok=True)
output_path = os.path.join(graphs_dir, f'error_plot_v{version}.png')
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"Error plot saved to: {output_path}")

# Display the plot
plt.show()

# FFT Analysis
print("\n" + "="*80)
print("FFT ANALYSIS")
print("="*80)

# Calculate sampling rate from time data
dt = df_valid['time_s'].diff().mean()  # Average time step
sampling_rate = 1 / dt  # Hz
print(f"Sampling rate: {sampling_rate:.2f} Hz")
print(f"Time step: {dt:.6f} s")

# Perform FFT on the error signal
error_signal = df_valid['error_mm'].values
N = len(error_signal)

# Apply FFT
fft_result = np.fft.fft(error_signal)
fft_freq = np.fft.fftfreq(N, dt)

# Get magnitude (only positive frequencies)
positive_freq_idx = fft_freq > 0
frequencies = fft_freq[positive_freq_idx]
magnitude = np.abs(fft_result[positive_freq_idx]) * 2 / N  # Normalize

# Find dominant frequency
dominant_idx = np.argmax(magnitude)
dominant_freq = frequencies[dominant_idx]
dominant_magnitude = magnitude[dominant_idx]

print(f"\nDominant frequency: {dominant_freq:.4f} Hz")
print(f"Dominant magnitude: {dominant_magnitude:.6f} mm")
print("="*80 + "\n")

# Create FFT plot
plt.figure(figsize=(14, 8))

plt.plot(frequencies, magnitude, linewidth=1.5, alpha=0.8, color='purple')
plt.axvline(x=dominant_freq, color='red', linestyle='--', linewidth=1.5, alpha=0.7,
            label=f'Dominant Frequency: {dominant_freq:.4f} Hz')

plt.xlabel('Frequency (Hz)', fontsize=12)
plt.ylabel('Magnitude (mm)', fontsize=12)
plt.title(f'FFT of Error Signal (Magnetic Raw - Keyence) (Version {version})', fontsize=14)
plt.legend(fontsize=10)
plt.grid(True, alpha=0.3)
plt.xlim(0, sampling_rate / 2)  # Show up to Nyquist frequency

# Tight layout
plt.tight_layout()

# Save the FFT figure
fft_output_path = os.path.join(graphs_dir, f'error_fft_v{version}.png')
plt.savefig(fft_output_path, dpi=300, bbox_inches='tight')
print(f"FFT plot saved to: {fft_output_path}")

# Display the plot
plt.show()


# Packages and adjustments to the figures
from scipy import signal
import matplotlib.pyplot as plt
import numpy as np
import math

samplingFreq = 1000; # sampled at 1 kHz = 1000 samples / second
signalFreq = [2,50]; # Cycles / second

# # Butterworth filter
# wc = 2*np.pi*5; # cutoff frequency (rad/s)
# n = 2; # Filter order

# # Compute the Butterworth filter coefficents
# a = np.zeros(n+1)
# gamma = np.pi/(2.0*n)
# a[0] = 1; # first coef is always 1
# for k in range(0,n):
#     rfac = np.cos(k*gamma)/np.sin((k+1)*gamma)
#     a[k+1] = rfac*a[k]; # Other coefficients by recursion

# print("Butterworth polynomial coefficients a_i:                " + str(a))

# # Adjust the cutoff frequency
# c = np.zeros(n+1)
# for k in range(0,n+1):
#     c[n-k] = a[k]/pow(wc,k)

# print("Butterworth coefficients with frequency adjustment c_i: " + str(c))

# # Low-pass filter
# w0 = 2*np.pi*5; # pole frequency (rad/s)
# num = [1];      # transfer function numerator coefficients
# den = c;        # transfer function denominator coefficients
# lowPass = signal.TransferFunction(num,den) # Transfer function

# # Generate the bode plot
# w = np.logspace( np.log10(min(signalFreq)*2*np.pi/10), np.log10(max(signalFreq)*2*np.pi*10), 500 )
# w, mag, phase = signal.bode(lowPass,w)

# # Magnitude plot
# plt.figure()
# plt.semilogx(w, mag)
# for sf in signalFreq:
#     plt.semilogx([sf*2*np.pi,sf*2*np.pi],[min(mag),max(mag)],'k:')
# plt.ylabel("Magnitude (dB)")
# plt.xlim([min(w),max(w)])
# plt.ylim([min(mag),max(mag)])

# # Phase plot
# plt.figure()
# plt.semilogx(w, phase)  # Bode phase plot
# plt.ylabel("Phase")
# plt.xlabel("w (rad/s)")
# plt.xlim([min(w),max(w)])
# plt.show()

# # Compute the discrete low pass with delta_t = 1/samplingFrequency
# dt = 1.0/samplingFreq
# discreteLowPass = lowPass.to_discrete(dt,method='gbt',alpha=0.5)
# print(discreteLowPass)

# # The coefficients from the discrete form of the filter transfer function (but with a negative sign)
# b = discreteLowPass.num
# a = -discreteLowPass.den
# print("Filter coefficients b_i: " + str(b))
# print("Filter coefficients a_i: " + str(a[1:]))

# # # Filter the signal
# # Nb = len(b)
# # yfilt = np.zeros(len(y));
# # for m in range(3,len(y)):
# #     yfilt[m] = b[0]*y[m];
# #     for i in range(1,Nb):
# #         yfilt[m] += a[i]*yfilt[m-i] + b[i]*y[m-i];

# Low-pass filter
w0 = 2*np.pi*5; # pole frequency (rad/s)
num = w0        # transfer function numerator coefficients
den = [1,w0]    # transfer function denominator coefficients
lowPass = signal.TransferFunction(num,den) # Transfer function

# Generate the bode plot
w = np.logspace( np.log10(min(signalFreq)*2*np.pi/10), np.log10(max(signalFreq)*2*np.pi*10), 500 )
w, mag, phase = signal.bode(lowPass,w)

# Magnitude plot
plt.figure()
plt.semilogx(w, mag)
for sf in signalFreq:
    plt.semilogx([sf*2*np.pi,sf*2*np.pi],[min(mag),max(mag)],'k:')
plt.ylabel("Magnitude (dB)")
plt.xlim([min(w),max(w)])
plt.ylim([min(mag),max(mag)])

# Phase plot
plt.figure()
plt.semilogx(w, phase)  # Bode phase plot
plt.ylabel("Phase")
plt.xlabel("w (rad/s)")
plt.xlim([min(w),max(w)])
plt.show()

dt = 1.0/samplingFreq
discreteLowPass = lowPass.to_discrete(dt,method='gbt',alpha=0.5)
print(discreteLowPass)

# The coefficients from the discrete form of the filter transfer function (but with a negative sign)
b = discreteLowPass.num
a = -discreteLowPass.den
print("Filter coefficients b_i: " + str(b))
print("Filter coefficients a_i: " + str(a[1:]))

# # Filter the signal
# yfilt = np.zeros(len(y));
# for i in range(3,len(y)):
#     yfilt[i] = a[1]*yfilt[i-1] + b[0]*y[i] + b[1]*y[i-1];