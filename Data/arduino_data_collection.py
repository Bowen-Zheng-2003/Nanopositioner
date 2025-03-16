import pandas as pd
import matplotlib.pyplot as plt

# File paths
file1 = 'settling_time_PID_control.csv'
file2 = 'settling_time_continuous_control.csv'

# Function to read and process CSV data
def read_csv_data(filename):
    try:
        data = pd.read_csv(filename)
        if 'Time(s)' not in data.columns or 'Value' not in data.columns:
            print(f"Error: CSV file '{filename}' must contain 'Time(s)' and 'Value' columns.")
            return None
        return data
    except FileNotFoundError:
        print(f"Error: The file '{filename}' was not found.")
        return None
    except Exception as e:
        print(f"Error reading CSV file '{filename}': {e}")
        return None

# Read both CSV files
data1 = read_csv_data(file1)
data2 = read_csv_data(file2)

# Check if data was loaded successfully
if data1 is None or data2 is None:
    exit()

# Extract time and displacement for both datasets
time1 = data1['Time(s)']
displacement1 = data1['Value']
time2 = data2['Time(s)']
displacement2 = data2['Value']

# Convert displacement to microns for both datasets
displacement_microns1 = displacement1 * 1.949 / 1000
displacement_microns2 = displacement2 * 1.949 / 1000

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(time1, displacement_microns1, 'r-', label='PID Control')
plt.plot(time2, displacement_microns2, 'g-', label='Continuous Control')

# Add blue reference lines at 1, 2, 3, 4, and 5 mm
for level in [1, 2, 3, 4, 5]:
    plt.axhline(y=level, color='b', linestyle='--', alpha=0.7, label='Reference' if level == 1 else None)

plt.xlabel('Time (s)')
plt.ylabel('Displacement (mm)')
plt.title('Settling Time')
plt.grid(True)
plt.legend()

# Show the plot
plt.show()
