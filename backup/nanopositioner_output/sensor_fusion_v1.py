import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.signal import argrelextrema
from scipy import signal

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
graphs_dir = os.path.join(script_dir, 'graphs')
os.makedirs(graphs_dir, exist_ok=True)

fig, ax = plt.subplots(figsize=(12, 6))
first_flag = True
reference_val = 0
comparison_val = 0
shift_factor = 0

for version in range(29, 32):
    csv_input_path = os.path.join(script_dir, f'combined_sensor_data_v{version}_filtered.csv')
    
    if not os.path.exists(csv_input_path):
        print(f"Warning: File not found for version {version}, skipping...")
        continue
    
    df_merged = pd.read_csv(csv_input_path)

    if first_flag:
        first_flag = False
        reference_val = df_merged["magnetic_raw"][0]

    comparison_val = df_merged["magnetic_raw"][0]
    shift_factor = (reference_val - comparison_val) * 0.00048828125 * 0.9459754389
    # df_merged['posic_lut_calibrated'] = df_merged['posic_lut_calibrated'] - shift_factor

    ax.plot(df_merged['time_s'], df_merged['posic_lut_calibrated'], label=f'v{version}')


ax.set_xlabel('Time (s)')
ax.set_ylabel('Position Encoder (Filtered)')
ax.set_title('Filtered Position Encoder vs Time (Versions 29-31)')
ax.legend()
ax.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(graphs_dir, 'position_encoder_v29_v31.png'), dpi=150)
plt.show()