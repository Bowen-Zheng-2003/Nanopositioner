import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.signal import argrelextrema
from scipy import signal

#### CONTROLS ####
version = 29

def plot_raw_sensor_data(df_merged, version):
    """
    Plot Keyence, Magnetic (filtered), and Posic Encoder data against time on the same graph.
    
    Parameters:
    -----------
    df_merged : pandas.DataFrame
        The merged dataframe containing all sensor data
    version : int
        Version number for the plot title and filename
    """
    
    fig, ax = plt.subplots(1, 1, figsize=(16, 6))
    
    # Plot Keyence displacement
    valid_keyence = df_merged['keyence_mm'].notna()
    ax.plot(
        df_merged.loc[valid_keyence, 'time_s'],
        df_merged.loc[valid_keyence, 'keyence_mm'],
        linewidth=1.5,
        color='blue',
        label='Keyence',
        alpha=0.8
    )
    
    # Plot Magnetic filtered
    valid_magnetic = df_merged['magnetic_raw_mm'].notna()
    ax.plot(
        df_merged.loc[valid_magnetic, 'time_s'],
        df_merged.loc[valid_magnetic, 'magnetic_raw_mm'],
        linewidth=1.5,
        color='red',
        label='Magnetic',
        alpha=0.8
    )

    # # Plot Magnetic filtered
    # valid_magnetic = df_merged['magnetic_raw_filtered'].notna()
    # ax.plot(
    #     df_merged.loc[valid_magnetic, 'time_s'],
    #     df_merged.loc[valid_magnetic, 'magnetic_raw_filtered'],
    #     linewidth=1.5,
    #     color='red',
    #     label='Magnetic (filtered)',
    #     alpha=0.8
    # )
    
    # # Plot Posic Encoder filtered
    # valid_posic = df_merged['posic_encoder_mm'].notna()
    # ax.plot(
    #     df_merged.loc[valid_posic, 'time_s'],
    #     df_merged.loc[valid_posic, 'posic_encoder_mm'],
    #     linewidth=1.5,
    #     color='green',
    #     label='Posic Encoder',
    #     alpha=0.8
    # )

    # Plot Posic Encoder filtered
    valid_posic = df_merged['posic_encoder_filtered'].notna()
    ax.plot(
        df_merged.loc[valid_posic, 'time_s'],
        df_merged.loc[valid_posic, 'posic_encoder_filtered'],
        linewidth=1.5,
        color='green',
        label='Posic Encoder (LP Filter)',
        alpha=0.8
    )
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Displacement (mm)', fontsize=12)
    ax.set_title(f'Raw Sensor Data vs Time (Version {version})', fontsize=14)
    ax.legend(fontsize=10, loc='best')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    output_path = os.path.join(graphs_dir, f'raw_sensor_data_v{version}.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\nRaw sensor data plot saved to: {output_path}")
    
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
    # for i in range(len(all_extrema_times) - 1):
    for i in range(1, len(all_extrema_times) - 1):
        t_start = all_extrema_times[i]
        t_end = all_extrema_times[i + 1]
        
        # Create mask for this time segment
        segment_mask = (df_merged['time_s'] >= t_start) & (df_merged['time_s'] <= t_end)
        
        # Create column names for this segment
        segment_num = i + 1
        time_col = f'time_segment_{segment_num}'
        magnetic_col = f'magnetic_segment_{segment_num}'
        keyence_col = f'displacement{segment_num}'
        error_col = f'error_segment_{segment_num}'
        
        # Initialize columns with NaN
        df_merged[time_col] = np.nan
        df_merged[magnetic_col] = np.nan
        df_merged[keyence_col] = np.nan
        df_merged[error_col] = np.nan
        
        # Fill in the data for this segment with normalized time (starting at 0)
        df_merged.loc[segment_mask, time_col] = df_merged.loc[segment_mask, 'time_s'] - t_start
        df_merged.loc[segment_mask, magnetic_col] = df_merged.loc[segment_mask, 'posic_encoder_filtered']
        df_merged.loc[segment_mask, keyence_col] = df_merged.loc[segment_mask, 'keyence_mm']
        df_merged.loc[segment_mask, error_col] = (
            df_merged.loc[segment_mask, 'keyence_mm'] - 
            df_merged.loc[segment_mask, 'posic_encoder_filtered']
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
    
def plot_keyence_vs_magnetic(df_merged, version):
    """
    Plot Keyence displacement on x-axis vs Magnetic filtered displacement on y-axis.
    Creates a scatter plot to visualize the correlation between the two sensors.
    
    Parameters:
    -----------
    df_merged : pandas.DataFrame
        The merged dataframe containing both sensor data
    version : int
        Version number for the plot title and filename
    """
    
    # Filter out rows where either measurement is NaN
    valid_mask = df_merged['keyence_mm'].notna() & df_merged['posic_encoder_filtered'].notna()
    
    keyence_data = df_merged.loc[valid_mask, 'keyence_mm'].values
    magnetic_data = df_merged.loc[valid_mask, 'posic_encoder_filtered'].values
    
    # Create figure
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    
    # Create scatter plot
    scatter = ax.scatter(keyence_data, magnetic_data, 
                        c=df_merged.loc[valid_mask, 'time_s'], 
                        cmap='viridis', 
                        alpha=0.6, 
                        s=10)
    
    # Add colorbar to show time progression
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label('Time (s)', fontsize=12)
    
    # Add ideal 1:1 line (perfect correlation)
    min_val = min(keyence_data.min(), magnetic_data.min())
    max_val = max(keyence_data.max(), magnetic_data.max())
    ax.plot([min_val, max_val], [min_val, max_val], 
            'r--', linewidth=2, alpha=0.7, label='Ideal 1:1 Line')
    
    # Calculate and display correlation coefficient
    correlation = np.corrcoef(keyence_data, magnetic_data)[0, 1]
    
    # Calculate RMSE
    rmse = np.sqrt(np.mean((keyence_data - magnetic_data)**2))
    
    # Add statistics text box
    stats_text = f'Correlation: {correlation:.4f}\nRMSE: {rmse:.4f} mm'
    ax.text(0.05, 0.95, stats_text, 
            transform=ax.transAxes, 
            fontsize=11, 
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    ax.set_xlabel('Keyence Displacement (mm)', fontsize=12)
    ax.set_ylabel('Posic Filtered Displacement (mm)', fontsize=12)
    ax.set_title(f'Keyence vs Posic Sensor Comparison (Version {version})', fontsize=14)
    ax.legend(fontsize=10, loc='lower right')
    ax.grid(True, alpha=0.3)
    
    # Make axes equal to better visualize deviation from 1:1 line
    ax.set_aspect('equal', adjustable='box')
    
    plt.tight_layout()
    
    # Save the figure
    output_path = os.path.join(graphs_dir, f'keyence_vs_posic_v{version}.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\nKeyence vs Posic correlation plot saved to: {output_path}")
    print(f"Correlation coefficient: {correlation:.4f}")
    print(f"RMSE: {rmse:.4f} mm")
    
def create_lookup_table(df_merged, N, version):
    """
    Create a lookup table for converting magnetic sensor readings to calibrated displacement values.
    The lookup table maps magnetic filtered values to Keyence displacement values.
    
    Parameters:
    -----------
    df_merged : pandas.DataFrame
        The merged dataframe containing both sensor data
    N : int
        Bit resolution. Creates lookup table with 2^N entries
    version : int
        Version number for the output filename
    
    Returns:
    --------
    lookup_table : numpy.array
        Array of size 2^N containing interpolated Keyence values
    """
    
    # Filter out rows where either measurement is NaN
    valid_mask = df_merged['keyence_mm'].notna() & df_merged['posic_encoder_filtered'].notna()
    
    keyence_data = df_merged.loc[valid_mask, 'keyence_mm'].values
    magnetic_data = df_merged.loc[valid_mask, 'posic_encoder_filtered'].values
    
    # Sort by magnetic data for proper interpolation
    sorted_indices = np.argsort(magnetic_data)
    magnetic_sorted = magnetic_data[sorted_indices]
    keyence_sorted = keyence_data[sorted_indices]
    
    # Determine the range of magnetic data
    magnetic_min = 0
    magnetic_max = magnetic_sorted.max()
    
    print(f"\n{'='*60}")
    print(f"Creating Lookup Table (Version {version})")
    print(f"{'='*60}")
    print(f"Bit resolution (N): {N}")
    print(f"Lookup table size: 2^{N} = {2**N} entries")
    print(f"Magnetic data range: {magnetic_min:.4f} mm to {magnetic_max:.4f} mm")
    print(f"Keyence data range: {keyence_sorted.min():.4f} mm to {keyence_sorted.max():.4f} mm")
    
    # Create lookup table indices from 0 to 2^N - 1
    table_size = 2**N
    lookup_indices = np.arange(table_size)
    
    # Map indices to magnetic displacement values (evenly spaced)
    # Index 0 corresponds to magnetic_min, Index 2^N-1 corresponds to magnetic_max
    magnetic_lookup_values = magnetic_min + (magnetic_max - magnetic_min) * lookup_indices / (table_size - 1)
    
    # Interpolate to find corresponding Keyence values
    lookup_table = np.interp(magnetic_lookup_values, magnetic_sorted, keyence_sorted)
    
    print(f"Interpolation complete!")
    print(f"Index 0: Magnetic={magnetic_lookup_values[0]:.4f} mm -> Keyence={lookup_table[0]:.4f} mm")
    print(f"Index {table_size-1}: Magnetic={magnetic_lookup_values[-1]:.4f} mm -> Keyence={lookup_table[-1]:.4f} mm")
    
    # Export to TXT file with comma separation
    txt_output_path = os.path.join(script_dir, f'lookup_table_v{version}_N{N}.txt')
    with open(txt_output_path, 'w') as f:
        # Write as comma-separated values
        f.write(','.join(map(str, lookup_table)))
    
    print(f"\nLookup table exported to: {txt_output_path}")
    
    # Also export a CSV with more detail (index, magnetic value, keyence value)
    csv_output_path = os.path.join(script_dir, f'lookup_table_v{version}_N{N}_detailed.csv')
    df_lookup = pd.DataFrame({
        'index': lookup_indices,
        'magnetic_mm': magnetic_lookup_values,
        'keyence_calibrated_mm': lookup_table
    })
    df_lookup.to_csv(csv_output_path, index=False)
    print(f"Detailed lookup table (with indices and magnetic values) exported to: {csv_output_path}")
    
    # Create a visualization of the lookup table
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Plot 1: Lookup table values vs index
    ax1.plot(lookup_indices, lookup_table, linewidth=2, color='blue', label='Lookup Table')
    ax1.set_xlabel(f'Index (0 to 2^{N}-1)', fontsize=12)
    ax1.set_ylabel('Calibrated Displacement (mm)', fontsize=12)
    ax1.set_title(f'Lookup Table: Index to Calibrated Displacement (Version {version}, N={N})', fontsize=14)
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Lookup table overlaid on original data
    ax2.scatter(magnetic_data, keyence_data, 
                c=df_merged.loc[valid_mask, 'time_s'], 
                cmap='viridis', 
                alpha=0.3, 
                s=5, 
                label='Raw Data')
    ax2.plot(magnetic_lookup_values, lookup_table, 
             linewidth=3, 
             color='red', 
             label='Lookup Table Interpolation',
             alpha=0.8)
    
    cbar = plt.colorbar(ax2.collections[0], ax=ax2)
    cbar.set_label('Time (s)', fontsize=10)
    
    ax2.set_xlabel('Magnetic Filtered Displacement (mm)', fontsize=12)
    ax2.set_ylabel('Keyence Displacement (mm)', fontsize=12)
    ax2.set_title(f'Lookup Table Interpolation vs Raw Data (Version {version}, N={N})', fontsize=14)
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save the visualization
    plot_output_path = os.path.join(graphs_dir, f'lookup_table_v{version}_N{N}.png')
    plt.savefig(plot_output_path, dpi=300, bbox_inches='tight')
    print(f"Lookup table visualization saved to: {plot_output_path}")
        
    print(f"{'='*60}\n")
    
    return lookup_table

# # def create_lookup_table(df_merged, segment_info, segment_num, N, version):
# #     """
# #     Create a lookup table for converting magnetic sensor readings to calibrated displacement values.
# #     The lookup table maps magnetic filtered values to Keyence displacement values.
    
# #     Parameters:
# #     -----------
# #     df_merged : pandas.DataFrame
# #         The merged dataframe containing both sensor data
# #     segment_info : list of dict
# #         Information about each segment
# #     segment_num : int
# #         The segment number to use for creating the lookup table (1-indexed)
# #     N : int
# #         Bit resolution. Creates lookup table with 2^N entries
# #     version : int
# #         Version number for the output filename
    
# #     Returns:
# #     --------
# #     lookup_table : numpy.array
# #         Array of size 2^N containing interpolated Keyence values
# #     """
    
# #     # Find the segment info
# #     segment = None
# #     for seg in segment_info:
# #         if seg['segment_num'] == segment_num:
# #             segment = seg
# #             break
    
# #     if segment is None:
# #         print(f"Error: Segment {segment_num} not found!")
# #         return None
    
# #     # Get the mask for this segment - only use data from this segment
# #     segment_mask = df_merged[segment['error_col']].notna()
    
# #     # Filter to only this segment's data
# #     keyence_data = df_merged.loc[segment_mask, segment['keyence_col']].values
# #     magnetic_data = df_merged.loc[segment_mask, segment['magnetic_col']].values
    
# #     # Sort by magnetic data for proper interpolation
# #     sorted_indices = np.argsort(magnetic_data)
# #     magnetic_sorted = magnetic_data[sorted_indices]
# #     keyence_sorted = keyence_data[sorted_indices]
    
# #     # Determine the range of magnetic data
# #     magnetic_min = 0
# #     magnetic_max = magnetic_sorted.max()
    
# #     print(f"\n{'='*60}")
# #     print(f"Creating Lookup Table (Version {version}) - SEGMENT {segment_num}")
# #     print(f"{'='*60}")
# #     print(f"Segment time range: {segment['start_time']:.3f}s to {segment['end_time']:.3f}s")
# #     print(f"Segment duration: {segment['duration']:.3f}s")
# #     print(f"Data points in segment: {len(keyence_data)}")
# #     print(f"Bit resolution (N): {N}")
# #     print(f"Lookup table size: 2^{N} = {2**N} entries")
# #     print(f"Magnetic data range: {magnetic_min:.4f} mm to {magnetic_max:.4f} mm")
# #     print(f"Keyence data range: {keyence_sorted.min():.4f} mm to {keyence_sorted.max():.4f} mm")
    
# #     # Create lookup table indices from 0 to 2^N - 1
# #     table_size = 2**N
# #     lookup_indices = np.arange(table_size)
    
# #     # Map indices to magnetic displacement values (evenly spaced)
# #     # Index 0 corresponds to magnetic_min, Index 2^N-1 corresponds to magnetic_max
# #     magnetic_lookup_values = magnetic_min + (magnetic_max - magnetic_min) * lookup_indices / (table_size - 1)
    
# #     # Interpolate to find corresponding Keyence values
# #     lookup_table = np.interp(magnetic_lookup_values, magnetic_sorted, keyence_sorted)
    
# #     print(f"Interpolation complete!")
# #     print(f"Index 0: Magnetic={magnetic_lookup_values[0]:.4f} mm -> Keyence={lookup_table[0]:.4f} mm")
# #     print(f"Index {table_size-1}: Magnetic={magnetic_lookup_values[-1]:.4f} mm -> Keyence={lookup_table[-1]:.4f} mm")
    
# #     # Export to TXT file with comma separation
# #     txt_output_path = os.path.join(script_dir, f'lookup_table_v{version}_N{N}.txt')
# #     with open(txt_output_path, 'w') as f:
# #         # Write as comma-separated values
# #         f.write(','.join(map(str, lookup_table)))
    
# #     print(f"\nLookup table exported to: {txt_output_path}")
    
# #     # Also export a CSV with more detail (index, magnetic value, keyence value)
# #     csv_output_path = os.path.join(script_dir, f'lookup_table_v{version}_seg{segment_num}_N{N}_detailed.csv')
# #     df_lookup = pd.DataFrame({
# #         'index': lookup_indices,
# #         'magnetic_mm': magnetic_lookup_values,
# #         'keyence_calibrated_mm': lookup_table
# #     })
# #     df_lookup.to_csv(csv_output_path, index=False)
# #     print(f"Detailed lookup table (with indices and magnetic values) exported to: {csv_output_path}")
    
# #     # Create a visualization of the lookup table
# #     fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
# #     # Plot 1: Lookup table values vs index
# #     ax1.plot(lookup_indices, lookup_table, linewidth=2, color='blue', label='Lookup Table')
# #     ax1.set_xlabel(f'Index (0 to 2^{N}-1)', fontsize=12)
# #     ax1.set_ylabel('Calibrated Displacement (mm)', fontsize=12)
# #     ax1.set_title(f'Lookup Table: Index to Calibrated Displacement (Version {version}, Segment {segment_num}, N={N})', fontsize=14)
# #     ax1.legend(fontsize=10)
# #     ax1.grid(True, alpha=0.3)
    
# #     # Plot 2: Lookup table overlaid on segment data
# #     ax2.scatter(magnetic_data, keyence_data, 
# #                 alpha=0.3, 
# #                 s=5, 
# #                 label=f'Segment {segment_num} Data',
# #                 color='blue')
# #     ax2.plot(magnetic_lookup_values, lookup_table, 
# #              linewidth=3, 
# #              color='red', 
# #              label='Lookup Table Interpolation',
# #              alpha=0.8)
    
# #     ax2.set_xlabel('Magnetic Filtered Displacement (mm)', fontsize=12)
# #     ax2.set_ylabel('Keyence Displacement (mm)', fontsize=12)
# #     ax2.set_title(f'Lookup Table Interpolation vs Segment {segment_num} Data (Version {version}, N={N})', fontsize=14)
# #     ax2.legend(fontsize=10)
# #     ax2.grid(True, alpha=0.3)
    
# #     plt.tight_layout()
    
# #     # Save the visualization
# #     plot_output_path = os.path.join(graphs_dir, f'lookup_table_v{version}_seg{segment_num}_N{N}.png')
# #     plt.savefig(plot_output_path, dpi=300, bbox_inches='tight')
# #     print(f"Lookup table visualization saved to: {plot_output_path}")
    
# #     plt.show()
    
# #     print(f"{'='*60}\n")
    
# #     return lookup_table

# def load_lookup_table(filepath):
#     """
#     Helper function to load a lookup table from a txt file.
    
#     Parameters:
#     -----------
#     filepath : str
#         Path to the lookup table txt file
    
#     Returns:
#     --------
#     lookup_table : numpy.array
#         The loaded lookup table
#     """
#     with open(filepath, 'r') as f:
#         content = f.read()
#         lookup_table = np.array([float(x) for x in content.split(',')])
    
#     print(f"Loaded lookup table from: {filepath}")
#     print(f"Table size: {len(lookup_table)} entries")
#     print(f"Range: {lookup_table.min():.4f} to {lookup_table.max():.4f} mm")
    
#     return lookup_table

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

graphs_dir = os.path.join(script_dir, 'graphs')

# Build the full path to the CSV files
arduino_file_path = os.path.join(script_dir, f"arduino_output{version}.txt")
keyence_file_path = os.path.join(script_dir, f"keyence_output{version}.csv")

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

# # Remove rows where any measurement column is NaN
# df_arduino = df_arduino.dropna(subset=['magnetic_raw_mm', 'magnetic_filtered_mm', 'posic_encoder_mm'])

print(f"Arduino dataframe length {len(df_arduino)}")

# Import Keyence CSV file
df_keyence = pd.read_csv(keyence_file_path, header=None)

# Find the column with the most non-null values (since Keyence outputs multiple columns, only the long one has the position data)
column_lengths = df_keyence.count()
longest_column_index = column_lengths.idxmax()

# Create a new DataFrame with only the longest column
df_keyence = pd.DataFrame({
    'keyence_mm': df_keyence[longest_column_index]
})

# Merge both keyence and arduino dataframe (keyence will usually have less data points)
df_merged = pd.concat([df_arduino, df_keyence], axis=1)

# Create time_s column by calculating cumulative time differences in seconds
df_merged['time_s'] = (df_merged['time_ms'] - df_merged['time_ms'].iloc[0]) / 1000.0
# Zero the displacement values
df_merged['keyence_mm'] = (df_merged['keyence_mm'] - df_merged['keyence_mm'].iloc[0]) * -1
df_merged['posic_encoder_mm'] = df_merged['posic_encoder_mm'] - df_merged['posic_encoder_mm'].iloc[0]
df_merged['magnetic_raw'] = df_merged['magnetic_raw_mm']
df_merged['magnetic_raw_mm'] = df_merged['magnetic_raw_mm'] - df_merged['magnetic_raw_mm'].iloc[0]
df_merged['magnetic_raw_mm'] = df_merged['magnetic_raw_mm'] * -1 * 0.00048828125 * 0.9459754389

# ============================================================================
# APPLY LOW-PASS FILTER TO MAGNETIC_RAW AND POSIC_ENCODER DATA
# ============================================================================

samplingFreq = 200  # matching sampling frequency of arduino itself
cutoffFreq = 4

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
y = df_merged['magnetic_raw_mm'].values
yfilt = np.zeros(len(y))

# Apply the filter equation: yfilt[i] = a[1]*yfilt[i-1] + b[0]*y[i] + b[1]*y[i-1]
for i in range(1, len(y)):
    yfilt[i] = a[1]*yfilt[i-1] + b[0]*y[i] + b[1]*y[i-1]

# Add filtered data to dataframe
df_merged['magnetic_raw_filtered'] = yfilt

# Apply filter to posic_encoder data
y_posic = df_merged['posic_encoder_mm'].values
yfilt_posic = np.zeros(len(y_posic))

# Apply the filter equation: yfilt[i] = a[1]*yfilt[i-1] + b[0]*y[i] + b[1]*y[i-1]
for i in range(1, len(y_posic)):
    yfilt_posic[i] = a[1]*yfilt_posic[i-1] + b[0]*y_posic[i] + b[1]*y_posic[i-1]

# Add filtered data to dataframe
df_merged['posic_encoder_filtered'] = yfilt_posic

# ============================================================================
# FIND TURNING POINTS FOR MAGNETIC_RAW_FILTERED DATA
# ============================================================================
# Use only rows where magnetic_raw_filtered is not NaN
valid_mask = df_merged['keyence_mm'].notna()
comparison_data = df_merged.loc[valid_mask, 'keyence_mm'].values
time_data = df_merged.loc[valid_mask, 'time_s'].values

# Find local minima and maxima
local_min_indices = argrelextrema(df_merged["keyence_mm"].values, np.less, order=5)[0]
local_max_indices = argrelextrema(df_merged["keyence_mm"].values, np.greater, order=5)[0]

min_times = time_data[local_min_indices] # List of local mins
min_values = comparison_data[local_min_indices]
max_times = time_data[local_max_indices]
max_values = comparison_data[local_max_indices] # List of local maxes

# print(f"Found {len(local_min_indices)} local minima and {len(local_max_indices)} local maxima")
# print(local_min_indices)

# ============================================================================
# CREATE THE PLOTS
# ============================================================================
# After you've calculated min_times and max_times, add:
df_merged, segment_info = create_extrema_segments(df_merged, min_times, max_times)

# # Plot raw sensor data against time
plot_raw_sensor_data(df_merged, version)

# # Plot the overlapping segments
# plot_overlapping_segments(df_merged, segment_info, version)

# # Plot Keyence vs Magnetic correlation
# plot_keyence_vs_magnetic(df_merged, version)

# Create lookup table (e.g., N=12 for 4096 entries, N=10 for 1024 entries)
lookup_table = create_lookup_table(df_merged, N=12, version=version)
# lookup_table = create_lookup_table(df_merged, segment_info, segment_num=1, N=12, version=version)

# # Plot a single segment (e.g., segment 1)
# plot_single_segment(df_merged, segment_info, segment_num=2, version=version)

# plt.show()

# ============================================================================
# EXPORT COMBINED DATAFRAME TO CSV
# ============================================================================
# Select columns for export
df_export = df_merged[['time_s', 'magnetic_raw', 'magnetic_raw_mm', 'posic_encoder_mm', "magnetic_raw_filtered",
                        'posic_encoder_filtered', 'keyence_mm']].copy()

# Export to CSV
csv_output_path = os.path.join(script_dir, f'combined_sensor_data_v{version}_filtered.csv')
df_export.to_csv(csv_output_path, index=False)
print(f"\nCombined data exported to: {csv_output_path}")
print(f"Columns: {list(df_export.columns)}")
print(f"Total rows: {len(df_export)}")