import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.signal import argrelextrema
from scipy import signal

# #### CONTROLS ####
version = 88
# # Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

graphs_dir = os.path.join(script_dir, 'graphs')

def load_version_data(version: int, script_dir: str = None) -> pd.DataFrame:
    """
    Load, merge, and process Arduino and Keyence data for a given version number.

    Args:
        version: The version number to load (e.g., 75)
        script_dir: Directory containing the data files. Defaults to current working directory.

    Returns:
        A merged and processed DataFrame with time, keyence, encoder, and magnetic columns.
    """
    if script_dir is None:
        script_dir = os.getcwd()

    arduino_file_path = os.path.join(script_dir, f"arduino_output{version}.txt")
    keyence_file_path = os.path.join(script_dir, f"keyence_output{version}.csv")

    # Import Arduino TXT file
    df_arduino = pd.read_csv(
        arduino_file_path,
        header=None,
        delimiter=', ',
        names=['time_ms', 'magnetic_raw_mm', 'posic_encoder_mm', 'magnetic_wrapped'],
        engine='python'
    )

    # Strip whitespace and convert to numeric
    for col in df_arduino.columns:
        df_arduino[col] = pd.to_numeric(
            df_arduino[col].astype(str).str.strip(),
            errors='coerce'
        )

    # print(f"[v{version}] Arduino dataframe length: {len(df_arduino)}")

    # Import Keyence CSV file — keep only the column with the most non-null values
    df_keyence_raw = pd.read_csv(keyence_file_path, header=None)
    longest_column_index = df_keyence_raw.count().idxmax()
    df_keyence = pd.DataFrame({'keyence_mm': df_keyence_raw[longest_column_index]})

    # print(f"[v{version}] Keyence dataframe length: {len(df_keyence)}")

    # Merge (Keyence will usually have fewer rows)
    df_merged = pd.concat([df_arduino, df_keyence], axis=1)

    # Time column in seconds, zeroed from start
    df_merged['time_s'] = (df_merged['time_ms'] - df_merged['time_ms'].iloc[0]) / 1000.0

    # Zero and scale displacement values
    df_merged['keyence_mm'] = df_merged['keyence_mm'] * -1
    # df_merged['posic_encoder_mm'] = df_merged['posic_encoder_mm'] - df_merged['posic_encoder_mm'].iloc[0]
    # df_merged['magnetic_raw_mm'] = df_merged['magnetic_raw_mm'] - df_merged['magnetic_raw_mm'].iloc[0]
    df_merged['posic_encoder_mm'] = df_merged['posic_encoder_mm'] / 1000000 * 19.53125 * (-1) * 0.938
    df_merged['magnetic_raw'] = df_merged['magnetic_raw_mm']
    # df_merged['magnetic_raw_mm'] = df_merged['magnetic_raw_mm'] - df_merged['magnetic_raw_mm'].iloc[0]
    df_merged['magnetic_raw_mm'] = df_merged['magnetic_raw_mm'] * -1 * 0.00048828125 * 0.9459754389

    # print(f"[v{version}] Merged dataframe length: {len(df_merged)}")

    return df_merged

def plot_sensor_data(
    version,
    *plot_configs,
    n_graphs=1,
    title="Sensor Data vs Time",
    xlabel="Time (s)",
    ylabel="Displacement (mm)",
    figsize_per_graph=(16, 6),
    output_filename=None
):
    """
    Flexible sensor data plotting function.

    Parameters:
    -----------
    version : int
        Version number for the plot title and filename.
    *plot_configs : dict
        Each dict defines one line to plot. Keys:
            - df         : pd.DataFrame (required)
            - y_col      : str, column to plot on y-axis (required)
            - x_col      : str, column to plot on x-axis (default: 'time_s')
            - label      : str, legend label (default: y_col)
            - color      : str (default: auto)
            - linewidth  : float (default: 1.5)
            - alpha      : float (default: 0.8)
            - graph_idx  : int, which subplot to plot on, 0-indexed (default: 0)
            - plot_type  : 'line' or 'scatter' (default: 'line')
    n_graphs : int
        Number of subplots (stacked vertically). Default: 1.
    title : str or list of str
        Title(s) for the plot(s). If a single string, used for the figure
        suptitle. If a list, each entry is the title for the corresponding subplot.
    xlabel : str or list of str
        X-axis label(s). Scalar applies to all subplots; list applies per subplot.
    ylabel : str or list of str
        Y-axis label(s). Scalar applies to all subplots; list applies per subplot.
    figsize_per_graph : tuple
        (width, height) per subplot. Total figure height scales with n_graphs.
    output_filename : str, optional
        Output filename (without directory). Defaults to
        f'sensor_data_v{version}.png'.

    Example:
    --------
    plot_sensor_data(
        version=1,
        {"df": df_merged, "y_col": "keyence_mm",      "label": "Keyence",       "color": "blue",  "graph_idx": 0},
        {"df": df_merged, "y_col": "magnetic_raw_mm",  "label": "Magnetic",      "color": "red",   "graph_idx": 0},
        {"df": df_merged, "y_col": "posic_encoder_mm", "label": "Posic Encoder", "color": "green", "graph_idx": 1},
        n_graphs=2,
        title=["Raw Signals", "Encoder"],
        ylabel=["Displacement (mm)", "Displacement (mm)"],
    )
    """

    def _to_list(val, n):
        """Broadcast scalar or list to length-n list."""
        if isinstance(val, list):
            return val
        return [val] * n

    titles = _to_list(title, n_graphs)
    xlabels = _to_list(xlabel, n_graphs)
    ylabels = _to_list(ylabel, n_graphs)

    fig_height = figsize_per_graph[1] * n_graphs
    fig, axes = plt.subplots(n_graphs, 1, figsize=(figsize_per_graph[0], fig_height))

    # Always work with a list of axes
    if n_graphs == 1:
        axes = [axes]

    # Apply per-subplot labels and titles
    for i, ax in enumerate(axes):
        ax.set_xlabel(xlabels[i], fontsize=12)
        ax.set_ylabel(ylabels[i], fontsize=12)
        ax.set_title(titles[i], fontsize=14)
        ax.grid(True, alpha=0.3)

    # Plot each config
    for cfg in plot_configs:
        df      = cfg["df"]
        y_col   = cfg["y_col"]
        x_col   = cfg.get("x_col", "time_s")
        label   = cfg.get("label", y_col)
        color   = cfg.get("color", None)
        lw      = cfg.get("linewidth", 1.5)
        alpha   = cfg.get("alpha", 0.8)
        idx     = cfg.get("graph_idx", 0)
        ptype   = cfg.get("plot_type", "line")

        ax = axes[idx]
        valid = df[y_col].notna()

        plot_kwargs = dict(color=color, label=label, alpha=alpha)

        if ptype == "scatter":
            ax.scatter(df.loc[valid, x_col], df.loc[valid, y_col], **plot_kwargs)
        else:
            ax.scatter(df.loc[valid, x_col], df.loc[valid, y_col],
                    linewidth=lw, **plot_kwargs)

        ax.legend(fontsize=10, loc="best")

    # Overall suptitle only when n_graphs > 1 or title is a single string
    if n_graphs > 1:
        fig.suptitle(f"Version {version}", fontsize=13, y=1.01)

    plt.tight_layout()

    fname = output_filename or f"sensor_data_v{version}.png"
    output_path = os.path.join(graphs_dir, fname)
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"\nPlot saved to: {output_path}")

def compute_errors(df: pd.DataFrame) -> pd.DataFrame:
    """
    Compute magnetic and positional encoder errors relative to Keyence reference.

    Args:
        df: A merged DataFrame returned by load_version_data()

    Returns:
        The same DataFrame with added 'magnetic_error' and 'posic_error' columns.
    """
    df["magnetic_error"] = df["keyence_mm"] - df["magnetic_raw_mm"]
    df["posic_error"] = df["keyence_mm"] - df["posic_encoder_mm"]
    return df

def sync_dataframes(*dfs: pd.DataFrame, col: str = "magnetic_raw", search_rows: int = 20) -> list[pd.DataFrame]:
    
    """
    Sync multiple DataFrames to start at the same shared value in a given column.
    Compares the first `search_rows` rows of each DataFrame to find the first
    value that appears in all of them, then trims each DataFrame to start there.

    Args:
        *dfs: Two or more DataFrames to sync.
        col: Column to compare (default: 'magnetic_raw').
        search_rows: How many rows from the top to search (default: 20).

    Returns:
        A list of trimmed DataFrames, in the same order as input.
    """
    # Get the candidate values from the top N rows of each dataframe
    candidate_sets = [set(df[col].iloc[:search_rows].dropna()) for df in dfs]

    # Find values common to all dataframes
    shared_values = candidate_sets[0].intersection(*candidate_sets[1:])

    if not shared_values:
        raise ValueError(f"No shared values found in the first {search_rows} rows of '{col}' across all DataFrames.")

    # For each dataframe, find the first row index (within search window) that holds a shared value
    first_shared_indices = []
    for df in dfs:
        for i, val in enumerate(df[col].iloc[:search_rows]):
            if val in shared_values:
                first_shared_indices.append(i)
                break

    # The sync point is the shared value that appears latest (i.e. requires the most trimming)
    sync_pos = max(first_shared_indices)
    sync_value = dfs[first_shared_indices.index(sync_pos)][col].iloc[sync_pos]

    print(f"Syncing on value: {sync_value} (column: '{col}')")

    # Trim each dataframe to start at the sync value
    trimmed = []
    for df in dfs:
        start_idx = df[col].iloc[:search_rows][df[col].iloc[:search_rows] == sync_value].index[0]
        trimmed.append(df.loc[start_idx:].reset_index(drop=True))
        print(f"  Trimmed {start_idx} rows → new length: {len(trimmed[-1])}")

    return trimmed

def normalize_to(df: pd.DataFrame, reference_df: pd.DataFrame, columns: list[str]) -> pd.DataFrame:
    """
    Normalize specified columns in df using the first row values of a reference DataFrame.

    Args:
        df: DataFrame to normalize.
        reference_df: DataFrame whose first-row values are used as the zero reference.
        columns: List of column names to normalize.

    Returns:
        The df with specified columns zeroed relative to reference_df's first row.
    """
    for col in columns:
        df[col] = df[col] - reference_df[col].iloc[0]
    return df

def add_cycle_groups(*dfs: pd.DataFrame, time_col: str = "time_s", threshold: float = 0.5) -> list[pd.DataFrame]:
    """
    Adds a 'cycle_group' column to each DataFrame that increments only when
    the time difference between consecutive rows exceeds the threshold.

    Args:
        *dfs: One or more DataFrames to process.
        time_col: Column containing time in seconds (default: 'time_s').
        threshold: Time gap in seconds that triggers a new group (default: 0.5).

    Returns:
        A list of DataFrames with the new 'cycle_group' column added.
    """
    result = []
    for df in dfs:
        df = df.copy()
        time_diff = df[time_col].diff().abs()
        df["cycle_group"] = (time_diff > threshold).cumsum()
        result.append(df)
    return result

df_main = load_version_data(version, script_dir)
df_76 = load_version_data(76, script_dir)
df_77 = load_version_data(77, script_dir)

df_main, df_76, df_77 = sync_dataframes(df_main, df_76, df_77, col="magnetic_raw", search_rows=50)

cols = ["keyence_mm", "posic_encoder_mm", "magnetic_raw_mm"]

df_main = normalize_to(df_main, df_main, cols)
df_76 = normalize_to(df_76, df_76, cols)
df_77 = normalize_to(df_77, df_77, cols)

# Plot raw sensor data against time
cfg1 = {"df": df_main, "y_col": "keyence_mm",      "label": "Keyence",       "color": "blue"}
cfg2 = {"df": df_main, "y_col": "magnetic_raw_mm",  "label": "Magnetic",      "color": "red"}
cfg3 = {"df": df_main, "y_col": "posic_encoder_mm", "label": "Posic Encoder", "color": "green"}

# plot_sensor_data(
#     1,
#     cfg1, cfg2, cfg3,
#     n_graphs=1,
#     title=f"Raw Sensor Data vs Time (Version {version})",
#     output_filename=f"raw_sensor_data_v{version}.png"
# )

## ERROR PlOTS
df_main = compute_errors(df_main)
df_76 = compute_errors(df_76)
df_77 = compute_errors(df_77)

# cfg1 = {"df": df_main, "x_col": "magnetic_raw_mm", "y_col": "posic_error", "label": "Posic Error (V75)", "color": "blue"}
# cfg2 = {"df": df_76, "x_col": "magnetic_raw_mm", "y_col": "posic_error", "label": "Posic Error (V76)",  "color": "red"}
# cfg3 = {"df": df_77, "x_col": "magnetic_raw_mm", "y_col": "posic_error", "label": "Posic Error (V77)",  "color": "green"}

cfg1 = {"df": df_main.iloc[:len(df_main)//2], "x_col": "keyence_mm", "y_col": "magnetic_error", "label": "Magnetic Error (V80)", "color": "blue"}
cfg2 = {"df": df_76.iloc[:len(df_76)//2], "x_col": "keyence_mm", "y_col": "magnetic_error", "label": "Magnetic Error (V76)",  "color": "red"}
cfg3 = {"df": df_77.iloc[:len(df_77)//2], "x_col": "keyence_mm", "y_col": "magnetic_error", "label": "Magnetic Error (V77)",  "color": "green"}

plot_sensor_data(
    1,
    cfg1, cfg2, cfg3,
    n_graphs=1,
    title=f"Error vs Ground Truth Displacement (Magnetic Encoder)",
    # title=f"Error vs Ground Truth Displacement (Version {version})",
    output_filename=f"error_data_v{version}.png",
    xlabel="Magnetic Raw Displacement (mm)"
)

# plt.show()

df_main, df_76, df_77 = add_cycle_groups(df_main, df_76, df_77)

cfg1 = {"df": df_main[df_main["cycle_group"] == 1], "x_col": "magnetic_raw_mm", "y_col": "magnetic_error", "label": f"Magnetic Error (V{version})",       "color": "blue"}
cfg2 = {"df": df_76[df_76["cycle_group"] == 1], "x_col": "magnetic_raw_mm", "y_col": "magnetic_error", "label": "Magnetic Error (V76)",      "color": "red"}
cfg3 = {"df": df_77[df_77["cycle_group"] == 1], "x_col": "magnetic_raw_mm", "y_col": "magnetic_error", "label": "Magnetic Error (V77)", "color": "green"}

plot_sensor_data(
    1,
    cfg1, cfg2, cfg3,
    n_graphs=1,
    title=f"Error vs Ground Truth Displacement Forward Motion (Magnetic Encoder)",
    output_filename=f"error_backward_magnetic.png"
)

# Note for this case, cycle_group = 0 is from 0 to 0.1 mm, 1 is 0.1 to 5 mm, and 2 is 5 to 0.1 mm
# Plot raw sensor data against time
cfg1 = {"df": df_main[df_main["cycle_group"] == 2], "x_col": "magnetic_raw_mm", "y_col": "magnetic_error", "label": f"Magnetic Error (V{version})",       "color": "blue"}
cfg2 = {"df": df_76[df_76["cycle_group"] == 2], "x_col": "magnetic_raw_mm", "y_col": "magnetic_error", "label": "Magnetic Error (V76)",      "color": "red"}
cfg3 = {"df": df_77[df_77["cycle_group"] == 2], "x_col": "magnetic_raw_mm", "y_col": "magnetic_error", "label": "Magnetic Error (V77)", "color": "green"}

plot_sensor_data(
    1,
    cfg1, cfg2, cfg3,
    n_graphs=1,
    title=f"Error vs Ground Truth Displacement Backward Motion (Magnetic Encoder)",
    output_filename=f"error_backward_magnetic.png"
)



# plt.show()


from scipy.signal import savgol_filter
import matplotlib.pyplot as plt

# 2. Apply Savitzky-Golay Filter
#    - window_length: number of points used to fit each polynomial (must be odd)
#    - polyorder: degree of the polynomial (must be < window_length)
window_length = 51   # adjust: larger = smoother, must be odd
polyorder = 3        # adjust: typically 2–4

df_main_forward = df_main[df_main["cycle_group"] == 1]

df_main_forward["magnetic_error_smoothed"] = savgol_filter(
    df_main_forward["magnetic_error"],
    window_length=window_length,
    polyorder=polyorder
)

df_main_backward = df_main[df_main["cycle_group"] == 2]

df_main_backward["magnetic_error_smoothed"] = savgol_filter(
    df_main_backward["magnetic_error"],
    window_length=window_length,
    polyorder=polyorder
)

# Create index column that increments by 1 when sequential difference > 100
# df_main_forward["mag_error_LUT"] = df_main_forward["magnetic_error_smoothed"]
df_main_forward["mag_error_LUT"] = (df_main_forward["magnetic_error_smoothed"] / 0.00048828125).round().astype(int)
diff = df_main_forward["magnetic_wrapped"].diff().abs()
df_main_forward["mag_segment_fwd"] = (diff > 100).cumsum()

segment_counts = df_main_forward.groupby("mag_segment_fwd").size()
valid_segments = segment_counts[segment_counts > 10].index
df_main_forward = df_main_forward[df_main_forward["mag_segment_fwd"].isin(valid_segments)].reset_index(drop=True)

### START CREATING LUT ###
# Sorting the magnetic_wrapped index in order
df_main_forward = df_main_forward.groupby("mag_segment_fwd", group_keys=False).apply(lambda x: x.sort_values("magnetic_wrapped")).reset_index(drop=True)

# Recreate mag_segment_fwd after sorting
diff = df_main_forward["magnetic_wrapped"].diff().abs()
df_main_forward["mag_segment_fwd"] = (diff > 100).cumsum()

# Average mag_error_LUT for duplicate magnetic_wrapped values, then drop duplicates
df_main_forward = (df_main_forward.groupby(["mag_segment_fwd", "magnetic_wrapped"], as_index=False)
                 .agg(mag_error_LUT=("mag_error_LUT", lambda x: round(x.mean())))
                 .reset_index(drop=True))

# ## Not even averaging - straight up dropping duplicates
# df_main_forward = (df_main_forward
#                  .sort_values(["mag_segment_fwd", "magnetic_wrapped"])
#                  .drop_duplicates(subset=["mag_segment_fwd", "magnetic_wrapped"])
#                  .reset_index(drop=True))

# Fill in skipped magnetic_wrapped indexes by interpolating mag_error_LUT
# def fill_missing_indexes(group):
#     min_idx = group["magnetic_wrapped"].min()
#     max_idx = group["magnetic_wrapped"].max()
#     full_range = pd.DataFrame({"magnetic_wrapped": range(min_idx, max_idx + 1)})
#     merged = full_range.merge(group[["magnetic_wrapped", "mag_error_LUT", "magnetic_raw"]], on="magnetic_wrapped", how="left")
#     merged["mag_error_LUT"] = merged["mag_error_LUT"].interpolate(method="linear")
#     merged["magnetic_raw"] = merged["magnetic_raw"].interpolate(method="linear")
#     return merged

def fill_missing_indexes(group):
    min_idx = group["magnetic_wrapped"].min()
    max_idx = group["magnetic_wrapped"].max()
    full_range = pd.DataFrame({"magnetic_wrapped": range(min_idx, max_idx + 1)})
    merged = full_range.merge(group[["magnetic_wrapped", "mag_error_LUT"]], on="magnetic_wrapped", how="left")
    merged["mag_error_LUT"] = merged["mag_error_LUT"].interpolate(method="linear").round().astype(int)
    return merged

def pad_to_full_range(df: pd.DataFrame,
                      segment_col: str,
                      index_col: str = "magnetic_wrapped",
                      value_col: str = "mag_error_LUT",
                      full_min: int = 0,
                      full_max: int = 4095) -> pd.DataFrame:
    """
    For each segment, ensure the index_col spans [full_min, full_max],
    padding any missing boundary values with 0.
    """
    def pad_group(group):
        full_range = pd.DataFrame({index_col: range(full_min, full_max + 1)})
        merged = full_range.merge(group[[index_col, value_col]], on=index_col, how="left")
        merged[value_col] = merged[value_col].fillna(0).round().astype(int)
        return merged

    return (df.groupby(segment_col, group_keys=False)
              .apply(pad_group)
              .reset_index(drop=True))

df_main_forward = (df_main_forward.groupby("mag_segment_fwd", group_keys=False)
                 .apply(fill_missing_indexes)
                 .reset_index(drop=True))

# Reassign mag_segment_fwd after filling gaps
diff = df_main_forward["magnetic_wrapped"].diff().abs()
df_main_forward["mag_segment_fwd"] = (diff > 100).cumsum()

# Pad each segment so magnetic_wrapped covers 0–4095 with 0s at boundaries
df_main_forward = pad_to_full_range(df_main_forward, segment_col="mag_segment_fwd")
diff = df_main_forward["magnetic_wrapped"].diff().abs()
df_main_forward["mag_segment_fwd"] = (diff > 100).cumsum()

# ─── Export df_main_forward as a C header LUT ───────────────────────────────────
def export_lut_header(df: pd.DataFrame,
                      file_title: str = "MAG_ENCODER_LUT_H",
                      variable_name: str = "mag_encoder_LUT",
                      segment_col: str = "mag_segment_fwd",
                      index_col:   str = "magnetic_wrapped",
                      value_col:   str = "mag_error_LUT",
                      output_path: str = "mag_encoder_LUT.h") -> None:
    """
    Export a DataFrame as a nested C-array LUT in a .h file.

    The outer array is indexed by segment (mag_segment_fwd).
    Each inner array is indexed by magnetic_wrapped value (offset to 0).
    Missing wrapped values within a segment are filled with 0.

    Args:
        df          : DataFrame containing the LUT data.
        segment_col : Column name for the outer (segment) index.
        index_col   : Column name for the inner (wrapped encoder) index.
        value_col   : Column name for the correction values.
        output_path : Destination .h file path.
    """
    segments     = sorted(df[segment_col].unique())
    n_segments   = len(segments)

    # Compute the global wrapped-index range so all inner arrays share the same length
    global_min   = int(df[index_col].min())
    global_max   = int(df[index_col].max())
    inner_length = global_max - global_min + 1

    lines = []
    lines.append("/* Auto-generated magnetic encoder LUT */")
    lines.append("/* Outer index: mag_segment_fwd  |  Inner index: magnetic_wrapped (offset by MIN_WRAPPED) */")
    lines.append("")
    lines.append(f"#ifndef {file_title}")
    lines.append(f"#define {file_title}")
    lines.append("")
    lines.append(f"#define LUT_NUM_SEGMENTS   {n_segments}")
    lines.append(f"#define LUT_INNER_LENGTH   {inner_length}")
    lines.append(f"#define LUT_WRAPPED_OFFSET {global_min}  /* subtract from magnetic_wrapped before indexing */")
    lines.append("")
    lines.append(f"static const int16_t {variable_name}[{n_segments}][{inner_length}] = {{")

    for seg_i, seg_id in enumerate(segments):
        seg_df  = df[df[segment_col] == seg_id].set_index(index_col)[value_col]
        row_vals = []
        for wrapped_idx in range(global_min, global_max + 1):
            row_vals.append(int(seg_df.get(wrapped_idx, 0)))

        # Format as a commented inner array
        inner_str = ", ".join(str(v) for v in row_vals)
        comma     = "," if seg_i < n_segments - 1 else ""
        lines.append(f"    /* segment {seg_id} */")
        lines.append(f"    {{{inner_str}}}{comma}")

    lines.append("};")
    lines.append("")
    lines.append(f"#endif /* {file_title} */")

    with open(output_path, "w") as f:
        f.write("\n".join(lines))

    print(f"LUT header written to: {output_path}")

export_lut_header(
    df_main_forward,
    file_title = f"MAG_ENCODER_FWD_LUT_H_V{version}",
    variable_name = f"mag_encoder_fwd_LUT_V{version}",
    segment_col  = "mag_segment_fwd",
    index_col    = "magnetic_wrapped",
    value_col    = "mag_error_LUT",
    output_path  = rf"C:\Users\bzhen\OneDrive\Documents\Arduino\resolution_test\mag_encoder_fwd_LUT_V{version}.h"
)

df_main_forward.to_csv(f"output_LUT_fwd_V{version}.csv", index=False)

### PLOT EVERYTHING
df_main_forward["mag_x_plot"] = df_main_forward["magnetic_wrapped"] + 4096*(df_main_forward["mag_segment_fwd"])
# df_main_forward["mag_x_plot"] = df_main_forward["magnetic_wrapped"] + 4096*(max(df_main_forward["mag_segment_fwd"]) - df_main_forward["mag_segment_fwd"])
plot_75_fwd = {"df": df_main_forward, "x_col": "mag_x_plot", "y_col": "mag_error_LUT", "label": "Unwrapped Magnetic Data", "color": "green"}
plot_75_fwd1 = {"df": df_main_forward, "x_col": "magnetic_raw", "y_col": "mag_error_LUT", "label": "Wrapped Magnetic Data", "color": "red"}

# plot_sensor_data(
#     1,
#     plot_75_fwd,
#     # plot_75_fwd1, plot_75_fwd,
#     n_graphs=1,
#     xlabel="True Displacement (counts)",
#     title=f"Filtered Error vs Ground Truth Displacement Forward Motion (Magnetic Encoder)",
#     output_filename=f"filtered_LUT_75_fwd.png"
# )
# plt.show()

####### FOR BACKWARD LUT NOW
# Create index column that increments by 1 when sequential difference > 100
# df_main_forward["mag_error_LUT"] = df_main_forward["magnetic_error_smoothed"]
df_main_backward["mag_error_LUT"] = (df_main_backward["magnetic_error_smoothed"] / 0.00048828125).round().astype(int)
diff = df_main_backward["magnetic_wrapped"].diff().abs()
df_main_backward["mag_segment_bkwd"] = (diff > 100).cumsum()

segment_counts = df_main_backward.groupby("mag_segment_bkwd").size()
valid_segments = segment_counts[segment_counts > 10].index
df_main_backward = df_main_backward[df_main_backward["mag_segment_bkwd"].isin(valid_segments)].reset_index(drop=True)

### START CREATING LUT ###
# Sorting the magnetic_wrapped index in order
df_main_backward = df_main_backward.groupby("mag_segment_bkwd", group_keys=False).apply(lambda x: x.sort_values("magnetic_wrapped")).reset_index(drop=True)

# Recreate mag_segment_fwd after sorting
diff = df_main_backward["magnetic_wrapped"].diff().abs()
df_main_backward["mag_segment_bkwd"] = (3-(diff > 100).cumsum())
# df_main_backward["mag_segment_bkwd"] = (diff > 100).cumsum()

# Average mag_error_LUT for duplicate magnetic_wrapped values, then drop duplicates
df_main_backward = (df_main_backward.groupby(["mag_segment_bkwd", "magnetic_wrapped"], as_index=False)
                 .agg(mag_error_LUT=("mag_error_LUT", lambda x: round(x.mean())))
                 .reset_index(drop=True))

# ## Not even averaging - straight up dropping duplicates
# df_main_forward = (df_main_forward
#                  .sort_values(["mag_segment_fwd", "magnetic_wrapped"])
#                  .drop_duplicates(subset=["mag_segment_fwd", "magnetic_wrapped"])
#                  .reset_index(drop=True))

# Fill in skipped magnetic_wrapped indexes by interpolating mag_error_LUT
# def fill_missing_indexes(group):
#     min_idx = group["magnetic_wrapped"].min()
#     max_idx = group["magnetic_wrapped"].max()
#     full_range = pd.DataFrame({"magnetic_wrapped": range(min_idx, max_idx + 1)})
#     merged = full_range.merge(group[["magnetic_wrapped", "mag_error_LUT", "magnetic_raw"]], on="magnetic_wrapped", how="left")
#     merged["mag_error_LUT"] = merged["mag_error_LUT"].interpolate(method="linear")
#     merged["magnetic_raw"] = merged["magnetic_raw"].interpolate(method="linear")
#     return merged

df_main_backward = (df_main_backward.groupby("mag_segment_bkwd", group_keys=False)
                 .apply(fill_missing_indexes)
                 .reset_index(drop=True))

# Reassign mag_segment_fwd after filling gaps
diff = df_main_backward["magnetic_wrapped"].diff().abs()
df_main_backward["mag_segment_bkwd"] = (diff > 100).cumsum()

# Pad each segment so magnetic_wrapped covers 0–4095 with 0s at boundaries
df_main_backward = pad_to_full_range(df_main_backward, segment_col="mag_segment_bkwd")
diff = df_main_backward["magnetic_wrapped"].diff().abs()
df_main_backward["mag_segment_bkwd"] = (diff > 100).cumsum()

export_lut_header(
    df_main_backward,
    file_title = f"MAG_ENCODER_BKWD_LUT_H_V{version}",
    variable_name = f"mag_encoder_bkwd_LUT_V{version}",
    segment_col  = "mag_segment_bkwd",
    index_col    = "magnetic_wrapped",
    value_col    = "mag_error_LUT",
    output_path  = rf"C:\Users\bzhen\OneDrive\Documents\Arduino\resolution_test\mag_encoder_bkwd_LUT_V{version}.h"
)

df_main_backward.to_csv(f"output_bkwd_LUT_V{version}.csv", index=False)

### PLOT EVERYTHING
df_main_backward["mag_x_plot"] = df_main_backward["magnetic_wrapped"] + 4096*(df_main_backward["mag_segment_bkwd"])
# df_main_backward["mag_x_plot"] = df_main_backward["magnetic_wrapped"] + 4096*(max(df_main_backward["mag_segment_bkwd"])-df_main_backward["mag_segment_bkwd"])
plot_75_bkwd = {"df": df_main_backward, "x_col": "mag_x_plot", "y_col": "mag_error_LUT", "label": "Unwrapped Magnetic Data", "color": "green"}
plot_75_bkwd1 = {"df": df_main_backward, "x_col": "magnetic_raw", "y_col": "mag_error_LUT", "label": "Wrapped Magnetic Data", "color": "red"}

plot_sensor_data(
    1,
    plot_75_bkwd,
    # plot_75_fwd1, plot_75_fwd,
    n_graphs=1,
    xlabel="True Displacement (counts)",
    title=f"Filtered Error vs Ground Truth Displacement Backward Motion (Magnetic Encoder)",
    output_filename=f"filtered_LUT_75_bkwd.png"
)

# plt.show()
