"""
Satellite Elevation and Azimuth Plotting
"""

from matplotlib.patches import Circle
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def plot_elevation_time(df, time_col='time', elev_col='elevation',
                        sat_id_col='sat_id', timeseries_col='timeseries',
                        figsize=(14, 6), ylim=(0, 90)):
    """
    Plot elevation angle vs time for all satellites.

    Parameters:
        df (pd.DataFrame): Main dataframe where each row is a satellite
        time_col (str): Column name for time in nested dataframes
        elev_col (str): Column name for elevation in nested dataframes
        sat_id_col (str): Column name for satellite ID in main dataframe
        timeseries_col (str): Column name containing nested timeseries dataframes
        figsize (tuple): Figure size (width, height)
        ylim (tuple): Y-axis limits for elevation

    Returns:
        tuple: (fig, ax) matplotlib figure and axis objects
    """
    fig, ax = plt.subplots(figsize=figsize)

    # Plot each satellite's elevation
    for idx, row in df.iterrows():
        sat_id = row[sat_id_col]
        ts_data = row[timeseries_col]

        if ts_data is not None and not ts_data.empty:
            # Extract time and elevation
            times = ts_data[time_col]
            elevations = ts_data[elev_col]

            # Plot with label
            ax.plot(times, elevations, marker='.', markersize=3,
                    linewidth=1, label=sat_id, alpha=0.8)

    ax.set_xlabel('Time', fontsize=12)
    ax.set_ylabel('Elevation (degrees)', fontsize=12)
    ax.set_title('Satellite Elevation Angles Over Time', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(ylim)

    # Add horizon line
    ax.axhline(y=0, color='r', linestyle='--', linewidth=1, alpha=0.5, label='Horizon')

    # Legend outside plot area
    ax.legend(bbox_to_anchor=(1.02, 1), loc='upper left', fontsize=9)

    plt.tight_layout()
    return fig, ax


def plot_azimuth_time(df, time_col='time', azim_col='azimuth',
                      sat_id_col='sat_id', timeseries_col='timeseries',
                      figsize=(14, 6), ylim=(0, 360)):
    """
    Plot azimuth angle vs time for all satellites.

    Parameters:
        df (pd.DataFrame): Main dataframe where each row is a satellite
        time_col (str): Column name for time in nested dataframes
        azim_col (str): Column name for azimuth in nested dataframes
        sat_id_col (str): Column name for satellite ID in main dataframe
        timeseries_col (str): Column name containing nested timeseries dataframes
        figsize (tuple): Figure size (width, height)
        ylim (tuple): Y-axis limits for azimuth

    Returns:
        tuple: (fig, ax) matplotlib figure and axis objects
    """
    fig, ax = plt.subplots(figsize=figsize)

    # Plot each satellite's azimuth
    for idx, row in df.iterrows():
        sat_id = row[sat_id_col]
        ts_data = row[timeseries_col]

        if ts_data is not None and not ts_data.empty:
            # Extract time and azimuth
            times = ts_data[time_col]
            azimuths = ts_data[azim_col]

            # Plot with label
            ax.plot(times, azimuths, marker='.', markersize=3,
                    linewidth=1, label=sat_id, alpha=0.8)

    ax.set_xlabel('Time', fontsize=12)
    ax.set_ylabel('Azimuth (degrees)', fontsize=12)
    ax.set_title('Satellite Azimuth Angles Over Time', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(ylim)

    # Add cardinal direction markers
    ax.axhline(y=0, color='gray', linestyle=':', linewidth=1, alpha=0.5)
    ax.axhline(y=90, color='gray', linestyle=':', linewidth=1, alpha=0.5)
    ax.axhline(y=180, color='gray', linestyle=':', linewidth=1, alpha=0.5)
    ax.axhline(y=270, color='gray', linestyle=':', linewidth=1, alpha=0.5)

    # Add text labels for directions
    ax.text(ax.get_xlim()[1], 0, ' N', va='center', fontsize=9, alpha=0.6)
    ax.text(ax.get_xlim()[1], 90, ' E', va='center', fontsize=9, alpha=0.6)
    ax.text(ax.get_xlim()[1], 180, ' S', va='center', fontsize=9, alpha=0.6)
    ax.text(ax.get_xlim()[1], 270, ' W', va='center', fontsize=9, alpha=0.6)

    # Legend outside plot area
    ax.legend(bbox_to_anchor=(1.02, 1), loc='upper left', fontsize=9)

    plt.tight_layout()
    return fig, ax


def plot_skyplot(df, azim_col='azimuth', elev_col='elevation',
                 sat_id_col='sat_id', timeseries_col='timeseries',
                 figsize=(10, 10), elev_mask=10):
    """
    Create a polar skyplot showing satellite tracks.

    Azimuth is mapped to angle (0° = North = top)
    Elevation is mapped to radius (0° = edge, 90° = center)

    Parameters:
        df (pd.DataFrame): Main dataframe where each row is a satellite
        azim_col (str): Column name for azimuth in nested dataframes
        elev_col (str): Column name for elevation in nested dataframes
        sat_id_col (str): Column name for satellite ID in main dataframe
        timeseries_col (str): Column name containing nested timeseries dataframes
        figsize (tuple): Figure size (width, height)
        elev_mask (float): Minimum elevation angle to plot (degrees)

    Returns:
        tuple: (fig, ax) matplotlib figure and axis objects
    """
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='polar')

    # Configure polar plot
    ax.set_theta_zero_location('N')  # 0° at top (North)
    ax.set_theta_direction(-1)  # Clockwise

    # Plot each satellite's track
    for idx, row in df.iterrows():
        sat_id = row[sat_id_col]
        ts_data = row[timeseries_col]

        if ts_data is not None and not ts_data.empty:
            # Extract azimuth and elevation
            azimuths = ts_data[azim_col].values
            elevations = ts_data[elev_col].values

            # Filter by elevation mask
            mask = elevations >= elev_mask
            azimuths_filtered = azimuths[mask]
            elevations_filtered = elevations[mask]

            if len(azimuths_filtered) > 0:
                # Convert to polar coordinates
                # Azimuth -> theta (in radians)
                theta = np.radians(azimuths_filtered)
                # Elevation -> radius (90° - elevation for correct mapping)
                r = 90 - elevations_filtered

                # Plot track
                ax.plot(theta, r, marker='.', markersize=2,
                        linewidth=1, label=sat_id, alpha=0.7)

                # Mark start and end points
                ax.plot(theta[0], r[0], 'o', markersize=8, alpha=0.8)  # Start
                ax.plot(theta[-1], r[-1], 's', markersize=6, alpha=0.8)  # End

    # Set radial limits (0° elevation at edge, 90° at center)
    ax.set_ylim(0, 90)

    # Set radial tick labels (elevation angles)
    ax.set_yticks([0, 15, 30, 45, 60, 75, 90])
    ax.set_yticklabels(['90°', '75°', '60°', '45°', '30°', '15°', '0°'])

    # Add elevation circles
    for elev in [30, 60]:
        circle = Circle((0, 0), 90 - elev, transform=ax.transData._b,
                        fill=False, edgecolor='gray', linewidth=1,
                        linestyle='--', alpha=0.3)
        ax.add_patch(circle)

    # Add cardinal direction labels
    ax.set_xticks(np.radians([0, 45, 90, 135, 180, 225, 270, 315]))
    ax.set_xticklabels(['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'])

    ax.set_title(f'Satellite Sky Plot (Elevation Mask: {elev_mask}°)',
                 fontsize=14, fontweight='bold', pad=20)
    ax.legend(bbox_to_anchor=(1.15, 1.0), loc='upper left', fontsize=9)

    plt.tight_layout()
    return fig, ax


def plot_elevation_azimuth_combined(df, time_col='time', elev_col='elevation',
                                    azim_col='azimuth', sat_id_col='sat_id',
                                    timeseries_col='timeseries', figsize=(14, 10)):
    """
    Create combined elevation and azimuth plots in a single figure.

    Parameters:
        df (pd.DataFrame): Main dataframe where each row is a satellite
        time_col (str): Column name for time in nested dataframes
        elev_col (str): Column name for elevation in nested dataframes
        azim_col (str): Column name for azimuth in nested dataframes
        sat_id_col (str): Column name for satellite ID in main dataframe
        timeseries_col (str): Column name containing nested timeseries dataframes
        figsize (tuple): Figure size (width, height)

    Returns:
        tuple: (fig, axes) matplotlib figure and axes objects
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=figsize, sharex=True)

    # Plot each satellite
    for idx, row in df.iterrows():
        sat_id = row[sat_id_col]
        ts_data = row[timeseries_col]

        if ts_data is not None and not ts_data.empty:
            times = ts_data[time_col]
            elevations = ts_data[elev_col]
            azimuths = ts_data[azim_col]

            # Plot elevation
            ax1.plot(times, elevations, marker='.', markersize=3,
                     linewidth=1, label=sat_id, alpha=0.8)

            # Plot azimuth
            ax2.plot(times, azimuths, marker='.', markersize=3,
                     linewidth=1, label=sat_id, alpha=0.8)

    # Configure elevation plot
    ax1.set_ylabel('Elevation (degrees)', fontsize=12)
    ax1.set_title('Satellite Elevation and Azimuth Over Time',
                  fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, 90)
    ax1.axhline(y=0, color='r', linestyle='--', linewidth=1, alpha=0.5)
    ax1.legend(bbox_to_anchor=(1.02, 1), loc='upper left', fontsize=9)

    # Configure azimuth plot
    ax2.set_xlabel('Time', fontsize=12)
    ax2.set_ylabel('Azimuth (degrees)', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(0, 360)
    ax2.axhline(y=90, color='gray', linestyle=':', linewidth=1, alpha=0.3)
    ax2.axhline(y=180, color='gray', linestyle=':', linewidth=1, alpha=0.3)
    ax2.axhline(y=270, color='gray', linestyle=':', linewidth=1, alpha=0.3)

    plt.tight_layout()
    return fig, (ax1, ax2)


def plot_satellite_visibility(df, time_col='time', elev_col='elevation',
                              sat_id_col='sat_id', timeseries_col='timeseries',
                              elev_mask=10, figsize=(14, 8)):
    """
    Create a visibility timeline plot showing when each satellite is above elevation mask.

    Parameters:
        df (pd.DataFrame): Main dataframe where each row is a satellite
        time_col (str): Column name for time in nested dataframes
        elev_col (str): Column name for elevation in nested dataframes
        sat_id_col (str): Column name for satellite ID in main dataframe
        timeseries_col (str): Column name containing nested timeseries dataframes
        elev_mask (float): Minimum elevation angle for visibility (degrees)
        figsize (tuple): Figure size (width, height)

    Returns:
        tuple: (fig, ax) matplotlib figure and axis objects
    """
    fig, ax = plt.subplots(figsize=figsize)

    sat_ids = []
    y_positions = []

    for idx, row in enumerate(df.iterrows()):
        row_idx, row_data = row
        sat_id = row_data[sat_id_col]
        ts_data = row_data[timeseries_col]

        if ts_data is not None and not ts_data.empty:
            times = ts_data[time_col]
            elevations = ts_data[elev_col].values

            # Find visible periods
            visible = elevations >= elev_mask

            # Plot visible periods
            for i in range(len(visible)):
                if visible[i]:
                    ax.plot([times.iloc[i], times.iloc[i]],
                            [idx - 0.4, idx + 0.4],
                            color='blue', linewidth=2, alpha=0.6)

            sat_ids.append(sat_id)
            y_positions.append(idx)

    ax.set_yticks(y_positions)
    ax.set_yticklabels(sat_ids)
    ax.set_xlabel('Time', fontsize=12)
    ax.set_ylabel('Satellite', fontsize=12)
    ax.set_title(f'Satellite Visibility Timeline (Elevation Mask: {elev_mask}°)',
                 fontsize=14, fontweight='bold')
    ax.grid(True, axis='x', alpha=0.3)

    plt.tight_layout()
    return fig, ax


if __name__ == "__main__":
    # Example usage with synthetic data
    print("Satellite Elevation and Azimuth Plotting Module")
    print("=" * 60)
    print("\nExample usage:")
    print("""
    import pandas as pd
    from plot_satellite_angles import (
        plot_elevation_time, 
        plot_azimuth_time,
        plot_skyplot,
        plot_elevation_azimuth_combined
    )

    # Assuming you have a nested dataframe 'df' where:
    # - Each row is a satellite
    # - 'sat_id' column contains satellite identifier
    # - 'timeseries' column contains a dataframe with time, elevation, azimuth

    # Plot elevation over time
    fig, ax = plot_elevation_time(df, sat_id_col='sat_id', 
                                   timeseries_col='timeseries')
    plt.show()

    # Plot azimuth over time
    fig, ax = plot_azimuth_time(df, sat_id_col='sat_id',
                                timeseries_col='timeseries')
    plt.show()

    # Create sky plot
    fig, ax = plot_skyplot(df, sat_id_col='sat_id',
                          timeseries_col='timeseries')
    plt.show()

    # Combined elevation and azimuth plot
    fig, axes = plot_elevation_azimuth_combined(df, sat_id_col='sat_id',
                                                timeseries_col='timeseries')
    plt.show()
    """)