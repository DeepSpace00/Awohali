import math
import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

_DATA = Path(__file__).parent.parent / "data"

rinex_database = _DATA / "RINEX_data/ORMD/2026-02-27/ormd0580_26o_GPS_Galileo.db"


OUTPUT_PATH = _DATA / "figures/feb_27_mar"
OUTPUT_PATH.mkdir(parents=True, exist_ok=True)

conn = sqlite3.connect(rinex_database)
cursor = conn.cursor()

# Get all table names
cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
tables = cursor.fetchall()
sat_ids = [row[0] for row in tables if row[0][0] in ('E', 'G')]

print(sat_ids)

def plot_data(rinex_data):
    fig, ax = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(20, 20),
        layout="constrained"
    )

    ax[0].plot(rinex_data['rcvTOW'], rinex_data['troposphericDelay'], label='RINEX tropoDelay', color='green')
    ax0r = ax[0].twinx()
    ax0r.plot(rinex_data['rcvTOW'], rinex_data['elevation'], label='RINEX Elevation', color='blue')
    ax[0].set_ylabel("RINEX Tropospheric Delay (m)")
    ax0r.set_ylabel("RINEX Elevation (deg)")
    ax[0].set_title("Tropospheric Delay")
    lines = ax[0].get_lines() + ax0r.get_lines()
    labels = [l.get_label() for l in lines]
    ax[0].legend(lines, labels, loc="upper right")

    ax[0].set_xlim(x_min, x_max)
    ax[0].set_ylim(rinex_data['troposphericDelay'].min(), rinex_data['troposphericDelay'].max())

    ax[0].grid()

    ax[1].plot(rinex_data['rcvTOW'], rinex_data['ztd'], label='RINEX ZTD', color='green')
    ax1r = ax[1].twinx()
    ax1r.plot(rinex_data['rcvTOW'], rinex_data['elevation'], label='RINEX Elevation', color='blue')
    ax[1].set_ylabel("RINEX Zenith Time Delay (mm)")
    ax1r.set_ylabel("RINEX Elevation (deg)")
    ax[1].set_title("Zenith Time Delay (ZTD)")
    lines = ax[1].get_lines() + ax1r.get_lines()
    labels = [l.get_label() for l in lines]
    ax[1].legend(lines, labels, loc="upper right")

    ax[1].set_ylim(rinex_data['ztd'].min(), rinex_data['ztd'].max())

    ax[1].grid()

    ax[2].plot(rinex_data['rcvTOW'], rinex_data['pwv'], label='RINEX PWV', color='green')
    ax2r = ax[2].twinx()
    ax2r.plot(rinex_data['rcvTOW'], rinex_data['elevation'], label='RINEX Elevation', color='blue')
    ax[2].set_ylabel("RINEX Precipitable Water Vapor (mm)")
    ax2r.set_ylabel("RINEX Elevation (deg)")
    ax[2].set_title("Precipitable Water Vapor (PWV)")
    lines = ax[2].get_lines() + ax2r.get_lines()
    labels = [l.get_label() for l in lines]
    ax[2].legend(lines, labels, loc="upper right")

    ax[2].set_ylim(rinex_data['pwv'].min(), rinex_data['pwv'].max())

    ax[2].set_xlabel("Receiver TOW (s)")

    fig.suptitle(f"Tropospheric Products - {sat_id}")

    ax[2].grid()

    plt.savefig(f"{OUTPUT_PATH}/{sat_id}.png")


for sat_id in sat_ids:
    rinex_data = pd.read_sql(f"SELECT * FROM '{sat_id}'", con=conn)

    if rinex_data.shape[0] == 0:
        continue

    rinex_data = rinex_data[rinex_data['elevation'] >= 20]


    x_min = rinex_data['rcvTOW'].min()
    x_max = rinex_data['rcvTOW'].max()

    if math.isnan(x_min) or math.isnan(x_max):
        continue

    rinex_data = rinex_data[(rinex_data['rcvTOW'] >= x_min) & (rinex_data['rcvTOW'] <= x_max)]

    plot_data(rinex_data)

conn.close()

plt.show()