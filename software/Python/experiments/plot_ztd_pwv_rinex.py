import sqlite3
from datetime import datetime, timezone
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

_DATA = Path(__file__).parent.parent / "data"

rinex_database = _DATA / "RINEX_data/ORMD/concat_data_2.db"

OUTPUT_PATH = _DATA / "figures/ormd_2"
OUTPUT_PATH.mkdir(parents=True, exist_ok=True)

GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
GPS_LEAP_SECONDS = 18  # current as of 2026

def gps_to_utc(week, tow):
    gps_seconds = week * 604800 + tow - GPS_LEAP_SECONDS
    return pd.to_datetime(GPS_EPOCH + pd.to_timedelta(gps_seconds, unit='s'))

rinex_con = sqlite3.connect(rinex_database)
rinex_data = pd.read_sql(f"select * from epoch_summary", con=rinex_con)
rinex_con.close()

x_min = rinex_data['rcvTOW'].min()
x_max = rinex_data['rcvTOW'].max()

rinex_data = rinex_data[(rinex_data['rcvTOW'] >= x_min) & (rinex_data['rcvTOW'] <= x_max)]
rinex_data['utc_time'] = gps_to_utc(2407, rinex_data['rcvTOW'])

fig, ax = plt.subplots(
    nrows=2,
    ncols=1,
    sharex=True,
    figsize=(20, 20),
    layout="constrained"
)



ax[0].plot(rinex_data['utc_time'], rinex_data['ZTD_m'], label='RINEX ZTD', color='green')
ax[0].set_ylabel("RINEX ZTD (m)")
ax[0].set_title("Zenith Time Delay (ZTD)")
lines = ax[0].get_lines()
labels = [l.get_label() for l in lines]
ax[0].legend(lines, labels, loc="upper right")

ax[0].set_xlim(rinex_data['utc_time'].min(), rinex_data['utc_time'].max())
ax[0].set_ylim(rinex_data['ZTD_m'].min(), rinex_data['ZTD_m'].max())

plt.grid()

ax[1].plot(rinex_data['utc_time'], rinex_data['PWV_mm'], label='RINEX PWV', color='green')
ax[1].set_ylabel("RINEX PWV (mm)")
ax[1].set_title("Precipitable Water Vapor (PWV)")
lines = ax[1].get_lines()
labels = [l.get_label() for l in lines]
ax[1].legend(lines, labels, loc="upper right")

ax[1].set_ylim(rinex_data['PWV_mm'].min(), rinex_data['PWV_mm'].max())

plt.grid()

ax[1].set_xlabel("Time (UTC)")

fig.suptitle("Tropospheric Products - epoch_summary")

plt.savefig(f"{OUTPUT_PATH}/epoch_summary.png")

plt.show()