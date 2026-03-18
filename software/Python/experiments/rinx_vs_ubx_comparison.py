import math
import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

_DATA = Path(__file__).parent.parent / "data"

ubx_database = _DATA / "ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_testing_fast.db"
rinex_database = _DATA / "RINEX_data/ORMD/2025-11-25/RINEX_pwv_test_new.db"

tables = ['E05', 'E06', 'E09', 'E11', 'E16', 'E23', 'E25', 'E31', 'E36', 'G03',
         'G04', 'G10', 'G16', 'G25', 'G26', 'G27', 'G28', 'G32']

def plot_data(rinex_data, ubx_data):
    fig, ax = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(20, 20),
        layout="constrained"
    )

    ax[0].plot(rinex_data['rcvTOW'], rinex_data['troposphericDelay'], label='RINEX tropoDelay', color='green')
    ax0r = ax[0].twinx()
    ax0r.plot(ubx_data['rcvTOW'], ubx_data['troposphericDelay'], label='UBX tropoDelay', color='blue')
    ax[0].set_ylabel("RINEX Tropospheric Delay (m)")
    ax0r.set_ylabel("UBX Tropospheric Delay (m)")
    ax[0].set_title("Tropospheric Delay")
    lines = ax[0].get_lines() + ax0r.get_lines()
    labels = [l.get_label() for l in lines]
    ax[0].legend(lines, labels, loc="upper right")

    ax[0].set_xlim(x_min, x_max)
    ax[0].set_ylim(rinex_data['troposphericDelay'].min(), rinex_data['troposphericDelay'].max())
    ax0r.set_ylim(ubx_data['troposphericDelay'].min(), ubx_data['troposphericDelay'].max())

    plt.grid()

    ax[1].plot(rinex_data['rcvTOW'], rinex_data['ztd'], label='RINEX ZTD', color='green')
    ax1r = ax[1].twinx()
    ax1r.plot(ubx_data['rcvTOW'], ubx_data['ztd'], label='UBX ZTD', color='blue')
    ax[1].set_ylabel("RINEX Zenith Time Delay (mm)")
    ax1r.set_ylabel("UBX Zenith Time Delay (mm)")
    ax[1].set_title("Zenith Time Delay (ZTD)")
    lines = ax[1].get_lines() + ax1r.get_lines()
    labels = [l.get_label() for l in lines]
    ax[1].legend(lines, labels, loc="upper right")

    ax[1].set_ylim(rinex_data['ztd'].min(), rinex_data['ztd'].max())
    ax1r.set_ylim(ubx_data['ztd'].min(), ubx_data['ztd'].max())

    plt.grid()

    ax[2].plot(rinex_data['rcvTOW'], rinex_data['pwv'], label='RINEX PWV', color='green')
    ax2r = ax[2].twinx()
    ax2r.plot(ubx_data['rcvTOW'], ubx_data['pwv'], label='UBX PWV', color='blue')
    ax[2].set_ylabel("RINEX Precipitable Water Vapor (mm)")
    ax2r.set_ylabel("UBX Precipitable Water Vapor (mm)")
    ax[2].set_title("Precipitable Water Vapor (PWV)")
    lines = ax[2].get_lines() + ax2r.get_lines()
    labels = [l.get_label() for l in lines]
    ax[2].legend(lines, labels, loc="upper right")

    ax[2].set_ylim(rinex_data['pwv'].min(), rinex_data['pwv'].max())
    ax2r.set_ylim(ubx_data['pwv'].min(), ubx_data['pwv'].max())

    ax[2].set_xlabel("Receiver TOW (s)")

    fig.suptitle(f"Tropospheric Products - {table}")

    plt.grid()

    plt.savefig(f"{_DATA}/figures/troposphericProducts_test/{table}.png")


for table in tables:
    ubx_con = sqlite3.connect(ubx_database)
    ubx_data = pd.read_sql(f"select * from '{table}'", con=ubx_con)
    ubx_con.close()

    rinex_con = sqlite3.connect(rinex_database)
    rinex_data = pd.read_sql(f"select * from '{table}'", con=rinex_con)
    rinex_con.close()

    if ubx_data.shape[0] == 0 or rinex_data.shape[0] == 0:
        continue

    ubx_data = ubx_data[ubx_data['elevation'] >= 20]
    rinex_data = rinex_data[rinex_data['elevation'] >= 20]


    x_min = max(ubx_data['rcvTOW'].min(), rinex_data['rcvTOW'].min())
    x_max = min(ubx_data['rcvTOW'].max(), rinex_data['rcvTOW'].max())

    if math.isnan(x_min) or math.isnan(x_max):
        continue

    plot_data(rinex_data,ubx_data)

plt.show()