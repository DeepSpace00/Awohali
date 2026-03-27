import math
import sqlite3
import pandas as pd
from pathlib import Path
import matplotlib
matplotlib.use('pgf')
import matplotlib.pyplot as plt
import numpy as np

# PGF/LaTeX rendering configuration.
matplotlib.rcParams.update({
    'pgf.texsystem': 'pdflatex',
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
    'font.size': 30,
    'axes.titlesize': 30,
    'axes.labelsize': 30,
    'xtick.labelsize': 30,
    'ytick.labelsize': 30,
    'legend.fontsize': 30,
    'figure.titlesize': 30,
    'pgf.preamble': r"""
        \usepackage[utf8]{inputenc}
        \usepackage[T1]{fontenc}
    """,
})


_DATA = Path(__file__).parent.parent / "data"

ubx_database = _DATA / "ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_filter_gpsTOW_fixed.db"
rinex_database = _DATA / "RINEX_data/ORMD/2025-11-25/RINEX_pwv_test_new_filter.db"

ubx_con = sqlite3.connect(ubx_database)
ubx_data = pd.read_sql(f"select * from epoch_summary", con=ubx_con)
ubx_con.close()

rinex_con = sqlite3.connect(rinex_database)
rinex_data = pd.read_sql(f"select * from epoch_summary", con=rinex_con)
rinex_con.close()

x_min = max(ubx_data['rcvTOW'].min(), rinex_data['rcvTOW'].min())
x_max = min(ubx_data['rcvTOW'].max(), rinex_data['rcvTOW'].max())

ubx_data = ubx_data[(ubx_data['rcvTOW'] >= x_min) & (ubx_data['rcvTOW'] <= x_max)]
rinex_data = rinex_data[(rinex_data['rcvTOW'] >= x_min) & (rinex_data['rcvTOW'] <= x_max)]

fig, ax = plt.subplots(
    nrows=2,
    ncols=1,
    sharex=True,
    figsize=(20, 20),
    layout="constrained"
)

ax[0].plot(rinex_data['rcvTOW'], rinex_data['ZTD_m'], label='RINEX ZTD', color='green', linewidth=3)
ax0r = ax[0].twinx()
ax0r.plot(ubx_data['rcvTOW'], ubx_data['ZTD_m'], label='UBX ZTD', color='blue', linewidth=3)
ax[0].set_ylabel("RINEX ZTD (m)")
ax0r.set_ylabel("UBX ZTD (m)")
ax[0].set_title("Zenith Time Delay (ZTD)")
lines = ax[0].get_lines() + ax0r.get_lines()
labels = [l.get_label() for l in lines]
ax[0].legend(lines, labels, loc="upper right")

ax[0].set_xlim(x_min, x_max)
ax[0].set_ylim(rinex_data['ZTD_m'].min(), rinex_data['ZTD_m'].max())
ax0r.set_ylim(ubx_data['ZTD_m'].min(), ubx_data['ZTD_m'].max())
ax[0].set_yticks(np.arange(1.5, 3.6, 0.5))

ax[1].plot(rinex_data['rcvTOW'], rinex_data['PWV_mm'], label='RINEX PWV', color='green', linewidth=3)
ax1r = ax[1].twinx()
ax1r.plot(ubx_data['rcvTOW'], ubx_data['PWV_mm'], label='UBX PWV', color='blue', linewidth=3)
ax[1].set_ylabel("RINEX PWV (mm)")
ax1r.set_ylabel("UBX PWV (mm)")
ax[1].set_title("Precipitable Water Vapor (PWV)")
lines = ax[1].get_lines() + ax1r.get_lines()
labels = [l.get_label() for l in lines]
ax[1].legend(lines, labels, loc="upper right")

ax[1].set_ylim(rinex_data['PWV_mm'].min(), rinex_data['PWV_mm'].max())
ax1r.set_ylim(ubx_data['PWV_mm'].min(), ubx_data['PWV_mm'].max())
ax[1].set_yticks(np.arange(-1, 8, 1))


ax[1].set_xticks(np.arange(226800.0, 236000.0, 1000))

ax[1].set_xlabel("Receiver TOW (s)")

fig.suptitle("GNSS Tropospheric Products")

plt.savefig(f"{_DATA}/figures/troposphericProducts_test_3/epoch_summary_3.png")