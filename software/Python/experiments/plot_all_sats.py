import math
import sqlite3
import pandas as pd
from pathlib import Path
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# PGF/LaTeX rendering configuration.



_DATA = Path(__file__).parent.parent / "data"

ubx_database = _DATA / "test_raw.db"
rinex_database = _DATA / "RINEX_pwv_test_new_filter.db"

ubx_con = sqlite3.connect(ubx_database)
rinex_con = sqlite3.connect(rinex_database)

sql_query = """SELECT name FROM sqlite_master 
    WHERE type='table';"""

ubx_cursor = ubx_con.cursor()
ubx_cursor.execute(sql_query)

rinex_cursor = rinex_con.cursor()
rinex_cursor.execute(sql_query)

ubx_tables = ubx_cursor.fetchall()
ubx_tables = ubx_tables[:-1]

rinex_tables = rinex_cursor.fetchall()
rinex_tables = rinex_tables[:-1]

# print(rinex_tables)

tables = []
i = 0
while i < len(ubx_tables):
    if rinex_tables.count(ubx_tables[i]) > 0:
        tables.append(ubx_tables[i])
    i += 1

table_new = []
for table in tables:
    table_new = table[0]

chars_to_remove = ["(", ")", ","]
strings_to_remove = ["G31", "E16", "E19", "E25", "E34", "G02", "G03", "G04", "G08", "G09", "G12", "G16", "G23",
                     "G27", "G29"]

remove_set = set(chars_to_remove)
result = ["".join(ch for ch in s if ch not in remove_set) for s in tables]

result = [s for s in result if not any(r in s for r in strings_to_remove)]

#fig, ax = plt.subplots()

#tables = ["E06", "E09", "E31", "G26", "G28", "G31"]
# tables = ["epoch_summary"]

fig = plt.figure(figsize=(12, 8))  # ← width, height in inches

for table in result:
    ubx_data = pd.read_sql(f"select * from {table}", con=ubx_con)
    rinex_data = pd.read_sql(f"select * from {table}", con=rinex_con)

    x_min = max(ubx_data['rcvTOW'].min(), rinex_data['rcvTOW'].min())
    x_max = min(ubx_data['rcvTOW'].max(), rinex_data['rcvTOW'].max())

    ubx_data = ubx_data[ubx_data['elevation'] >= 20].reset_index(drop=True)
    rinex_data = rinex_data[rinex_data['elevation'] >= 20].reset_index(drop=True)

    trop_delay_L1 = (
                ubx_data['pseudorange_combined'] - ubx_data['geoRange'] + ubx_data['satClockBias'] + ubx_data['relBias']
                - ubx_data['rcv_clock_m'])

    trop_delay_L5 = (ubx_data['pseudorange_L5'] - ubx_data['geoRange'] + ubx_data['satClockBias'] + ubx_data['relBias']
                     - ubx_data['rcv_clock_m'])

    fig, ax1 = plt.subplots(figsize=(12, 8))
    ax2 = ax1.twinx()

    ax1.plot(ubx_data['rcvTOW'], trop_delay_L1, label=f'UBX {table}_L1', color='tab:green')
    ax2.plot(rinex_data['rcvTOW'], rinex_data['troposphericDelay'], label=f'RINEX {table}', color='tab:blue')

    ax1.set_ylim([trop_delay_L1.min(), trop_delay_L1.max()])
    ax2.set_ylim([rinex_data['troposphericDelay'].min(), rinex_data['troposphericDelay'].max()])

    ax1.set_xlim(x_min, x_max)
    ax1.set_ylabel('troposphericDelay L1 (m)')
    ax2.set_ylabel('troposphericDelay L5 (m)')
    ax1.set_xlabel("Receiver Time of Week (s)")

    # Combine legends from both axes
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2)

    plt.title("Tropospheric Delay")
    plt.savefig(f"{_DATA}/figures/new/{table}.png")
    plt.close(fig)

ubx_con.close()
rinex_con.close()