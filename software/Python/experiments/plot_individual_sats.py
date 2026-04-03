import math
import sqlite3
import pandas as pd
from pathlib import Path
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# PGF/LaTeX rendering configuration.

c = 299792458

_DATA = Path(__file__).parent.parent / "data"

ubx_database = _DATA / "test_raw.db"
rinex_database = _DATA / "RINEX_pwv.db"

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

print(ubx_tables)

# tables = []
# i = 0
# while i < len(ubx_tables):
#     if rinex_tables.count(ubx_tables[i]) > 0:
#         tables.append(ubx_tables[i])
#     i += 1

# table_new = []
# for table in tables:
#     table_new = table[0]

chars_to_remove = ["(", ")", ","]
strings_to_remove = ["G31", "E16", "E19", "E25", "E34", "G02", "G03", "G04", "G08", "G09", "G12", "G16", "G23",
                     "G27", "G29"]

remove_set = set(chars_to_remove)
result = ["".join(ch for ch in s if ch not in remove_set) for s in ubx_tables]

print(result)

result = [s for s in result if not any(r in s for r in strings_to_remove)]

#fig, ax = plt.subplots()

#tables = ["E09"]
#tables = ["G26_L1", "G28_L1", "G31_L1 C"]
# tables = ["epoch_summary"]

fig = plt.figure(figsize=(12, 8))  # ← width, height in inches

for table in result:
    ubx_data = pd.read_sql(f"select * from {table}", con=ubx_con)
    #rinex_data = pd.read_sql(f"select * from {table}", con=rinex_con)

    #x_min = max(ubx_data['rcvTOW'].min(), rinex_data['rcvTOW'].min())
    #x_max = min(ubx_data['rcvTOW'].max(), rinex_data['rcvTOW'].max())

    ubx_data = ubx_data[ubx_data['elevation'] >= 20].reset_index(drop=True)

    diff = ubx_data['pseudorange_combined'] - ubx_data['geoRange']
    trop_delay_L1 = (ubx_data['pseudorange_combined'] - ubx_data['geoRange'] + ubx_data['satClockBias'] + ubx_data['relBias']
                  - ubx_data['rcv_clock_m'])

    trop_delay_L5 = (ubx_data['pseudorange_L5'] - ubx_data['geoRange'] + ubx_data['satClockBias'] + ubx_data['relBias']
                     - ubx_data['rcv_clock_m'])

    plt.plot(ubx_data['rcvTOW'], trop_delay_L1, label=f'UBX {table}_L1')
    #plt.plot(ubx_data['rcvTOW'], trop_delay_L5, label=f'UBX {table}_L5')


ubx_con.close()
rinex_con.close()

#plt.set_xlim(x_min, x_max)
#plt.set_ylabel('RINEX PWV (mm)')
#plt.set_ylabel('UBX PWV (mm)')
#plt.set_xlabel("Receiver Time of Week (s)")

plt.title("Tropospheric Delay")
#plt.savefig(f"{_DATA}/figures/new/{table}_pwv_filtered.png")
#plt.close(fig)
plt.legend()
plt.show()