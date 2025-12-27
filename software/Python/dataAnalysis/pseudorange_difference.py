import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sqlite3

c = 299792458.0 # Speed of light (m/s)

diff = 0.0001540337537880987
ormdDB = "../../data/RINEX_data/ORMD/2025-11-25/ormd3290_GPS.db"
zedf9pDB = "../../data/ubx_data/2025-11-25/GNSS001_5.db"

ormdDB_con = sqlite3.connect(ormdDB)
zedf9pDB_con = sqlite3.connect(zedf9pDB)

ormdData = pd.read_sql_query("SELECT * FROM G01", ormdDB_con)
ormdDB_con.commit()
ormdDB_con.close()

zedf9pData = pd.read_sql_query("SELECT * FROM 'G01_L1 C'", zedf9pDB_con)
zedf9pDB_con.commit()
zedf9pDB_con.close()

ormdData_mask = (ormdData['rcvTOW'] >= zedf9pData['rcvTOW'][0])
ormdData = ormdData.loc[ormdData_mask]

rcvTow_new = []
zedf9p_pseudorange_list = []
ormd_pseudorange_list = []
range_diff_list = []
zedf9p_trop_new_list = []
ormd_trop_list = []
zedf9p_gpsTow_diff_list = []


# Calculate ZED-F9P receiver time offset
for _ in range(len(zedf9pData)):
    zedf9p_rcvTow = zedf9pData.iloc[_]['rcvTOW']
    closest_ormd_rcvTow = ormdData.iloc[(ormdData['rcvTOW'] - zedf9p_rcvTow).abs().argsort()[:1]]
    ormd_rcvTow = closest_ormd_rcvTow['rcvTOW'].values[0]

    if abs(zedf9p_rcvTow - ormd_rcvTow) > 1:
        continue

    ormdData_new = ormdData.set_index(['rcvTOW'])

    zedf9p_gpsTow = zedf9pData.iloc[_]['gpsTOW']
    zedf9p_pseudorange= zedf9pData.iloc[_]['pseudorange']
    zedf9p_georange = zedf9pData.iloc[_]['geoRange']
    zedf9p_rcvClockB = zedf9pData.iloc[_]['rcvClockBias']
    zedf9p_svClockB = zedf9pData.iloc[_]['satClockBias']
    zedf9p_relBias = zedf9pData.iloc[_]['relBias']
    ormd_trop = ormdData_new.loc[ormd_rcvTow]['troposphericDelay']

    print(zedf9p_rcvTow - zedf9p_gpsTow)

    zedf9p_gpsTow_diff_list.append(zedf9p_rcvTow - zedf9p_gpsTow)

    rcvTow_new.append(zedf9p_rcvTow)

    zedf9p_pseudorange_new = zedf9p_pseudorange - (diff * c)

    biases = zedf9p_rcvClockB + zedf9p_svClockB + zedf9p_relBias
    zedf9p_trop_new = zedf9p_pseudorange_new - zedf9p_georange - biases

    zedf9p_trop_new_list.append(zedf9p_trop_new)

    ormd_trop_list.append(ormd_trop)

    zedf9p_pseudorange_list.append(zedf9p_pseudorange_new)
    #ormd_pseudorange_list.append(ormd_pseudorange)
    #range_diff_list.append((ormd_pseudorange - zedf9p_pseudorange_new) / c)

    #print((ormd_pseudorange - zedf9p_pseudorange_new) / c)


print(np.average(zedf9p_gpsTow_diff_list))

#plt.plot(rcvTow_new, zedf9p_gpsTow_diff_list)
plt.plot(rcvTow_new, zedf9p_trop_new_list)
#.plot(rcvTow_new, ormd_trop_list)
plt.show()

# plt.figure(figsize=(10, 6))
# plt.title("Tropospheric Delay - G31")
# plt.plot(ormdData['rcvTOW'], ormdData['troposphericDelay'])
# plt.plot(zedf9pData['rcvTOW'], zedf9pData['troposphericDelay'])
# plt.xlabel("rcvTOW (s)")
# plt.ylabel("Tropospheric Delay (m)")
# plt.grid()
# plt.tight_layout()
# plt.show()