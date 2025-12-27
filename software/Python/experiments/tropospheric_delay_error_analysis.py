import matplotlib.pyplot as plt
import pandas as pd
import sqlite3

c = 299792458.0 # Speed of light (m/s)

svId = "G29_L1 C"

ubx_database = "../data/ubx_data/2025-11-25/GNSS001.db"

ubx_con = sqlite3.connect(ubx_database)
ubx_data = pd.read_sql(f"select * from '{svId}'", con=ubx_con)
ubx_con.close()

# Create analysis lists

# Distance parameters converted to time (nanoseconds)
geoRange_ns_list = []
satClockBias_ns_list = []
rcvClockBias_ns_list = []
relativisticBias_ns_list = []
pseudorange_ns_list = []
rangeDiff_ns_list = []
troposphericDelay_ns_list = []
total_bias_ns_list = []

# Differences converted to time (nanoseconds)
delta_gpsTOW_list = []
delta_rcvTOW_list = []
delta_t_tx_list = []
delta_geoRange_ns_list = []
delta_satClockBias_ns_list = []
delta_rcvClockBias_ns_list = []
delta_relativisticBias_ns_list = []
delta_pseudorange_ns_list = []
delta_rangeDiff_ns_list = []
delta_troposphericDelay_ns_list = []
delta_total_bias_ns_list = []

# Differences between parameters
totalClockBias_ns_list = [] # gpsTOW - rcvTOW
delta_totalClockBias_ns_list = []
totalClockBias_rangeDiff_difference_ns_list = [] # The difference between totalClockBias_ns and rangeDiff_ns

delta_totalClockBias_rangeDiff_difference_ns_list = [] # The difference between sequential totalClockBias_rangeDiff_difference_ns values

# Add data to lists
for i in range(len(ubx_data)):
    # Calculate values
    geoRange_ns = ubx_data.iloc[i]['geoRange'] / c * 1e9
    satClockBias_ns = ubx_data.iloc[i]['satClockBias'] / c * 1e9
    rcvClockBias_ns = ubx_data.iloc[i]['rcvClockBias'] / c * 1e9
    relativisticBias_ns = ubx_data.iloc[i]['relBias'] / c * 1e9
    pseudorange_ns = ubx_data.iloc[i]['pseudorange'] / c * 1e9
    rangeDiff_ns = ubx_data.iloc[i]['rangeDiff'] / c * 1e9
    troposphericDelay_ns = ubx_data.iloc[i]['troposphericDelay'] / c * 1e9
    total_bias_ns = rcvClockBias_ns - satClockBias_ns - relativisticBias_ns

    totalClockBias_ns = (ubx_data.iloc[i]['gpsTOW'] - ubx_data.iloc[i]['rcvTOW']) * 1e9
    totalClockBias_rangeDiff_difference_ns = totalClockBias_ns - rangeDiff_ns

    if i < len(ubx_data) - 1:
        delta_gpsTOW = ubx_data.iloc[i + 1]['gpsTOW'] - ubx_data.iloc[i]['gpsTOW']
        delta_rcvTOW = ubx_data.iloc[i + 1]['rcvTOW'] - ubx_data.iloc[i]['rcvTOW']
        delta_t_tx = ubx_data.iloc[i + 1]['t_tx'] - ubx_data.iloc[i]['t_tx']
        delta_geoRange_ns = (ubx_data.iloc[i + 1]['geoRange'] / c * 1e9) - geoRange_ns
        delta_satClockBias_ns = (ubx_data.iloc[i + 1]['satClockBias'] / c * 1e9) - satClockBias_ns
        delta_rcvClockBias_ns = (ubx_data.iloc[i + 1]['rcvClockBias'] / c * 1e9) - rcvClockBias_ns
        delta_relativisticBias_ns = (ubx_data.iloc[i + 1]['relBias'] / c * 1e9) - relativisticBias_ns
        delta_pseudorange_ns = (ubx_data.iloc[i + 1]['pseudorange'] / c * 1e9) - pseudorange_ns
        delta_rangeDiff_ns = (ubx_data.iloc[i + 1]['rangeDiff'] / c * 1e9) - rangeDiff_ns
        delta_troposphericDelay_ns = (ubx_data.iloc[i + 1]['troposphericDelay'] / c * 1e9) - troposphericDelay_ns

        delta_total_bias_ns = ((ubx_data.iloc[i+1]['rcvClockBias'] - ubx_data.iloc[i+1]['satClockBias'] - ubx_data.iloc[i+1]['relBias']) / c * 1e9) - total_bias_ns

        delta_totalClockBias_ns = ((ubx_data.iloc[i+1]['gpsTOW'] - ubx_data.iloc[i+1]['rcvTOW']) * 1e9) - totalClockBias_ns
        delta_totalClockBias_rangeDiff_difference_ns = ((ubx_data.iloc[i + 1]['gpsTOW'] - ubx_data.iloc[i + 1][
            'rcvTOW'] * 1e9) - (ubx_data.iloc[i + 1]['rangeDiff'] / c * 1e9)) - totalClockBias_rangeDiff_difference_ns

    else:
        delta_gpsTOW = None
        delta_rcvTOW = None
        delta_t_tx = None
        delta_geoRange_ns = None
        delta_satClockBias_ns = None
        delta_rcvClockBias_ns = None
        delta_relativisticBias_ns = None
        delta_pseudorange_ns = None
        delta_rangeDiff_ns = None
        delta_troposphericDelay_ns = None
        delta_total_bias_ns = None

        delta_totalClockBias_ns = None
        delta_totalClockBias_rangeDiff_difference_ns = None

    # Append values to lists
    geoRange_ns_list.append(geoRange_ns)
    satClockBias_ns_list.append(satClockBias_ns)
    rcvClockBias_ns_list.append(rcvClockBias_ns)
    relativisticBias_ns_list.append(relativisticBias_ns)
    pseudorange_ns_list.append(pseudorange_ns)
    rangeDiff_ns_list.append(rangeDiff_ns)
    troposphericDelay_ns_list.append(troposphericDelay_ns)
    total_bias_ns_list.append(total_bias_ns)
    totalClockBias_ns_list.append(totalClockBias_ns)
    totalClockBias_rangeDiff_difference_ns_list.append(totalClockBias_rangeDiff_difference_ns)

    delta_gpsTOW_list.append(delta_gpsTOW)
    delta_rcvTOW_list.append(delta_rcvTOW)
    delta_t_tx_list.append(delta_t_tx)
    delta_geoRange_ns_list.append(delta_geoRange_ns)
    delta_satClockBias_ns_list.append(delta_satClockBias_ns)
    delta_rcvClockBias_ns_list.append(delta_rcvClockBias_ns)
    delta_relativisticBias_ns_list.append(delta_relativisticBias_ns)
    delta_pseudorange_ns_list.append(delta_pseudorange_ns)
    delta_rangeDiff_ns_list.append(delta_rangeDiff_ns)
    delta_troposphericDelay_ns_list.append(delta_troposphericDelay_ns)
    delta_total_bias_ns_list.append(delta_total_bias_ns)
    delta_totalClockBias_ns_list.append(delta_totalClockBias_ns)
    delta_totalClockBias_rangeDiff_difference_ns_list.append(delta_totalClockBias_rangeDiff_difference_ns)

calculations_dict = {
    'geoRange_ns': geoRange_ns_list,
    'satClockBias_ns': satClockBias_ns_list,
    'rcvClockBias_ns': rcvClockBias_ns_list,
    'relativisticBias_ns': relativisticBias_ns_list,
    'pseudorange_ns': pseudorange_ns_list,
    'rangeDiff_ns': rangeDiff_ns_list,
    'troposphericDelay_ns': troposphericDelay_ns_list,
    'total_bias_ns': total_bias_ns_list,
    'totalClockBias_ns': totalClockBias_ns_list,
    'totalClockBias_rangeDiff_difference_ns': totalClockBias_rangeDiff_difference_ns_list,
    'delta_gpsTOW': delta_gpsTOW_list,
    'delta_rcvTOW': delta_rcvTOW_list,
    'delta_t_tx': delta_t_tx_list,
    'delta_geoRange_ns': delta_geoRange_ns_list,
    'delta_satClockBias_ns': delta_satClockBias_ns_list,
    'delta_rcvClockBias_ns': delta_rcvClockBias_ns_list,
    'delta_relativisticBias_ns': delta_relativisticBias_ns_list,
    'delta_pseudorange_ns': delta_pseudorange_ns_list,
    'delta_rangeDiff_ns': delta_rangeDiff_ns_list,
    'delta_troposphericDelay_ns': delta_troposphericDelay_ns_list,
    'delta_total_bias_ns': delta_total_bias_ns_list,
    'delta_totalClockBias_ns': delta_totalClockBias_ns_list,
    'delta_totalClockBias_rangeDiff_difference_ns': delta_totalClockBias_rangeDiff_difference_ns_list
}

calculations_df = pd.DataFrame(calculations_dict)
data = ubx_data.join(calculations_df)

# print(data['delta_troposphericDelay_ns'])

fig, ax1 = plt.subplots()

color = 'tab:blue'
ax1.plot(data['rcvTOW'], data['troposphericDelay_ns'], color=color)

ax2 = ax1.twinx()
color = 'tab:green'
ax2.plot(data['rcvTOW'], data['delta_pseudorange_ns'], color=color)
plt.show()