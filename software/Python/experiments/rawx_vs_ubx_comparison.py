import sqlite3
import pandas as pd
import matplotlib.pyplot as plt

ubx_database = "../data/ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv.db"
rinex_database = "../data/RINEX_data/ORMD/2025-11-25/RINEX_pwv.db"

# ubx_database = "../data/ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_old_eph.db"
# rinex_database = "../data/RINEX_data/ORMD/2025-11-25/RINEX_pwv_old_eph.db"

svIds = ['E05', 'E06', 'E09', 'E10', 'E11', 'E16', 'E19', 'E23', 'E25', 'E31', 'E34', 'E36', 'G01', 'G03',
         'G04', 'G08', 'G09', 'G10', 'G25', 'G26', 'G27', 'G28', 'G32']

for svId in svIds:

    ubx_con = sqlite3.connect(ubx_database)
    ubx_data = pd.read_sql(f"select * from '{svId}'", con=ubx_con)
    ubx_con.close()

    rinex_con = sqlite3.connect(rinex_database)
    rinex_data = pd.read_sql(f"select * from '{svId}'", con=rinex_con)
    rinex_con.close()

    ubx_data = ubx_data[ubx_data['elevation'] >= 30]
    rinex_data = rinex_data[rinex_data['elevation'] >= 30]


    # fig, ax1 = plt.subplots()
    # ax1.plot(ubx_data['rcvTOW'], ubx_data['troposphericDelay'], color='blue')
    # ax2 = ax1.twinx()
    # ax2.plot(rinex_data['rcvTOW'], rinex_data['troposphericDelay'], color='green')

    plt.figure()
    fig, ax1 = plt.subplots()
    ax1.plot(ubx_data['rcvTOW'], ubx_data['ztd'], color='blue')
    ax2 = ax1.twinx()
    ax2.plot(rinex_data['rcvTOW'], rinex_data['ztd'], color='green')

    fig.supxlabel("rcvTOW (s)")
    fig.supylabel("Zenith Time Delay (mm)")
    plt.title(f"Zenith Time Delay (ZTD) - {svId}")
    plt.grid()
    plt.tight_layout()

    # plt.savefig(f"../data/figures/old_eph/troposphericDelay/{svId}.png")
    plt.savefig(f"../data/figures/new_eph/ztd/{svId}.png")
    plt.close()