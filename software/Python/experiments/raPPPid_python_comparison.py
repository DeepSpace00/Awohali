import pandas as pd
import matplotlib.pyplot as plt
from meteorology.tropospheric_products import conversion_coefficient

temp = 25.0

ubx_rapppid_file = "../data/ubx_results_float.csv"
cors_rapppid_file = "../data/ormd_results_float.csv"

ubx_rapppid_data = pd.read_csv(ubx_rapppid_file)
cors_rapppid_data = pd.read_csv(cors_rapppid_file)

ubx_rapppid_data = ubx_rapppid_data[["sow", "zwd"]]
cors_rapppid_data = cors_rapppid_data[["sow", "zwd"]]

ubx_rapppid_data = ubx_rapppid_data.rename(columns={"sow": "rcvTOW", "zwd": "ZWD_m"})
cors_rapppid_data = cors_rapppid_data.rename(columns={"sow": "rcvTOW", "zwd": "ZWD_m"})

conversion_coeff = conversion_coefficient(temp)

ubx_rapppid_data["PWV_mm"] = ubx_rapppid_data["ZWD_m"] * conversion_coeff * 1000.0
cors_rapppid_data["PWV_mm"] = cors_rapppid_data["ZWD_m"] * conversion_coeff * 1000.0

f = plt.figure()
f.set_figwidth(12)
f.set_figheight(8)

plt.plot(ubx_rapppid_data["rcvTOW"], ubx_rapppid_data["PWV_mm"], label="u-blox - ERAU")
plt.plot(cors_rapppid_data["rcvTOW"], cors_rapppid_data["PWV_mm"], label="CORS - ORMD")
plt.xlim(ubx_rapppid_data["rcvTOW"].min(), ubx_rapppid_data["rcvTOW"].max())
plt.xlabel("rcvTOW (s)")
plt.ylabel("PWV (kg/m^2)")
plt.title("CORS vs u-blox raPPPid Results")
plt.grid(True)
plt.legend()
plt.show()