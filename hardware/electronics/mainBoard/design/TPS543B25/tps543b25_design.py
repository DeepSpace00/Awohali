import numpy as np

# System Configuration

Vin = 6.6  # Volts
Vout = 3.3 # Volts
Iout = 3.0 # Amps


f_sw = 500 # kHz

L_ripple = 0.2 # Inductor ripple current

# Inductor Selection

L = (Vin-Vout)/(Iout*L_ripple) * Vout/Vin * 1/(f_sw*1000)

print(f"L:\t{L*np.pow(10,6):0.3f}\tuH")