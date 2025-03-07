import numpy as np

L = 10 / np.pow(10,6)       # Henries
L_dcr = 6.4 / np.pow(10,3)  # Ohms

R_DSon1 = 6.5 / np.pow(10,3) # Ohms
R_DSon2 = 2.0 / np.pow(10,3) # Ohms

Iout = 3    # Amps
Vout = 3.3  # Volts
Vin = 6.6   # Volts

P_out = Vout * Iout # Watts

f_sw = 500 * np.pow(10,3) # Hertz

# Inductor Power
I_ripple = ((Vin-Vout)*Vout)/(L*f_sw*Vin)

I_rms_inductor = np.sqrt(np.pow(Iout,2) + np.pow(I_ripple, 2)/12)

P_inductor = np.pow(I_rms_inductor, 2) * L_dcr

# MOSFET Power


P_q1 = Vout/Vin * (np.pow(Iout,2) + np.pow(I_ripple, 2)/12) * R_DSon1
P_q2 = (1 - Vout/Vin) * (np.pow(Iout,2) + np.pow(I_ripple, 2)/12) * R_DSon2

P_fet = P_q1 + P_q2

# Total Power Loss

P_d = P_inductor + P_fet


efficiency = P_out / (P_out + P_d)

print(f"eta:\t{efficiency*100:0.3f}%")