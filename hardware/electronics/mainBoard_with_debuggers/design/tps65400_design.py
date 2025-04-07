import numpy as np

# System Parameters
Vin = 6.6  # Volts
f_sw = 493.278 # kHz

Vref1 = 0.74 # Volts
Vref2 = 0.75 # Volts
Vref3 = 0.76 # Volts
Vref4 = 0.8 # Volts

R_fb = 100 # k-Ohm

Vout1 = 5.0 # Volts
Vout2 = 3.3 # Volts
Vout3 = 1.8 # Volts 
Vout4 = 1.1 # Volts

Iout1 = 0.50 # Amps
Iout2 = 1.00 # Amps
Iout3 = 0.25 # Amps
Iout4 = 0.10 # Amps


# Feedback Resistor Selection
R_fb1 = R_fb*np.pow(10,3)*((Vout1 / Vref1) - 1)
R_fb2 = R_fb*np.pow(10,3)*((Vout2 / Vref2) - 1)
R_fb3 = R_fb*np.pow(10,3)*((Vout3 / Vref3) - 1)
R_fb4 = R_fb*np.pow(10,3)*((Vout4 / Vref4) - 1)

print(f"RFB1:\t{R_fb1/np.pow(10,3):0.3f} k-Ohm")
print(f"RFB2:\t{R_fb2/np.pow(10,3):0.3f} k-Ohm")
print(f"RFB3:\t{R_fb3/np.pow(10,3):0.3f} k-Ohm")
print(f"RFB4:\t{R_fb4/np.pow(10,3):0.3f} k-Ohm")

# Inductor Selection
delta_I_L1 = 0.1 * Iout1 # Current Ripple in the inductor
delta_I_L2 = 0.1 * Iout2 # Current Ripple in the inductor
delta_I_L3 = 0.1 * Iout3 # Current Ripple in the inductor
delta_I_L4 = 0.1 * Iout4 # Current Ripple in the inductor

L1 = (Vout1 * (1 - Vout1/Vin)) / (delta_I_L1 * f_sw * np.pow(10,3)) # Henries
L2 = (Vout2 * (1 - Vout2/Vin)) / (delta_I_L2 * f_sw * np.pow(10,3)) # Henries
L3 = (Vout3 * (1 - Vout3/Vin)) / (delta_I_L3 * f_sw * np.pow(10,3)) # Henries
L4 = (Vout4 * (1 - Vout4/Vin)) / (delta_I_L4 * f_sw * np.pow(10,3)) # Henries

print(f"L1:\t{L1*np.pow(10,6):0.3f} uH")
print(f"L2:\t{L2*np.pow(10,6):0.3f} uH")
print(f"L3:\t{L3*np.pow(10,6):0.3f} uH")
print(f"L4:\t{L4*np.pow(10,6):0.3f} uH")

# Output Capacitor Selection
L1_actual = 47 / np.pow(10,6)
L2_actual = 33 / np.pow(10,6)
L3_actual = 100 / np.pow(10,6)
L4_actual = 180 / np.pow(10,6)

delta_Vout_ripple = 0.001 # Volts

C_out1 = ((Vin - Vout1)*Vout1) / (8 * np.pow(f_sw*np.pow(10,3),2) * delta_Vout_ripple * L1_actual * Vin)
C_out2 = ((Vin - Vout2)*Vout2) / (8 * np.pow(f_sw*np.pow(10,3),2) * delta_Vout_ripple * L2_actual * Vin)
C_out3 = ((Vin - Vout3)*Vout3) / (8 * np.pow(f_sw*np.pow(10,3),2) * delta_Vout_ripple * L3_actual * Vin)
C_out4 = ((Vin - Vout4)*Vout4) / (8 * np.pow(f_sw*np.pow(10,3),2) * delta_Vout_ripple * L4_actual * Vin)


print(f"C_Out1:\t{C_out1*np.pow(10,6):0.3}uF")
print(f"C_Out2:\t{C_out2*np.pow(10,6):0.3}uF")
print(f"C_Out3:\t{C_out3*np.pow(10,6):0.3}uF")
print(f"C_Out4:\t{C_out4*np.pow(10,6):0.3}uF")


# Compensation Calculator
Gm = 120 / np.pow(10,6) # uA/V
Gm_ps = 10 # A/V
Fc = f_sw*np.pow(10,3)/10

C_out1_actual = 20 / np.pow(10,6)
C_out2_actual = 30 / np.pow(10,6)
C_out3_actual = 10 / np.pow(10,6)
C_out4_actual = 10 / np.pow(10,6)

Rc1 = (2*np.pi * Fc * Vout1 * C_out1_actual) / (Gm * Gm_ps * Vref1)
Rc2 = (2*np.pi * Fc * Vout2 * C_out2_actual) / (Gm * Gm_ps * Vref2)
Rc3 = (2*np.pi * Fc * Vout3 * C_out3_actual) / (Gm * Gm_ps * Vref3)
Rc4 = (2*np.pi * Fc * Vout4 * C_out4_actual) / (Gm * Gm_ps * Vref4)

print(f"Rc1:\t{Rc1/1000:0.3f}k-Ohm")
print(f"Rc2:\t{Rc2/1000:0.3f}k-Ohm")
print(f"Rc3:\t{Rc3/1000:0.3f}k-Ohm")
print(f"Rc4:\t{Rc4/1000:0.3f}k-Ohm")

Cc1 = ((Vout1/Iout1)*C_out1_actual) / Rc1
Cc2 = ((Vout2/Iout2)*C_out2_actual) / Rc2
Cc3 = ((Vout3/Iout3)*C_out3_actual) / Rc3
Cc4 = ((Vout4/Iout4)*C_out4_actual) / Rc4

print(f"Cc1:\t{Cc1*np.pow(10,9):0.3f}nF")
print(f"Cc2:\t{Cc2*np.pow(10,9):0.3f}nF")
print(f"Cc3:\t{Cc3*np.pow(10,9):0.3f}nF")
print(f"Cc4:\t{Cc4*np.pow(10,9):0.3f}nF")
