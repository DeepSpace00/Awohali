import numpy as np

# Feedback Voltage Divider Calculations
Reg1_R1 = 45.3 # k-ohms
Reg1_R2 = 10 # k-ohms

Reg2_R1 = 20 # k-ohms
Reg2_R2 = 10 # k-ohms

Reg3_R1 = 8.45 # k-ohms
Reg3_R2 = 10 # k-ohms

Vout1 = 0.6*(Reg1_R1/Reg1_R2) + 0.6 # Volts
Vout2 = 0.6*(Reg2_R1/Reg2_R2) + 0.6 # Volts
Vout3 = 0.6*(Reg3_R1/Reg3_R2) + 0.6 # Volts

print(f"V_OUT_1\t= {Vout1:.3f} V")
print(f"V_OUT_2\t= {Vout2:.3f} V")
print(f"V_OUT_3\t= {Vout3:.3f} V")

# Switching Frequency Calculation
R_osc = 61.9 # k-ohm

f_osc = 37254 * np.pow(R_osc,-0.966)

print(f"f_osc\t= {f_osc:.1f} kHz")

# Inductor Selection Calculation
Vin_Max = 7.5 # Volts
Lir = 0.1 # Coefficient for inductor ripple relative to maximum output current
f_sw = f_osc * 1000

Iout1 = 1 # Amps
Iout2 = 0.5 # Amps
Iout3 = 0.5 # Amps

L1 = (Vin_Max - Vout1)/(Iout1*Lir) * Vout1/(Vin_Max*f_sw)
L2 = (Vin_Max - Vout2)/(Iout2*Lir) * Vout2/(Vin_Max*f_sw)
L3 = (Vin_Max - Vout3)/(Iout3*Lir) * Vout3/(Vin_Max*f_sw)

I1_ripple = (Vin_Max - Vout1)/L1 * Vout1/(Vin_Max*f_sw)
I2_ripple = (Vin_Max - Vout2)/L2 * Vout2/(Vin_Max*f_sw)
I3_ripple = (Vin_Max - Vout3)/L3 * Vout3/(Vin_Max*f_sw)

I1_Lrms = np.sqrt(np.pow(Iout1,2) + np.pow(2,((Vout1*(Vin_Max-Vout1))/(Vin_Max*L1*f_sw)))/12)
I2_Lrms = np.sqrt(np.pow(Iout2,2) + np.pow(2,((Vout2*(Vin_Max-Vout2))/(Vin_Max*L2*f_sw)))/12)
I3_Lrms = np.sqrt(np.pow(Iout3,2) + np.pow(2,((Vout3*(Vin_Max-Vout3))/(Vin_Max*L3*f_sw)))/12)

I1_Lpk = Iout1 + I1_ripple/2
I2_Lpk = Iout2 + I2_ripple/2
I3_Lpk = Iout3 + I3_ripple/2

print(f"L1\t= {L1*np.pow(10,6):.3f} uH")
print(f"I1_Lrms\t= {I1_Lrms:0.3f} A")
print(f"I1_Lpk\t= {I1_Lpk:0.3f} A")

print(f"L2\t= {L2*np.pow(10,6):.3f} uH")
print(f"I2_Lrms\t= {I2_Lrms:0.3f} A")
print(f"I2_Lpk\t= {I2_Lpk:0.3f} A")

print(f"L3\t= {L3*np.pow(10,6):.3f} uH")
print(f"I3_Lrms\t= {I3_Lrms:0.3f} A")
print(f"I3_Lpk\t= {I3_Lpk:0.3f} A")

# Output Capacitor Calculation
Iout_droop = 0.05 # percent output current change
Vout_droop = 0.01 # Percent output of voltage

Cout1_min = (2*Iout1*Iout_droop)/(f_sw*Vout1*Vout_droop)
Cout2_min = (2*Iout2*Iout_droop)/(f_sw*Vout2*Vout_droop)
Cout3_min = (2*Iout3*Iout_droop)/(f_sw*Vout3*Vout_droop)

print(f"C_OUT_1\t= {Cout1_min*np.pow(10,6):.2f} uF")
print(f"C_OUT_2\t= {Cout2_min*np.pow(10,6):.2f} uF")
print(f"C_OUT_3\t= {Cout3_min*np.pow(10,6):.2f} uF")

# Loop Compensation
R_l1 = 0.037 # Ohms
R_l2 = 0.057 # Ohms
R_l3 = 0.037 # Ohms

Cout1 = 10 / np.pow(10,6)
Cout2 = 10 / np.pow(10,6)
Cout3 = 10 / np.pow(10,6)

f_c = f_sw/10

G_m_EA = 300 / np.pow(10,6) # Seconds
G_m_PS = 7.4 # A/V

Rc1 = (2*np.pi*f_c*Vout1*Cout1)/(G_m_EA * 0.6 * G_m_PS)
Rc2 = (2*np.pi*f_c*Vout2*Cout2)/(G_m_EA * 0.6 * G_m_PS)
Rc3 = (2*np.pi*f_c*Vout3*Cout3)/(G_m_EA * 0.6 * G_m_PS)

Cc1 = R_l1 * Cout1 / Rc1
Cc2 = R_l2 * Cout2 / Rc2
Cc3 = R_l3 * Cout3 / Rc3

print(f"R_C_1\t= {Rc1/np.pow(10,3):0.3f} k-ohm")
print(f"R_C_2\t= {Rc2/np.pow(10,3):0.3f} k-ohm")
print(f"R_C_3\t= {Rc3/np.pow(10,3):0.3f} k-ohm")

print(f"C_C_1\t= {Cc1*np.pow(10,12):0.3f} pF")
print(f"C_C_2\t= {Cc2*np.pow(10,12):0.3f} pF")
print(f"C_C_3\t= {Cc3*np.pow(10,12):0.3f} pF")

C1_1 = 1/(np.pi*Reg1_R1*1000*f_c)
C1_2 = 1/(np.pi*Reg2_R1*1000*f_c)
C1_3 = 1/(np.pi*Reg3_R1*1000*f_c)

print(f"C1_1\t= {C1_1*np.pow(10,12):0.3f} pF")
print(f"C1_2\t= {C1_2*np.pow(10,12):0.3f} pF")
print(f"C1_3\t= {C1_3*np.pow(10,12):0.3f} pF")