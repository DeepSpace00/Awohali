import numpy as np

# ADCINx resistor divider calculator

R_down = 100 * np.pow(10,3)

# Decoded Value - 3
div = 0.1899

R_up = R_down / div - R_down

print(f"R_up:\t{R_up/1000:0.3f}k-ohm")

# Decoded Value - 5
div = 0.5368

R_up = R_down / div - R_down

print(f"R_up:\t{R_up/1000:0.3f}k-ohm")

# Decoded Value - 7
div = 0.9530

R_up = R_down / div - R_down

print(f"R_up:\t{R_up/1000:0.3f}k-ohm")
