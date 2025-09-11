import numpy as np

# Earth Parameters
Re = 6378137 # meters
e2 = 0.00669437999014 # eccentricity squared (WGS84)
mu = 3.986004418 * pow(10,14) # standard gravitational parameter
speedOfLight = 299792458 # m/s

# Receiver Parameters
lat = 32.2654654 # degrees
long = -82.312312312 # degrees
alt = 4.45454 # meters
time = 321313.3123 # from rcvTow
year = 2025
month = 1
day = 27
hour = 12
minute = 2
second = 3

# Satellite Parameters
svID = 25
pseudorange = 26146354.042 # meters
carrierPhase = 137400018.512 # cycles
elevation = 90 # degrees
azimuth = 36 # degrees
frequency = 1575.42 * pow(10,6) # MHz

a = 312312.3123
e = 12313123.3123
inc = 3123123.12312
raan = 31231.3312
omega0 = 3123123.3123
omega = 32312.31231
omegaDot = 3231.3123
m0 = 312312.312
ToE = 4234234.4234 # Ephemeris reference time

def solve_eccentric_anomaly(M, e, tolerance=1e-8, max_iterations=100):
    """
    Solves for the eccentric anomaly E using the Newton-Raphson method.

    Parameters:
        M: Mean anomaly (radians)
        e: Eccentricity
        tolerance: Convergence tolerance
        max_iterations: Maximum number of iterations

    Returns:
        E: Eccentric anomaly (radians)
    """
    # Initial guess
    E = M if e < 0.8 else np.pi

    for _ in range(max_iterations):
        f_E = E - e * np.sin(E) - M
        f_prime_E = 1 - e * np.cos(E)
        E_next = E - f_E / f_prime_E
        
        # Check convergence
        if abs(E_next - E) < tolerance:
            return E_next
        E = E_next

    raise RuntimeError("Eccentric anomaly calculation did not converge")

def current_julian_date(year,month,day,hour,minute,second):
    if month <= 2:
        year -= 1
        month += 12

    ut = hour + minute/60.0 + second/3600.0

    julian_date = (367 * year
        - np.floor(7 * (year + np.floor((month + 9) / 12)) / 4)
        + np.floor(275 * month / 9)
        + day
        + 1721013.5
        + ut / 24.0)
    return julian_date

def calculate_gst(julian_date):
    jd_ref = 2451545.0
    delta_jd = julian_date - jd_ref

    gst_deg = (280.46061837 + 360.98564736629 * delta_jd) % 360.0

    return np.radians(gst_deg)

# ECEF Coordinates Receiver
N = Re / np.sqrt(1 - e2*pow(np.sin(np.deg2rad(lat)),2))
xr = (N+alt)*np.cos(np.deg2rad(lat))*np.cos(np.deg2rad(long))
yr = (N+alt)*np.cos(np.deg2rad(lat))*np.sin(np.deg2rad(long))
zr = ((1-e2)*N+alt)*np.sin(np.deg2rad(lat))

# ECEF Coordinates Satellite
dt = time - ToE
n = np.sqrt(mu/pow(a,3))
meanAnomaly_t = m0 + n*dt
eccentricAnomaly = solve_eccentric_anomaly(meanAnomaly_t, e)
trueAnomaly = np.arctan2(np.sqrt(1-e2)*np.sin(meanAnomaly_t),np.cos(meanAnomaly_t-e))
r_orbital = a*(1-e*np.cos(eccentricAnomaly))
x_prime = r_orbital * np.cos(trueAnomaly)
y_prime = r_orbital * np.sin(trueAnomaly)

orbital_coords = [[x_prime * np.cos(omega) - y_prime * np.sin(omega)],[x_prime * np.sin(omega) + y_prime * np.cos(omega)],[0]]
angles = [[np.cos(omega0), -np.sin(omega0)*np.cos(inc), np.sin(omega0)*np.sin(inc)],np.sin(omega0),np.cos(omega0)*np.cos(inc),-np.cos(omega0)*np.sin(omega0)],[0,np.sin(inc),np.cos(inc)]

eci_sat = np.matmul(orbital_coords,angles)

julian_date = current_julian_date(year,month,day,hour,minute,second)
gst = calculate_gst(julian_date)

rotation_matrix = np.array([[np.cos(gst), np.sin(gst), 0],[-np.sin(gst),np.cos(gst),0],[0,0,1]])
ecef_sat = rotation_matrix @ eci_sat.T

xs = ecef_sat[0]
ys = ecef_sat[1]
zs = ecef_sat[2]

geometric_range = np.sqrt(pow((xs-xr),2)+pow((ys-yr),2)+pow((zs-zr),2))

wavelength = speedOfLight / frequency # meters

