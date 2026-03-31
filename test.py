"""
Vector-Based Smog Tower Simulation
Includes: 5ug/hr Background Pollution Increase, Vector Math, & Slip Correction
"""

import math
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt


# --- 1. User-Defined Physics Functions ---
def get_cunningham_correction(d_particle):
    lambda_air = 66e-9
    return 1 + (2 * lambda_air / d_particle) * (1.257 + 0.4 * math.exp(-1.1 * d_particle / (2 * lambda_air)))

def calc_gravity_force(density_particle, g, d_particle):
    mag = (math.pi / 6) * density_particle * g * (d_particle ** 3)
    return np.array([0.0, -mag])

def calc_buoyant_force(density_fluid, g, d_particle):
    mag = (math.pi / 6) * density_fluid * g * (d_particle ** 3)
    return np.array([0.0, mag])

def calc_drag_force(dynamic_viscosity, d_particle, v_apparent, Cc):
    return (3 * math.pi * dynamic_viscosity * d_particle * v_apparent) / Cc

# --- 2. Constants & Environment ---
time_step = 0.1
total_time = 600
t_span = np.arange(0, total_time, time_step)

# Pollution Settings
PM_init = 17.0             # Initial ug/m^3
pollution_flux_hr = 5.0    # New pollution entering system (ug/hr)
flux_per_sec = pollution_flux_hr / 3600.0

# Particle & Tower Properties
d_particle = 2.5e-6
p_particle = 1500
m_particle = (math.pi / 6) * p_particle * (d_particle ** 3)
q_particle = 3.2e-17
Cc = get_cunningham_correction(d_particle)

p_air = 1.194
v_air = np.array([0.0, 0.0])
g = 9.81
epsilon = 8.854e-12
sigma = 2.0 * epsilon * 300e3
dynamic_viscosity = 1.84e-5
cloud_height = 7.0
tower_h = 3.5    

F_grav = calc_gravity_force(p_particle, g, d_particle)
F_buoy = calc_buoyant_force(p_air, g, d_particle)

# --- 3. The Vector ODE Model ---
def model(state, t):
    pos = np.array([state[0], state[1]])
    v = np.array([state[2], state[3]])
    x, y = pos[0], pos[1]
   
    # Capture Logic (0.15m radius from tower center)
    if abs(x) < 0.15 and y <= tower_h:
        return [0.0, 0.0, 0.0, 0.0]

    v_apparent = v_air - v
    F_drag = calc_drag_force(dynamic_viscosity, d_particle, v_apparent, Cc)
   
    # Tower Force (Pull)
    f_tower_mag = (sigma * q_particle) / (2 * epsilon)
    F_tower = np.array([-np.sign(x) * f_tower_mag, 0.0])
   
    # Cloud Force (Repulsion/Attraction based on Purdue Slide)
    # Using PM_init for the field constant
    f_cloud_mag = (((PM_init/1e9/m_particle) * q_particle**2) / (2 * epsilon)) * (2 * abs(x) - cloud_height)
    F_cloud = np.array([np.sign(x) * f_cloud_mag, 0.0])

    F_total = F_grav + F_buoy + F_drag + F_tower + F_cloud
    a = F_total / m_particle

    if y <= 0 and a[1] < 0:
        a[1], v[1] = 0.0, 0.0

    return [v[0], v[1], a[0], a[1]]

# --- 4. Simulation Execution ---
radius = float(input("Evaluation Radius (m) [Default 20]: ") or 20.0)
num_samples = 50
all_solutions = []
capture_times = []

print(f"Simulating {num_samples} particles with {pollution_flux_hr}ug/hr influx...")
for n in (range(num_samples)):
    r = radius * np.sqrt(np.random.uniform(0, 1))
    angle = np.random.uniform(0, 2*np.pi)
    sx, sy = r * np.cos(angle), np.random.uniform(0, cloud_height)
   
    sol = odeint(model, [sx, sy, 0, 0], t_span)
    all_solutions.append(sol)
   
    cap_idx = np.where((np.abs(sol[:, 0]) < 0.15) & (sol[:, 1] <= tower_h))[0]
    capture_times.append(t_span[cap_idx[0]] if len(cap_idx) > 0 else float('inf'))

# --- 5. Process Concentration (The "Open System" Calculation) ---
concentration_history = []
for t in t_span:
    # 1. Particles remaining from original cloud
    still_floating = sum(1 for ct in capture_times if ct > t)
    base_pm = (still_floating / num_samples) * PM_init
   
    # 2. Accumulated new pollution (flux)
    new_pm = flux_per_sec * t
   
    concentration_history.append(base_pm + new_pm)

# --- 6. Plotting ---
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# Trajectory
for s in all_solutions: axes[0,0].plot(s[:, 0], s[:, 1], alpha=0.2)
axes[0,0].set_title("Trajectories")

# Concentration (Showing the Influx)
axes[0,1].plot(t_span, concentration_history, color='green', label='Total PM2.5')
axes[0,1].axhline(y=PM_init, color='red', linestyle='--', label='Start Level')
axes[0,1].set_title(f"Concentration (Includes +{pollution_flux_hr}µg/hr Influx)")
axes[0,1].legend()

# X vs Time
for s in all_solutions: axes[1,0].plot(t_span, s[:, 0], alpha=0.2)
axes[1,0].set_title("X-Position vs Time")

# Y vs Time
for s in all_solutions: axes[1,1].plot(t_span, s[:, 1], alpha=0.2)
axes[1,1].set_title("Y-Position vs Time")

plt.tight_layout()
plt.show()

print(f"\nFinal PM2.5 Level: {concentration_history[-1]:.2f} µg/m³")
print(f"Net Change: {concentration_history[-1] - PM_init:.2f} µg/m³")