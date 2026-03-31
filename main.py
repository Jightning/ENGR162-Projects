import math
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from tqdm import tqdm

def calc_gravity_force(density_particle, g, d_particle):
    grav = (math.pi / 6) * density_particle * g * (d_particle ** 3)
    return np.array([0, -grav])

def calc_buoyant_force(density_fluid, g, d_particle):
    buoyant = (math.pi / 6) * density_fluid * g * (d_particle ** 3)
    return np.array([0, buoyant])

# v_apparent is a numpy array (vector)
# just returning magnitude
def calc_drag_force(density_fluid, d_particle, dynamic_viscosity, v_apparent):
    v_mag = math.sqrt(np.sum(np.square(v_apparent)))
    
    Re = (density_fluid * d_particle * v_mag) / dynamic_viscosity
    if Re != 0: C_d = 24 / Re # I'm stoked for stokes
    
    cross_section = (math.pi / 4.0) * d_particle ** 2

    # TODO Someone figure out which one is correct
    # return 0.5 * density_fluid * C_d * cross_section * v_mag * v_apparent 
    return (3 * math.pi * dynamic_viscosity * d_particle * v_apparent)

# electric force from a single plate on tower (assuming the tower is a plate)
def calc_tower_electric_force(sigma, q, epsilon, sign):
    return np.array([sign * (sigma * q) / (2 * epsilon), 0])

# total electric force in a cloud
# c and height p are at time t
# height p is the distance from the plate (D(t))
# c is particle concentration I think
def calc_cloud_electric_force(plate_distance, cloud_height, q, PM_init, epsilon, sign):
    # TODO This lowkey gave me a seizure so idk if it's correct
    magnitude = (((PM_init / 1e9 / m_particle) * q ** 2) / (2 * epsilon)) * (2 * abs(plate_distance) - cloud_height)
    return np.array([sign * magnitude, 0])

# Constants
time_step = 0.1

d_particle = 2.5e-6 # particle diameter (in meters I think, again this is from notes)
A_particle = (math.pi / 4) * (d_particle) ** 2 # assuming spherical particles
p_particle = 1500 # density kg/m^3
m_particle = (math.pi / 6) * p_particle * (d_particle ** 3) # particle mass (kg)
q_particle = 3.2e-17 # particle charge (Coulombs)

# Environment
p_air = 1.194 # air density
v_air = np.array([0, 0]) # air velocity
g = 9.81 # gravity (m/s^2)
PM_init = 17 # ug / m^3
particle_concentration = (PM_init / 1e9) / m_particle # particles / m^3 c(t)
pollution = 5 / 3600 # ug / sec
cloud_height = 7 # total height of the cloud for electric field (H in m)

# Tower
E_tower = 500 # Electric Field Strength (V)
h_tower = 3.5 # Effective Tower Height (meters)
tower_radius = 1

# Others
epsilon = 8.854e-12 # vacuum permittivity C^2 / (N * m^2)
sigma = 2.0 * epsilon * 300e3 # charge per m^2
dynamic_viscosity = 1.84e-5

# Util Constants
AVG_RADIUS = 20 # radius to look at when doing averages 


# Constant Forces
gravity_force = calc_gravity_force(p_particle, g, d_particle)
buoyant_force = calc_buoyant_force(p_air, g, d_particle)

def model(state, t):
    pos = np.array([state[0], state[1]])
    v = np.array([state[2], state[3]])
    x, y = pos[0], pos[1]

    # Particle "captured" by tower
    if abs(x) < 0.15 and y <= h_tower:
        return [0, 0, 0, 0]

    v_apparent = v_air - v
    drag_force = calc_drag_force(p_air, d_particle, dynamic_viscosity, v_apparent)
    
    electric_tower_force = calc_tower_electric_force(sigma, q_particle, epsilon, -np.sign(x))
    electric_cloud_force = calc_cloud_electric_force(x, cloud_height, q_particle, PM_init, epsilon, np.sign(x))
        
    F = gravity_force + buoyant_force + drag_force + electric_tower_force + electric_cloud_force
    
    a = F / m_particle
    
    if y <= 0 and a[1] < 0:
        a[1], v[1] = 0, 0
    
    return [v[0], v[1], a[0], a[1]]



# Simulations
# ODE for predicting single particle movement in proximity to the tower
# This can help determine how the tower affects certain particles and at what distances and stuff

# V2
radius = float(input("Evaluation Radius (m) [Default 20]: ") or 20.0)
total_time = 600
num_samples = 50
all_solutions = []
capture_times = [] # all values where a particle got captured
times = np.arange(0, total_time, time_step)

print(f"Simulating {num_samples} particles with {pollution * 3600}ug/hr pollution...")
for n in (range(num_samples)):
    # Random point in polar coords with random velocity (:<)
    r = radius * np.sqrt(np.random.uniform(0, 1))
    angle = np.random.uniform(0, 2 * np.pi)
    sx, sy = r * np.cos(angle), np.random.uniform(0, cloud_height)
    
    # ODE (main calculations)
    sol = odeint(model, [sx, sy, 0, 0], times)
    all_solutions.append(sol)
    
    # Particles captured by the tower
    cap_idx = np.where((np.abs(sol[:, 0]) < 0.15) & (sol[:, 1] <= h_tower))[0]
    capture_times.append(times[cap_idx[0]] if len(cap_idx) > 0 else float('inf'))

# Final Concentration Calculations
concentration_history = []
for t in times:
    # TODO This seems to assume all points are eventually captured which although
    # the graphs show it's correct, theoretically shouldn't be happening
    still_floating = sum(1 for ct in capture_times if ct > t)
    base_pm = (still_floating / num_samples) * PM_init
    
    new_pm = pollution * t
    
    concentration_history.append(base_pm + new_pm)

# Results
print(f"\nFinal PM2.5 Level: {concentration_history[-1]:.2f} µg/m³")
print(f"Net Change: {concentration_history[-1] - PM_init:.2f} µg/m³")
# TODO There's a bunch of values we can calculate with the data, like this (tho it's 100 with our current setup)
print(f"Tower Effectiveness: {(len(capture_times) / num_samples) * 100}%")

# Plotting
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# Trajectory
for s in all_solutions: 
    axes[0, 0].plot(s[:, 0], s[:, 1], alpha=0.2)
    # TODO Decide if you wanna disable this, it's kinda ugly (I added it in for clarity)
    axes[0, 0].annotate( # Arrow for direction (ff there's a falloff, it points to 0,0 rather than falloff point)
        '',
        xy=(s[-1, 0], s[-1, 1]),
        xytext=(s[0, 0], s[0, 1]),
        arrowprops=dict(
            arrowstyle='->',
            lw=1.5,
            mutation_scale=15     # Adjust arrow head size
        ),
    )
axes[0,0].set_title("Trajectories")

# Concentration
axes[0,1].plot(times, concentration_history, color='green', label='Total PM2.5')
axes[0,1].axhline(y=PM_init, color='red', linestyle='--', label='Start Level')
axes[0,1].set_title(f"Concentration (Includes + {pollution * 3600}µg/hr Pollution)")
axes[0,1].legend()

# X vs Time
for s in all_solutions: axes[1,0].plot(times, s[:, 0], alpha=0.2)
axes[1,0].set_title("X-Position vs Time")

# Y vs Time
for s in all_solutions: axes[1,1].plot(times, s[:, 1], alpha=0.2)
axes[1,1].set_title("Y-Position vs Time")

plt.tight_layout()
plt.show()


# V1 (old calculations for later possible reuse)
# Inputs
# try: mode = int(input("Select Mode (1 - Graphs, 2 - Average): "))
# except: mode = 1

# if mode == 1:
#     try: x = float(input("Initial Particle X: "))
#     except: x = 1
#     try: y = float(input("Initial Particle Y: "))
#     except: y = 3.5

# try: vx = float(input("Initial Particle vx: "))
# except: vx = 0
# try: vy = float(input("Initial Particle vy: "))
# except: vy = 0
# try: total_time = float(input("Time in Seconds: "))
# except: total_time = 30

# t = np.arange(0.0, total_time + time_step, time_step)
# if mode == 1:
#     state = [x, y, vx, vy] # x, y, vx, vy
#     solution = odeint(model, state, t)
    
#     x = solution[:, 0]
#     y = solution[:, 1]
#     vx = solution[:, 2]
#     vy = solution[:, 3]
#     print(x, y, vx[-1], vy[-1])
#     c = solution[:, 4]
    
#     final_concentration = c[-1] * m_particle * 1e9 # micro grams / m^3
    
#     print(f"Total Concentration after {total_time} seconds: {final_concentration}")

#     fig, axes = plt.subplots(1, 4, figsize=(13, 5))
#     axes[0].plot(x, y)
#     axes[0].set_title("Position (m)")
#     # axes[0].invert_yaxis(True)
#     # axes[0].invert_xaxis(True)
    
#     axes[1].plot(t, c * m_particle * 10 ** 9)
#     axes[1].set_title("Concentration")

#     axes[2].plot(t, x)
#     axes[2].set_title("X vs Time")

#     axes[3].plot(t, y)
#     axes[3].set_title("Y vs Time")
#     plt.show()
# elif mode == 2:
#     solutions = []
#     for r in tqdm(range(0, AVG_RADIUS)):
#         for theta in np.arange(0, 2 * math.pi, 2 * math.pi / AVG_RADIUS):
#             x = r * math.cos(theta)
#             y = r * math.sin(theta)
#             state = [x, y, vx, vy] # x, y, vx, vy
#             solution = odeint(model, state, t)
        
#             x = solution[:, 0]
#             y = solution[:, 1]
#             c = solution[:, 4]
#             final_concentration = c[-1] * m_particle * 10 ** 9

#             solutions.append(final_concentration)
#     print(f"Average Concentration: {sum(solutions) / len(solutions)}")
# else:
#     print("Mode not available")