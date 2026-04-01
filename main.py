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
time_step = 0.5
total_time = 3600 * 24
num_samples = 500
rng = np.random.default_rng() # for randomizing stuff

d_particle = 2.5e-6 # particle diameter (in meters I think, again this is from notes)
A_particle = (math.pi / 4) * (d_particle) ** 2 # assuming spherical particles
p_particle = 1500 # density kg/m^3
m_particle = (math.pi / 6) * p_particle * (d_particle ** 3) # particle mass (kg)
q_particle = 3.2e-17 # particle charge (Coulombs)

# Environment
p_air = 1.194 # air density kg/m^3
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
terminal_velocity = 0.00025

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
sweep = False
step = 1
radius = float(input("Evaluation Radius (m) [Default 20] [-1 for sweep test]: ") or 20.0)
effective_radius = -1
if radius == -1: 
    sweep = True
    radii = 100
    radius = 50
else:
    radius = int(radius)
    radii = radius + 1

times = np.arange(0, total_time, time_step)

print(f"Simulating {num_samples} particles with {pollution * 3600}ug/hr pollution...")
for rad in tqdm(range(radius, radii, step)):
    all_solutions = []
    capture_times = [] # all values where a particle got captured

    for n in (range(num_samples)):
        # Random point in polar coords with random velocity (:<)
        r = rad * np.sqrt(np.random.uniform(0, 1))
        angle = np.random.uniform(0, 2 * np.pi)
        sx, sy = rad * np.cos(angle), np.random.uniform(0, cloud_height)

        # ODE (main calculations)
        v_air = rng.random(2) * 2 * terminal_velocity - terminal_velocity # random 2 values for air (from -terminal to terminal)
        sol = odeint(model, [sx, sy, 0, 0], times)
        all_solutions.append(sol)

        # Particles captured by the tower
        cap_idx = np.where((np.abs(sol[:, 0]) < tower_radius) & (sol[:, 1] <= h_tower))[0]
        capture_times.append(times[cap_idx[0]] if len(cap_idx) > 0 else float('inf'))

    num_free = num_samples - sum(1 for ct in capture_times if ct != float('inf'))
    # Final Concentration Calculations
    concentration_history = []
    final_concentration = -1
    for idx, t in enumerate(times): 
        # Counts number of particles yet to be captured
        still_floating = sum(1 for ct in capture_times if ct != float('inf') and ct > t)
        # Total % particle not captured at this time, times PM
        base_pm = ((still_floating + num_free) / num_samples) * PM_init

        new_pm = pollution * t

        if (still_floating <= 0 and final_concentration == -1):
            final_concentration = base_pm + new_pm
            break

        concentration_history.append(base_pm + new_pm)

    effectiveness = (sum(1 for t in capture_times if t != float('inf')) / num_samples)
    # if effective_radius == -1 and final_concentration - PM_init > 0:
    #     effective_radius = rad - step
    #     break
    if final_concentration > 5 and effective_radius == -1:
        effective_radius = rad - step
        break

# Results
print(f"\nFinal PM2.5 Level: {final_concentration:.2f} µg/m³")
print(f"Net Change: {final_concentration - PM_init:.2f} µg/m³")
# TODO There's a bunch of values we can calculate with the data, like this (tho it's 100 with our current setup)
print(f"Tower Effectiveness: {effectiveness * 100:.2f}%") # messy but whatever
volume = ((4/3) * math.pi * radius ** 3) * particle_concentration * ((4/3) * math.pi * (d_particle / 2) ** 3) * effectiveness
ratio = (PM_init * 1e-6) / (p_air * 1e3)
volumetric_flow = volume + volume / ratio
print(f"Smog Volume Absorbed: {volume} (cm)^3")
print(f"Volumetric Flow: {volume * 100 / total_time} (m)^3/s")

if sweep:
    print(f"Effective Radius: {effective_radius}")

# Plotting
fig, axes = plt.subplots(1, 2)

# Trajectory
for s in all_solutions:
    axes[0].plot(s[:, 0], s[:, 1], alpha=0.2)
    # Arrow pointing towards the travel direction of the particle
    end_x, end_y = s[-1, 0], s[-1, 1]
    dist = math.hypot(end_x, end_y)
    if dist > 1e-6:
        ux, uy = (end_x / dist), (end_y / dist) # unit vectors
        arrow_len = min(0.25 * dist, 0.5)
        # Tail is a step away from the head in the direction away from the tower
        tail_x = end_x - ux * arrow_len
        tail_y = end_y - uy * arrow_len
        axes[0].annotate(
            '',
            xy=(end_x, end_y),    # arrow head at particle endpoint
            xytext=(tail_x, tail_y),
            arrowprops=dict(
                arrowstyle='->',
                lw=1.5,
                mutation_scale=15     # Adjust arrow head size
            ),
        )

axes[0].set_title("Trajectories")
# Concentration
axes[1].plot(times[:idx], concentration_history, color='green', label='Total PM2.5')
axes[1].axhline(y=PM_init, color='red', linestyle='--', label='Start Level')
axes[1].axhline(y=5, color='red', linestyle='--', label='Start Level')
axes[1].set_title(f"Concentration (Includes + {pollution * 3600}µg/hr Pollution)")
axes[1].legend()

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