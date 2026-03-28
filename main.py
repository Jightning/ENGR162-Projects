import math
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from tqdm import tqdm

def calc_gravity_force(density_particle, g, d_particle):
    return (math.pi / 6) * density_particle * g * (d_particle ** 3)

def calc_buoyant_force(density_fluid, g, d_particle):
    return (math.pi / 6) * density_fluid * g * (d_particle ** 3)

# v_apparent is a numpy array (vector)
# just returning magnitude
def calc_drag_force(density_fluid, d_particle, dynamic_viscosity, v_apparent):
    v_mag = math.sqrt(np.sum(np.square(v_apparent)))
   
    if v_mag < 1e-30:
        return np.array([0.0, 0.0])
    
    Re = (density_fluid * d_particle * v_mag) / dynamic_viscosity
    C_d = 24 / Re # I'm stoked for stokes
    
    cross_section = (math.pi / 4.0) * d_particle ** 2
   
    return 0.5 * density_fluid * C_d * cross_section * v_mag ** 2 * (v_apparent / v_mag)

# electric force from a single plate on tower (assuming the tower is a plate)
def calc_tower_electric_force(sigma, q, epsilon):
    return (sigma * q) / (2 * epsilon)

# total electric force in a cloud
# c and height p are at time t
# height p is the distance from the plate (D(t))
# c is particle concentration I think
def calc_cloud_electric_force(total_height, height_p, q, concentration, epsilon):
    return ((concentration * q ** 2) / (2 * epsilon)) * (2 * height_p - total_height)

time_step = 0.001

# Laminar motion (possibly?)
# If it's laminar Re < 2000, with an estimated 150, probably calculate later
# Particles
# research later, this is the first values I found
v_particle = 15 * (10 ** -6) # idek what this is, I just saw it somewhere in the notes
d_particle = 2.5 * 10 ** -6 # particle diameter (in meters I think, again this is from notes)
A_particle = (math.pi / 4) * (d_particle) ** 2 # assuming spherical particles
p_particle = 1500 # density kg/m^3
m_particle = (math.pi / 6) * p_particle * d_particle ** 3 # particle mass (kg)
q_particle = 3.2 * 10 ** -17 # particle charge (Coulombs)

# Environment
p_air = 1.194 # air density
v_air = np.array([0, -0.5]) # air velocity
g = 9.81 # gravity (m/s^2)
PM_init = 17
concentration = (PM_init / 1e9) / m_particle # particles / m^3 c(t)
pollution = (((5 / 1e9) / 3600) / m_particle) # how much "concentration" gets added every time step
cloud_height = 7 # total height of the cloud for electric field (H in m)

# Tower
E_tower = 30000 # Electric Field Strength (V)
h_tower = 3.5 # Effective Tower Height (meters)

# Others
epsilon = 8.854 * 10 ** -12 # vacuum permittivity C^2 / (N * m^2)
sigma = 2.0 * epsilon * 300e3 # charge per m^2
dynamic_viscosity = 1.84 * 10 ** -5

# vectors represented with the great numpy
# 2D for now, tower is at (0, 0) - (x, y)
gravity_force = np.array([0, -calc_gravity_force(p_particle, g, d_particle)])
buoyant_force = np.array([0, calc_buoyant_force(p_air, g, d_particle)])

def model(state, t):
    x, y, vx, vy, concentration = state # current state of particle
    distance = math.sqrt(x ** 2 + y ** 2)
    v_apparent = v_air - np.array([vx, vy])
    drag_force = calc_drag_force(p_air, d_particle, dynamic_viscosity, v_apparent)
    
    electric_tower_force = calc_tower_electric_force(sigma, q_particle, epsilon)
    electric_cloud_force = calc_cloud_electric_force(cloud_height, y, q_particle, concentration, epsilon)
    
    electric_force = np.array([electric_cloud_force - electric_tower_force, 0])
    
    F = (gravity_force + buoyant_force + drag_force + electric_force)
    
    a = F / m_particle
    
    d_concentration_dt = pollution - (concentration * abs(vx) / cloud_height)
    
    return [vx, vy, a[0], a[1], d_concentration_dt]

# Inputs
try: mode = int(input("Select Mode (1 - Graphs, 2 - Average): "))
except: mode = 1

if mode == 1:
    try: x = float(input("Initial Particle X: "))
    except: x = 1
    try: y = float(input("Initial Particle Y: "))
    except: y = 3.5

try: vx = float(input("Initial Particle vx: "))
except: vx = 0
try: vy = float(input("Initial Particle vy: "))
except: vy = 0
try: total_time = int(input("Time in Seconds: "))
except: total_time = 30

# ODE for predicting single particle movement in proximity to the tower
# This can help determine how the tower affects certain particles and at what distances and stuff
t = np.arange(0.0, total_time + time_step, time_step)
if mode == 1:
    state = [x, y, vx, vy, concentration] # x, y, vx, vy
    solution = odeint(model, state, t)
    
    x = solution[:, 0]
    y = solution[:, 1]
    vx = solution[:, 2]
    vy = solution[:, 3]
    print(x, y, vx[-1], vy[-1])
    c = solution[:, 4]
    
    final_concentration = c[-1] * m_particle * 1e9 # micro grams / m^3
    
    print(f"Total Concentration after {total_time} seconds: {final_concentration}")

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    axes[0].plot(x, y)
    axes[0].set_title("Position (m)")
    
    axes[1].plot(t, c)
    axes[1].set_title("Concentration")
    plt.show()
elif mode == 2:
    solutions = []
    for i in tqdm(range(-10, 10)):
        for j in range(-10, 10):
            state = [i, j, vx, vy, concentration] # x, y, vx, vy
            solution = odeint(model, state, t)
        
            x = solution[:, 0]
            y = solution[:, 1]
            c = solution[:, 4]

            solutions.append(c[-1] * m_particle * 10 ** 9)
    print(f"Average Concentration: {sum(solutions) / len(solutions)}")
else:
    print("Mode not available")