import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

def inches_to_meters(inches: float) -> float:
    return inches * 0.0254

# Constants
g = 9.81  # m/s^2
dt = 0.002 # s
hub_entrance_height_m = inches_to_meters(72)
hub_entrance_width_m = inches_to_meters(40)
hub_target_offset = inches_to_meters(14) # inches back
shooter_height_m = inches_to_meters(16)

# Ball/air model - probably not accurate but whatever
rho_air = 1.225 # kg/m^3

fuel_diameter_m = inches_to_meters(5.906)
fuel_radius_m = fuel_diameter_m / 2.0
fuel_area_m2 = np.pi * fuel_radius_m**2

fuel_mass_lb = 0.47
fuel_mass_kg = fuel_mass_lb * 0.453592

# Drag coefficient (sphere-ish), has to be tuned but whatever
Cd = 0.43

def calculate_spin_rad_per_sec(v0: float) -> float:
    """Rough approximation of turret spin based on exit velocity.
    This is probably nowhere close, but... whatever"""
    # We have a one-side-fixed turret, so the final fuel angular velocity is roughly
    # its linear velocity over radius
    efficiency = 0.9
    return (v0 / fuel_radius_m) * efficiency

def rpm_to_rad_per_sec(rpm: float) -> float:
    return rpm * 2.0 * np.pi / 60.0

def hood_angle_to_theta(hood_angle_rad: float) -> float:
    """Convert hood angle (0 = horizontal/straight up shot) to launch angle theta."""
    return np.radians(90) - hood_angle_rad + np.radians(3)

def magnus_cl_from_spin(v_mag: float, spin_rad_s: float) -> float:
    """Very simple lift-coefficient model using spin parameter S = (w*r)/v.
    This is a very rough approximation but kind of works maybe?"""
    if v_mag < 1e-6:
        return 0.0
    S = (spin_rad_s * fuel_radius_m) / v_mag
    return 1.2 * (S / (0.25 + S))

def simulate_shot(v0: float, theta: float, target_distance_m: float, *,
    max_time_s: float = 4.0,
    dt_s: float = dt,
    stop_at_x: float | None = None):
    """Simulate a shot with quadratic drag and Magnus lift. x increases toward the target, y is up.
    Shooter starts at (0, shooter_height_m). Target plane is at x=target_distance_m.
    Returns xs, ys, ts arrays and an interpolated (y_at_target, tof) if crossing occurred.
    """
    x = 0.0
    y = shooter_height_m
    vx = v0 * np.cos(theta)
    vy = v0 * np.sin(theta)

    spin_rad_s = calculate_spin_rad_per_sec(v0)

    xs = [x]
    ys = [y]
    ts = [0.0]

    crossed = None  # (y_at_target, tof)
    x_stop = stop_at_x if stop_at_x is not None else target_distance_m

    t = 0.0
    while t < max_time_s:
        v_mag = float(np.hypot(vx, vy))

        # forces/accelerations
        if v_mag > 1e-9:
            # drag accel
            drag_acc_mag = 0.5 * rho_air * Cd * fuel_area_m2 * v_mag**2 / fuel_mass_kg
            ax_drag = -drag_acc_mag * (vx / v_mag)
            ay_drag = -drag_acc_mag * (vy / v_mag)

            # magnus accel (perpendicular to velocity)
            Cl = magnus_cl_from_spin(v_mag, spin_rad_s)
            magnus_acc_mag = 0.5 * rho_air * Cl * fuel_area_m2 * v_mag**2 / fuel_mass_kg

            # unit vector perpendicular to v: (-vy, vx) / |v|
            ax_mag = magnus_acc_mag * (-vy / v_mag)
            ay_mag = magnus_acc_mag * (vx / v_mag)
        else:
            ax_drag = ay_drag = 0.0
            ax_mag = ay_mag = 0.0

        ax = ax_drag + ax_mag
        ay = ay_drag + ay_mag - g

        # update v then x/y
        vx_next = vx + ax * dt_s
        vy_next = vy + ay * dt_s
        x_next = x + vx_next * dt_s
        y_next = y + vy_next * dt_s

        t_next = t + dt_s

        # detect crossing of the target x-plane (but only the first time)
        if crossed is None and x < target_distance_m <= x_next:
            alpha = (target_distance_m - x) / (x_next - x) if abs(x_next - x) > 1e-12 else 0.0
            y_at_target = y + alpha * (y_next - y)
            tof = t + alpha * (t_next - t)
            crossed = (float(y_at_target), float(tof))

        x, y, vx, vy, t = x_next, y_next, vx_next, vy_next, t_next
        xs.append(x)
        ys.append(y)
        ts.append(t)

        if y < -0.25:
            break
        if x >= x_stop:
            break

    return np.array(xs), np.array(ys), np.array(ts), crossed

def cost_function(params, target_distance):
    v0, hood_angle = params
    theta = hood_angle_to_theta(hood_angle)

    if v0 <= 0.0:
        return 1e9

    xs, ys, ts, crossed = simulate_shot(v0, theta, target_distance)
    if crossed is not None:
        y_at_target, tof = crossed
        height_err = (y_at_target - hub_entrance_height_m)
        flat_penalty = max(0.0, np.radians(20) - theta) ** 2
        x_final = xs[-1]
        x_err = abs(x_final - target_distance)
        
        # Error scaled by how far below the point at the x of the hub entrance is from the entrance height
        # Buncha random constants yay
        entrance_x = target_distance - hub_target_offset - hub_entrance_width_m/2 - inches_to_meters(2)
        y_at_entrance = np.interp(entrance_x, xs, ys) if xs[0] <= entrance_x <= xs[-1] else float('nan')
        entrance_height_err = max(0.0, hub_entrance_height_m + inches_to_meters(5) - y_at_entrance)

        return (
            v0
            + 20000.0 * height_err**2
            + 1.0 * tof  # mild preference for shorter TOF if ties
            + 50.0 * flat_penalty
            + 2000.0 * entrance_height_err**2
        )
    else:
        # if we didn't cross, penalize by how far we missed the target x-plane
        # also penalize for being below the floor
        y_final = ys[-1]
        y_penalty = 0.0
        if y_final < 0:
            y_penalty = abs(y_final) * 1000.0
        x_final = xs[-1]
        x_err = abs(x_final - target_distance)
        return 1e6 + 1e5 * x_err + y_penalty

distances = np.linspace(1.8, 7.0, 50) # target distances in meters
best_params = []

plt.figure(figsize=(10, 6))

first_vel_guess = 6
last_vel = first_vel_guess

for d in distances:
    # Initial guess
    res = minimize(
        cost_function, 
        x0=[
            last_vel, # Initial velocity guess
            np.radians(45) # Initial angle guess
        ], 
        args=(d,),
        bounds=[
            (0.1, 40.0), # Velocity bounds
            (np.radians(20), np.radians(45)) # Hood angle bounds
        ],
        method="SLSQP"
    )
    
    v0_opt, hood_angle_opt = res.x
    theta_opt = hood_angle_to_theta(hood_angle_opt)

    last_vel = v0_opt

    xs, ys, ts, crossed = simulate_shot(v0_opt, theta_opt, d)
    if crossed is None:
        # Shouldn't happen if optimizer succeeded, but be defensive.
        y_at_target, t_hit = float('nan'), float('nan')
    else:
        y_at_target, t_hit = crossed
    
    best_params.append((d, v0_opt, np.degrees(hood_angle_opt), t_hit))
    print(f"Distance: {d:.2f} m -> Velocity: {v0_opt:.2f} m/s, Hood Angle: {np.degrees(hood_angle_opt):.2f} deg, TOF: {t_hit:.2f} s")
    
    # Plot only until the target plane; shift so target is always at x=0.
    valid_idx = xs <= d
    x_shifted = xs[valid_idx] - d

    plt.plot(x_shifted, ys[valid_idx], label=f'Dist={d:.1f}m')
    plt.plot(0, hub_entrance_height_m, 'ro') # Target point
    
    # Turret (8 inches wide) at the starting point
    turret_width_m = 8 * 0.0254
    # TODO: color by something meaningful lol
    plt.plot([-d - turret_width_m/2, -d + turret_width_m/2], [shooter_height_m, shooter_height_m], 'k-', lw=3)

plt.axhline(0, color='brown', linewidth=2, linestyle='solid', label='Floor')

# Hub (40 inches wide) centered at x=-hub_target_offset
plt.plot([-hub_entrance_width_m/2-hub_target_offset, hub_entrance_width_m/2-hub_target_offset], [hub_entrance_height_m, hub_entrance_height_m], 'g-', lw=3, label='Target')

plt.axvline(0, color='black', linewidth=0.5, linestyle='--')

plt.xlabel('Distance (m)')
plt.ylabel('Height (m)')
plt.legend()
plt.grid(True)
plt.savefig('shot_trajectories.png')
print("Saved plot to shot_trajectories.png")


# Plot the best parameters in three plots
distances, velocities, angles, tofs = zip(*best_params)
plt.figure(figsize=(12, 4))

plt.subplot(1, 3, 1)
plt.plot(distances, velocities, 'o-')
plt.xlabel('Distance (m)')
plt.ylabel('Initial Velocity (m/s)')

plt.subplot(1, 3, 2)
plt.plot(distances, angles, 'o-')
plt.xlabel('Distance (m)')
plt.ylabel('Hood Angle (deg)')

plt.subplot(1, 3, 3)
plt.plot(distances, tofs, 'o-')
plt.xlabel('Distance (m)')
plt.ylabel('Time of Flight (s)')
plt.tight_layout()

plt.savefig('shot_parameters.png')

# Save to a csv file
import csv
with open('shot_parameters.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['distance', 'velocity', 'hood', 'tof'])
    writer.writerows(best_params)

# copy shot_parameters.csv to ../../src/main/deploy
import shutil
shutil.copy('shot_parameters.csv', '../../src/main/deploy/hub_shots.csv')