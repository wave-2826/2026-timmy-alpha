import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib.collections import LineCollection
from enum import Enum
from typing import Optional, Callable

def inches_to_meters(inches: float) -> float:
    return inches * 0.0254

# Constants
g = 9.81  # m/s^2
dt = 0.002 # s
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

hub_entrance_height_m = inches_to_meters(72)
hub_entrance_width_m = inches_to_meters(40)
hub_target_offset = inches_to_meters(14) # inches back
hub_margin_m = (-inches_to_meters(2), inches_to_meters(5))

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
    return np.radians(90) - hood_angle_rad + np.radians(5)

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
    stop_at_x: float | None = None,
    magnus: bool = True):
    """Simulate a shot with quadratic drag and Magnus lift. x increases toward the target, y is up.
    Shooter starts at (0, shooter_height_m). Target plane is at x=target_distance_m.
    Returns xs, ys, ts, and velocities arrays and an interpolated (y_at_target, tof) if crossing occurred.
    """
    x = 0.0
    y = shooter_height_m
    vx = v0 * np.cos(theta)
    vy = v0 * np.sin(theta)

    spin_rad_s = calculate_spin_rad_per_sec(v0)

    xs = [x]
    ys = [y]
    velocities = [v0]
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

            if not magnus:
                ax_mag = ay_mag = 0.0
            else:
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
        velocities.append(v_mag)

        if y < -0.25:
            break
        if x >= x_stop:
            break

    return np.array(xs), np.array(ys), np.array(ts), np.array(velocities), crossed

def cost_function(params, target_distance, target_height, extra_cost: Optional[Callable] = None):
    v0, hood_angle = params
    theta = hood_angle_to_theta(hood_angle)

    if v0 <= 0.0:
        return 1e9

    xs, ys, ts, velocities, crossed = simulate_shot(v0, theta, target_distance)
    if crossed is not None:
        y_at_target, tof = crossed
        height_err = (y_at_target - target_height)
        x_final = xs[-1]
        x_err = abs(x_final - target_distance)

        return (
            v0
            + 20000.0 * height_err**2
            + 1.0 * tof  # mild preference for shorter TOF if ties
            + (extra_cost(target_height, target_distance, xs, ys) if extra_cost is not None else 0.0)
        )
    else:
        # If we didn't cross, calculate the closest approach to the target
        closest_x = min(xs, key=lambda x: abs(x - target_distance))
        closest_y = ys[np.argmin([abs(x - target_distance) for x in xs])]
        x_err = abs(closest_x - target_distance)
        y_err = abs(closest_y - target_height)

        return (
            v0
            + 1e5 * x_err
            + 1e4 * y_err
            + (abs(closest_y) * 1000.0 if closest_y < 0 else 0.0)  # Penalize shots that fall below ground
            + (extra_cost(target_height, target_distance, xs, ys) if extra_cost is not None else 0.0)
        )

def simulate_and_save(
    suffix: str,
    csv_path: str,
    target_height: float,
    distances: list[float],
    plot_extra_cb: Optional[Callable] = None,
    extra_cost: Optional[Callable] = None
):
    best_params = []

    plt.figure(figsize=(10, 6))

    first_vel_guess = 6
    last_vel = first_vel_guess
    last_angle = np.radians(45)

    for i, d in enumerate(distances):
        # Initial guess
        res = minimize(
            cost_function, 
            x0=[
                last_vel, # Initial velocity guess
                last_angle # Initial angle guess
            ], 
            args=(d, target_height, extra_cost),
            bounds=[
                (0.1, 15.0), # Velocity bounds
                (np.radians(20), np.radians(45)) # Hood angle bounds
            ],
            method="SLSQP"
        )
        
        v0_opt, hood_angle_opt = res.x
        theta_opt = hood_angle_to_theta(hood_angle_opt)

        last_vel = v0_opt
        last_angle = hood_angle_opt

        xs, ys, ts, velocities, crossed = simulate_shot(v0_opt, theta_opt, d)
        if crossed is None:
            # Shouldn't happen if optimizer succeeded, but be defensive.
            y_at_target, t_hit = float('nan'), float('nan')
        else:
            y_at_target, t_hit = crossed
        
        if np.isfinite(y_at_target) and np.isfinite(t_hit):
            best_params.append((d, v0_opt, np.degrees(hood_angle_opt), t_hit))
        else:
            print("Degenerate shot: ", end='')
        
        print(f"Distance: {d:.2f} m -> Velocity: {v0_opt:.2f} m/s, Hood Angle: {np.degrees(hood_angle_opt):.2f} deg, TOF: {t_hit:.2f} s")
        
        # Plot only until the target plane; shift so target is always at x=0.
        valid_idx = xs <= d
        x_shifted = xs[valid_idx] - d

        if i % 5 == 0:
            plt.plot(x_shifted, ys[valid_idx], label=f'Dist={d:.1f}m')
            plt.plot(0, target_height, 'ro') # Target point
            
            # Turret (8 inches wide) at the starting point
            turret_width_m = 8 * 0.0254
            
            # Color trajectory by velocity
            points = np.array([x_shifted, ys[valid_idx]]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            norm = plt.Normalize(velocities[valid_idx].min(), velocities[valid_idx].max())
            lc = LineCollection(segments, cmap='viridis', norm=norm)
            lc.set_array(velocities[valid_idx][:-1])
            lc.set_linewidth(2)
            plt.gca().add_collection(lc)
            plt.plot(0, target_height, 'ro') # Target point

            # Also plot a no-magnus shot for comparison (dashed line)
            xs_nm, ys_nm, _, _, _ = simulate_shot(v0_opt, theta_opt, d, magnus=False)
            valid_idx_nm = xs_nm <= d
            x_shifted_nm = xs_nm[valid_idx_nm] - d
            plt.plot(x_shifted_nm, ys_nm[valid_idx_nm], 'k--', alpha=0.4, label='No Magnus' if i == 0 else None)

            # Turret (8 inches wide) at the starting point
            turret_width_m = 8 * 0.0254
            plt.plot([-d - turret_width_m/2, -d + turret_width_m/2], [shooter_height_m, shooter_height_m], 'k-', lw=3)

    plt.axhline(0, color='brown', linewidth=2, linestyle='solid', label='Floor')

    if plot_extra_cb is not None:
        plot_extra_cb()

    plt.axvline(0, color='black', linewidth=0.5, linestyle='--')

    plt.xlabel('Distance (m)')
    plt.ylabel('Height (m)')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'shot_trajectories_{suffix}.png')
    print(f"Saved plot to shot_trajectories_{suffix}.png")

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
    # Make sure the graph doesn't scale down even with numerical error
    plt.ylim(bottom=20)

    plt.subplot(1, 3, 3)
    plt.plot(distances, tofs, 'o-')
    plt.xlabel('Distance (m)')
    plt.ylabel('Time of Flight (s)')
    plt.tight_layout()

    plt.savefig(f'shot_parameters_{suffix}.png')

    # Save to a csv file
    import csv
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['distance', 'velocity', 'hood', 'tof'])
        writer.writerows(best_params)

def plot_hub():
    # Hub (40 inches wide) centered at x=-hub_target_offset
    plt.plot([-hub_entrance_width_m/2-hub_target_offset, hub_entrance_width_m/2-hub_target_offset], [hub_entrance_height_m, hub_entrance_height_m], 'g-', lw=3, label='Target')

def hub_margin_cost(target_height, target_distance, xs, ys):
    # Error scaled by how far below the point at the x of the hub entrance is from the entrance height
    # Buncha random constants yay
    entrance_x = target_distance - hub_target_offset - hub_entrance_width_m/2 + hub_margin_m[0]
    y_at_entrance = np.interp(entrance_x, xs, ys) if xs[0] <= entrance_x <= xs[-1] else float('nan')
    entrance_height_err = max(0.0, target_height + hub_margin_m[1] - y_at_entrance)

    return 2000. * entrance_height_err

hub_distances = np.linspace(1.8, 7.0, 50) # target distances in meters
lob_distances = np.linspace(1.8, 9.0, 50) # target distances in meters

simulate_and_save('hub', '../../src/main/deploy/hub_shots.csv', hub_entrance_height_m, hub_distances, plot_hub, hub_margin_cost)
simulate_and_save('lob', '../../src/main/deploy/lob_shots.csv', 0., lob_distances)