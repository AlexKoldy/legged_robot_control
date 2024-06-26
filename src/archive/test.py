import numpy as np
from scipy.interpolate import CubicSpline


def generate_footstep_trajectory(initial_point, final_point, peak_height, num_points):
    # Split initial and final points into x, y, z coordinates
    x0, y0, z0 = initial_point
    xf, yf, zf = final_point

    # Calculate the midpoint between initial and final points
    xm = (x0 + xf) / 2.0
    ym = (y0 + yf) / 2.0

    # Calculate the peak point
    xp = xm
    yp = ym
    zp = peak_height

    # Generate time variable
    t = np.linspace(0, 1, num_points)

    # Generate x, y, z trajectories
    x = x0 + (xf - x0) * t
    y = y0 + (yf - y0) * t
    z = (
        z0
        + (zf - z0) * t
        - ((t - 0.5) ** 2) * (zf - z0)
        + zp
        * np.exp(-((x - xp) ** 2 + (y - yp) ** 2) / ((xf - x0) ** 2 + (yf - y0) ** 2))
    )

    # Ensure velocity is 0 at the endpoints
    t_quintic = np.linspace(0, 1, num_points)
    t_quintic[0] = 0.0
    t_quintic[-1] = 1.0

    # Perform quintic spline interpolation for each coordinate
    cs_x = CubicSpline(t_quintic, x, bc_type="clamped")
    cs_y = CubicSpline(t_quintic, y, bc_type="clamped")
    cs_z = CubicSpline(t_quintic, z, bc_type="clamped")

    # Evaluate the interpolated functions
    x = cs_x(t)
    y = cs_y(t)
    z = cs_z(t)

    return x, y, z


def get_trajectory_derivative(x, y, z):
    # Calculate the time step
    dt = 1 / (len(x) - 1)

    # Calculate the derivative using central difference approximation
    dx = np.gradient(x, dt)
    dy = np.gradient(y, dt)
    dz = np.gradient(z, dt)

    return dx, dy, dz


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Generate the footstep trajectory
initial_point = (0, 0, 0)
final_point = (1, 2, 0)
peak_height = 0.5
num_points = 100
x, y, z = generate_footstep_trajectory(
    initial_point, final_point, peak_height, num_points
)

# Get the trajectory derivatives
dx, dy, dz = get_trajectory_derivative(x, y, z)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot the trajectory
ax.plot(x, y, z, label="Trajectory")

# Plot the derivatives
ax.plot(x, y, dz, label="dz/dt")

# Set labels and title
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Footstep Trajectory and Derivatives")

# Add a legend
ax.legend()

# Show the plot
plt.show()


t = np.linspace(0, 1, num_points)
plt.figure()
plt.plot(t, x, label="x")
plt.plot(t, y, label="y")
plt.plot(t, z, label="z")
plt.plot(t, dx, label="dx")
plt.plot(t, dy, label="dy")
plt.plot(t, dz, label="dz")
plt.legend()
plt.show()
