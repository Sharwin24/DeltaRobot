import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from matplotlib.path import Path  # Import for checking points inside polygon

# Load workspace points dataset
file_path = "workspace_points.csv"  # Update the path if needed
workspace_df = pd.read_csv(file_path)

# Filter points that intersect with the Z=-180 plane (with some tolerance)
z_plane = -180
tolerance = 1.0
plane_points = workspace_df[np.isclose(
    workspace_df["Z"], z_plane, atol=tolerance)]
x_plane_points = plane_points["X"].values
y_plane_points = plane_points["Y"].values

# Compute the convex hull to determine the actual boundary of the reachable workspace
hull = ConvexHull(np.column_stack((x_plane_points, y_plane_points)))

# We want to avoid going too close to the boundary, so add some padding
padding = 5  # [mm]
hull_points = hull.points[hull.vertices]
hull_x, hull_y = hull_points[:, 0], hull_points[:, 1]
hull_center = np.mean(hull_points, axis=0)

# Compute padded hull points
shrink_factor = 0.9  # Adjust this to control shrinkage
hull_x_padded = hull_center[0] + shrink_factor * (hull_x - hull_center[0])
hull_y_padded = hull_center[1] + shrink_factor * (hull_y - hull_center[1])


# Create a polygon from the padded hull for point containment checks
padded_hull_polygon = Path(np.column_stack((hull_x_padded, hull_y_padded)))

# Define scanning parameters
y_step = 5  # Step size in Y direction
num_points = 100  # Points per scan line

x_traj = []
y_traj = []
z_traj = []

# Generate evenly spaced Y values within the valid range
scan_y_values = np.arange(np.max(hull_y_padded),
                          np.min(hull_y_padded), -y_step)

moving_left = True  # Direction flag

# Generate the scan trajectory using the padded hull
for y in scan_y_values:
    # Find X values within the Padded Hull at this Y level
    x_vals_at_y = [
        x for x in np.linspace(np.min(hull_x_padded), np.max(hull_x_padded), num_points)
        if padded_hull_polygon.contains_point((x, y))
    ]

    if len(x_vals_at_y) == 0:
        continue  # Skip if no points at this level

    leftmost_x = np.min(x_vals_at_y)
    rightmost_x = np.max(x_vals_at_y)

    # Generate scan path at this Y level
    if moving_left:
        x_traj.extend(np.linspace(rightmost_x, leftmost_x, num_points))
    else:
        x_traj.extend(np.linspace(leftmost_x, rightmost_x, num_points))

    y_traj.extend([y] * num_points)
    z_traj.extend([z_plane] * num_points)

    # Toggle direction for next row
    moving_left = not moving_left

# Plot optimized snake scan path using padded hull
plt.figure(figsize=(8, 8))
plt.scatter(x_plane_points, y_plane_points,
            color='red', s=2, label='Workspace Points')
plt.plot(hull_x, hull_y, 'g-', linewidth=2, label='Original Boundary')
plt.plot(hull_x_padded, hull_y_padded, 'purple',
         linestyle="--", linewidth=2, label='Padded Hull')
plt.plot(x_traj, y_traj, 'b-', linewidth=1, label='Optimized Snake Scan Path')
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Optimized Snake Scan Path Using Padded Hull")
plt.grid()
plt.savefig("images/optimized_snake_scan.png")
plt.show()

# Save trajectory to a CSV file
scan_trajectory_df = pd.DataFrame({"X": x_traj, "Y": y_traj, "Z": z_traj})
scan_trajectory_df.to_csv("csv_files/scan_trajectory.csv", index=False)

print("Optimized scan trajectory saved to scan_trajectory.csv")
