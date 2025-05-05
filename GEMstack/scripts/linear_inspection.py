import matplotlib.pyplot as plt
import numpy as np

# Define geofence and inspection area
geofence = ((0, 0), (20, 20))  # xmin, ymin, xmax, ymax
inspection_area = ((8, 8), (12, 12))  # xmin, ymin, xmax, ymax
margin = 1.0  # margin from inspection area for vehicle path

def create_path_around_inspection(inspection_area, geofence, margin=1.0):
    (ixmin, iymin), (ixmax, iymax) = inspection_area
    (gxmin, gymin), (gxmax, gymax) = geofence

    # Define top path
    top_y = iymax + margin
    bottom_y = iymin - margin

    top_possible = gymin < top_y < gymax
    bottom_possible = gymin < bottom_y < gymax

    top_path = [(ixmin - margin, top_y), (ixmax + margin, top_y)] if top_possible else None
    bottom_path = [(ixmax + margin, bottom_y), (ixmin - margin, bottom_y)] if bottom_possible else None

    if top_possible and bottom_possible:
        full_path = top_path + bottom_path[::-1]
    elif top_possible:
        full_path = top_path
    elif bottom_possible:
        full_path = bottom_path
    else:
        full_path = []

    return full_path, top_path, bottom_path

# Generate paths
full_path, top_path, bottom_path = create_path_around_inspection(inspection_area, geofence, margin=margin)

# Plotting
fig, ax = plt.subplots(figsize=(8, 8))
(axmin, aymin), (axmax, aymax) = geofence
ax.set_xlim(axmin - 1, axmax + 1)
ax.set_ylim(aymin - 1, aymax + 1)

# Draw geofence
geofence_rect = plt.Rectangle((axmin, aymin), axmax - axmin, aymax - aymin,
                              edgecolor='black', facecolor='none', linewidth=2, linestyle='--')
ax.add_patch(geofence_rect)

# Draw inspection area
(ixmin, iymin), (ixmax, iymax) = inspection_area
inspection_rect = plt.Rectangle((ixmin, iymin), ixmax - ixmin, iymax - iymin,
                                edgecolor='red', facecolor='none', linewidth=2)
ax.add_patch(inspection_rect)

# Draw paths
if top_path:
    tx, ty = zip(*top_path)
    ax.plot(tx, ty, 'bo-', label='Top Path')
if bottom_path:
    bx, by = zip(*bottom_path)
    ax.plot(bx, by, 'go-', label='Bottom Path')

if full_path:
    fx, fy = zip(*full_path)
    ax.plot(fx, fy, 'k--', label='Full Path')

ax.set_aspect('equal')
ax.set_title("Inspection Path Around Object")
ax.legend()
plt.grid(True)
plt.show()
