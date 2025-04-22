import numpy as np
import math
import matplotlib.pyplot as plt


def is_inside_geofence(x, y, xmin, xmax, ymin, ymax):
    return xmin < x < xmax and ymin < y < ymax


def max_visible_arc(circle_center, radius, geofence):
    xc, yc = circle_center
    (xmin, ymin), (xmax, ymax) = geofence

    angles = np.linspace(0, 2 * np.pi, 500, endpoint=False)
    arc_segments = []
    curr_segment = []

    first_inside = last_inside = False
    flag_full_circle = True
    tangent_min = 0
    min_index = 0

    for i, theta in enumerate(angles):
        x = xc + radius * np.cos(theta)
        y = yc + radius * np.sin(theta)

        inside = is_inside_geofence(x, y, xmin, xmax, ymin, ymax)

        if i == 0:
            first_inside = inside
        if i == len(angles) - 1:
            last_inside = inside

        if inside:
            curr_segment.append((x, y))
            # Calculate the tangent heading in a clockwise direction
            tangent_heading = -np.arctan2(y - yc, x - xc)  # Clockwise heading
            if abs( 1 - tangent_heading) > abs( 1 - tangent_min):
                if np.arctan2(yc, xc) < np.arctan2(y, x):
                    tangent_min = tangent_heading
                    min_index = i
        else:
            flag_full_circle = False
            if curr_segment:
                arc_segments.append(curr_segment)
                curr_segment = []

    if curr_segment:
        arc_segments.append(curr_segment)

    # If arc wraps around from 2π back to 0, combine first and last segments
    if first_inside and last_inside and len(arc_segments) > 1:
        arc_segments[0] = arc_segments[-1] + arc_segments[0]
        arc_segments.pop()

    if not arc_segments:
        return []

    max_arc = list(max(arc_segments, key=len))
    for i in range(len(max_arc)):
        max_arc[i] = np.array(max_arc[i])
        # max_arc[i] = np.append(max_arc[i], heading_on_circle(xc, yc, max_arc[0][0], max_arc[1][0]))
        max_arc[i] = np.append(max_arc[i], 0)

    if flag_full_circle:
        max_arc = max_arc[min_index:] + max_arc[:min_index]

    return max_arc


def heading_on_circle(cx, cy, px, py):
    dx = px - cx
    dy = py - cy
    tx = -dy
    ty = dx
    return math.atan2(ty, tx)  # Heading in radians


def plot_circle_with_geofence_and_arc(circle_center, radius, geofence, rem_pts):
    fig, ax = plt.subplots()

    # Plot geofence
    (xmin, ymin), (xmax, ymax) = geofence
    rect = plt.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, linewidth=2, edgecolor='red', facecolor='none')
    ax.add_patch(rect)

    rem_pts = np.array(rem_pts)
    ax.plot(rem_pts[:, 0], rem_pts[:, 1], color='blue', linewidth=3, label='Max Valid Arc')

    ax.set_aspect('equal')
    ax.grid(True)
    plt.legend()
    plt.show()


circle_center = (4, -6)
radius = 2
geofence = ((-10, -10), (10, 10))


rem_points = max_visible_arc(circle_center, radius, geofence)
plot_circle_with_geofence_and_arc(circle_center, radius, geofence, rem_points)
print(rem_points)
