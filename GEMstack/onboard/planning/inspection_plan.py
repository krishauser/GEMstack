tateimport numpy as np
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
        else:
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

    max_arc = max(arc_segments, key=len)
    return max_arc


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


circle_center = (5, 6)
radius = 0.5
geofence = ((0, 0), (8, 8))

rem_points = max_visible_arc(circle_center, radius, geofence)
plot_circle_with_geofence_and_arc(circle_center, radius, geofence, rem_points)
