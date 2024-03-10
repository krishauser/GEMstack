import numpy as np

# Define the two vectors
v1 = np.array([1, 0, 0])  # Replace x1, y1, z1 with the coordinates of v1
v2 = np.array([0.8102, 0., 1.0020])  # Replace x2, y2, z2 with the coordinates of v2

# Calculate the forward vector (normalized)
forward_vector = (v2 - v1) / np.linalg.norm(v2 - v1)

# Define the up vector (assuming global up is [0, 1, 0])
up_vector = np.array([0, 1, 0])

# Calculate the right vector (normalized)
right_vector = np.cross(forward_vector, up_vector)
right_vector /= np.linalg.norm(right_vector)

# Re-calculate the up vector to ensure orthogonality
up_vector = np.cross(right_vector, forward_vector)

# Construct the rotation matrix
rotation_matrix = np.column_stack((right_vector, up_vector, forward_vector))

# Print the rotation matrix
print("Rotation Matrix:")
print(rotation_matrix)
