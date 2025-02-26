import yaml
import numpy as np

# Load the transformation data from the NPZ file
npz_file = "./data/extrinsics.npz"
extrinsics_data = np.load(npz_file)

# Extract rotation (3x3) and translation (3x1)
R = extrinsics_data["R"]  # Shape: (3, 3)
t = extrinsics_data["t"].reshape(3, 1)  # Ensure shape: (3, 1)

# Create a 4x4 transformation matrix
T = np.eye(4)  # Initialize as identity matrix
T[:3, :3] = R  # Set rotation part
T[:3, 3] = t.flatten()  # Set translation part

# Convert NumPy array to list for YAML compatibility
transformation_matrix = T.tolist()

# Load existing YAML file
yaml_file = "./gem_e4.yaml"
with open(yaml_file, "r") as file:
    data = yaml.safe_load(file)

# Add the transformation matrix to the YAML data
data["T_toplidar_frontcamera"] = transformation_matrix  # Modify key as needed

# Save the updated YAML file
with open(yaml_file, "w") as file:
    yaml.dump(data, file, default_flow_style=False)

print(f"Transformation matrix added to {yaml_file}")
