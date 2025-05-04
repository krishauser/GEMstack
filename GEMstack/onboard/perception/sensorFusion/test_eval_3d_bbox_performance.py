import os
import sys
import numpy as np
import traceback
import shutil


eval_script_filename = "eval_3d_bbox_performance.py"
eval_module_name = "eval_3d_bbox_performance"
try:
    if not os.path.exists(eval_script_filename):
         raise FileNotFoundError(f"Evaluation script '{eval_script_filename}' not found in the current directory.")

    # Attempt the standard import
    imported_module = __import__(eval_module_name)
    print(f"Module '{eval_module_name}' imported successfully.")

except:
    print("##################")
    print(f"{eval_script_filename} import error")
    print("##################")
    exit(1)

#### Generate Dummy Data in KITTI Format  #####

def create_dummy_kitti_data(base_dir, num_samples=3, classes=['Car', 'Pedestrian'], boxes_per_sample=5, is_pred=False, noise_level=0.1, seed=42):
    """Generates dummy data files in KITTI format."""
    if os.path.exists(base_dir):
        shutil.rmtree(base_dir) # Clean previous runs
    os.makedirs(base_dir)
    np.random.seed(seed) # reproducibility

    for i in range(num_samples):
        filename = os.path.join(base_dir, f"{i:06d}.txt")
        with open(filename, 'w') as f:
            num_boxes = np.random.randint(1, boxes_per_sample + 1)
            for _ in range(num_boxes):
                cls = np.random.choice(classes)

                # Generate somewhat  box parameters
                h = np.random.uniform(1.4, 1.8) if cls == 'Car' else np.random.uniform(1.5, 1.9) # height
                w = np.random.uniform(1.5, 2.0) if cls == 'Car' else np.random.uniform(0.5, 1.0) # width
                l = np.random.uniform(3.5, 5.0) if cls == 'Car' else np.random.uniform(0.5, 1.0) # length

                loc_x = np.random.uniform(-15, 15) # center x (lateral)
                loc_z = np.random.uniform(5, 50)   # center z (depth)
                loc_y_bottom = np.random.uniform(1.6, 1.7) # Approximate height of bottom relative to camera origin
                rot_y = np.random.uniform(-np.pi/2, np.pi/2) # Yaw

                # Placeholder values
                truncated = 0.0
                occluded = 0 # 0=visible
                alpha = -10
                bbox_2d = [0.0, 0.0, 50.0, 50.0]
                score = np.random.uniform(0.5, 1.0)

                # Add noise for predictions
                if is_pred:
                    h *= np.random.normal(1, noise_level * 0.1)
                    w *= np.random.normal(1, noise_level * 0.1)
                    l *= np.random.normal(1, noise_level * 0.1)
                    loc_x += np.random.normal(0, noise_level * 1.0)
                    loc_y_bottom += np.random.normal(0, noise_level * 0.1)
                    loc_z += np.random.normal(0, noise_level * 3.0)
                    rot_y += np.random.normal(0, noise_level * np.pi/8)
                    h, w, l = max(0.1, h), max(0.1, w), max(0.1, l) # Ensure positive dimensions

                # Format the line string to KITTI standard
                line_parts = [
                    cls, f"{truncated:.2f}", f"{occluded:d}", f"{alpha:.2f}",
                    f"{bbox_2d[0]:.2f}", f"{bbox_2d[1]:.2f}", f"{bbox_2d[2]:.2f}", f"{bbox_2d[3]:.2f}",
                    f"{h:.2f}", f"{w:.2f}", f"{l:.2f}",
                    f"{loc_x:.2f}", f"{loc_y_bottom:.2f}", f"{loc_z:.2f}",
                    f"{rot_y:.2f}"
                ]

                if is_pred:
                    line_parts.append(f"{score:.4f}")

                f.write(" ".join(line_parts) + "\n")

# --- Setup Directories ---
base_output_dir = "./kitti_eval_demo"
gt_dir = os.path.join(base_output_dir, "dummy_gt_kitti")
pred1_dir = os.path.join(base_output_dir, "dummy_pred1_kitti")
pred2_dir = os.path.join(base_output_dir, "dummy_pred2_kitti")
eval_output_dir = os.path.join(base_output_dir, "eval_output")

# --- Generate KITTI-formatted Dummy Data ---
print("\n[LOG] Generating dummy data (KITTI format) for demonstration...")
try:
    create_dummy_kitti_data(gt_dir, num_samples=5, classes=['Car', 'Pedestrian'], is_pred=False, seed=42)
    create_dummy_kitti_data(pred1_dir, num_samples=5, classes=['Car', 'Pedestrian'], is_pred=True, noise_level=0.1, seed=101)
    create_dummy_kitti_data(pred2_dir, num_samples=5, classes=['Car', 'Pedestrian'], is_pred=True, noise_level=0.3, seed=202)
    missing_pred_file = os.path.join(pred2_dir, "000004.txt")
    if os.path.exists(missing_pred_file):
        os.remove(missing_pred_file)
        print(f"Removed {missing_pred_file} to simulate missing prediction.")
    print("Dummy data generated.")
except Exception as e:
    print(f"Error generating dummy data: {e}")
    traceback.print_exc()
    exit(1)

# --- Run the Evaluation Script ---
print(f"\n[LOG] ####  Run Evaluation ")
cmd_args = [
    gt_dir,
    pred1_dir,
    pred2_dir,
    eval_output_dir,
    '--detector_names', 'DetectorA_Imported', 'DetectorB_Imported',
    '--iou_threshold', '0.7',
    '--classes', 'Car', 'Pedestrian'
]


original_argv = sys.argv
sys.argv = [eval_script_filename] + cmd_args
exit_code = 0

try:
    print(f"[LOG] call {eval_module_name}.main()...")
    imported_module.main()

except AttributeError:
     exit_code = 1
except Exception as e:
    traceback.print_exc()
    exit_code = 1
finally:
    # Restore original sys.argv
    sys.argv = original_argv

#####  List generated output files ##### 
print("\n--- Generated Output Files ---")
if os.path.exists(eval_output_dir):
    try:
        output_files = [os.path.join(eval_output_dir, item) for item in os.listdir(eval_output_dir)]
        if output_files:
            for item_path in sorted(output_files): # Sort for consistent listing
                print(item_path)
        else:
            print(f"Output directory '{eval_output_dir}' is empty.")
    except Exception as e:
        print(f"Error listing output directory {eval_output_dir}: {e}")
else:
    print(f"Output directory '{eval_output_dir}' not created or accessible.")

# Display contents of the metrics file if execution was successful
metrics_file = os.path.join(eval_output_dir, 'evaluation_metrics.txt')
if exit_code == 0 and os.path.exists(metrics_file):
    print(f"\n--- Content of {metrics_file} ---")
    try:
        with open(metrics_file, 'r') as f:
            print(f.read())
    except Exception as e:
        print(f"Error reading metrics file {metrics_file}: {e}")
elif exit_code != 0:
    # Use syntax compatible with Python < 3.8
    print(f"\n--- Metrics file not displayed due to execution error (exit_code={exit_code}) ---")
elif not os.path.exists(metrics_file):
     print(f"\n--- Metrics file not found: {metrics_file} ---")

print(f"\nDemo finished. Check the '{base_output_dir}' directory for generated data and results.")