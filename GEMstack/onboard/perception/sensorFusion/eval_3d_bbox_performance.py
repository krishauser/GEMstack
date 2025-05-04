import argparse
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import defaultdict
import math
import warnings
import traceback

######  Bbox coord mapping ######
IDX_X, IDX_Y, IDX_Z = 0, 1, 2
IDX_L, IDX_W, IDX_H = 3, 4, 5
IDX_YAW = 6
IDX_CLASS = 7
IDX_SCORE = 8

######  Helper Functions ######

def parse_box_line(line, is_gt=False):
    """
    Parses a line from a KITTI format label file.
    Handles the conversion from KITTI's y-coordinate (bottom of box)
    to the geometric center y used internally.
    """
    parts = line.strip().split()
    try:
        # Parses standard KITTI format:
        # type truncated occluded alpha bbox_2d(4) dims(3) loc(3) rotation_y (score)
        # 0    1          2        3     4-7       8-10    11-13  14           (15)

        # Check minimum length for standard KITTI format (15 fields without score)
        if len(parts) < 15:
             raise ValueError(f"Line does not have enough parts for KITTI format (expected >= 15, got {len(parts)})")

        class_label = parts[0]

        # Read dimensions (h, w, l) from KITTI standard indices 8, 9, 10
        h = float(parts[8])
        w = float(parts[9])
        l = float(parts[10])

        # Read location (x, y_bottom, z) from KITTI standard indices 11, 12, 13
        loc_x = float(parts[11])
        loc_y_bottom = float(parts[12]) # KITTI 'y' is bottom of the box in camera coords
        loc_z = float(parts[13])

        # Read rotation_y (yaw) from KITTI standard index 14
        rot_y = float(parts[14])

        # Convert to internal format
        # Internal format: [cx, cy, cz, l, w, h, yaw, class_label, score]

        # Use KITTI loc_x and loc_z directly for internal cx, cz
        cx = loc_x
        cz = loc_z

        # Convert KITTI y (bottom of box) to geometric center y for internal use.
        # In KITTI camera coords, Y points down, so center is 'above' bottom (numerically smaller y): loc_y - h / 2
        cy = loc_y_bottom - (h / 2.0)

        # Use KITTI rot_y directly for internal yaw
        yaw = rot_y

        # Create the standardized box list [cx, cy, cz, l, w, h, yaw, class_label]
        # Note the internal dimension order (l, w, h) differs from KITTI input order (h, w, l)
        box = [cx, cy, cz, l, w, h, yaw, class_label]

        # Append score if it's a prediction file (standard KITTI index 15)
        if not is_gt:
            if len(parts) > 15:
                 score = float(parts[15])
            else:
                 # Handle missing score in prediction file if needed
                 score = 1.0 # Assigning default 1.0 as before
                 warnings.warn(f"[LOG] Prediction line missing score (field 16), assigning default 1.0: {line.strip()}")
            box.append(score) # Append score using IDX_SCORE implicitly

        return box

    except ValueError as ve:
         print(f"[LOG] Error parsing line: '{line.strip()}' - {ve}")
         print("[LOG] Does input follows KITTI format ?")
         return None
    except Exception as e:
        print(f"[LOG] Unexpected error parsing line: '{line.strip()}' - {e}")
        traceback.print_exc() # Print detailed traceback for unexpected errors
        return None


def load_labels(file_path, is_gt=False):
    """Loads ground truth or prediction boxes from a KITTI-format file."""
    boxes = defaultdict(list)
    if not os.path.exists(file_path):
        # Reduced verbosity for missing prediction files (common)
        # print(f"[LOG] File not found {file_path}, returning empty dict.")
        return boxes
    try:
        with open(file_path, 'r') as f:
            for i, line in enumerate(f):
                box = parse_box_line(line, is_gt=is_gt)
                if box:
                    class_label = box[IDX_CLASS]
                    boxes[class_label].append(box)
                else:
                    print(f"[LOG] Skipping invalid line {i+1} in {file_path}")
    except Exception as e:
        print(f"[LOG] Error reading file {file_path}: {e}")
        traceback.print_exc()
    return boxes

def calculate_3d_iou(box1, box2):
    """
    Calculates the 3D Intersection over Union (IoU) between two bounding boxes.

    Args:
        box1, box2: List or tuple representing a 3D bounding box in the
                    *internal standardized format*: [cx, cy, cz, l, w, h, yaw, ...]
                    where cy is the geometric center y.

    Returns:
        float: The 3D IoU value.

    Doesn't consider yaw
    """

    ####### Simple Axis-Aligned Bounding Box (AABB) IoU #######
    def get_aabb_corners(box):
        # Uses internal format where cy is geometric center
        cx, cy, cz, l, w, h = box[IDX_X], box[IDX_Y], box[IDX_Z], box[IDX_L], box[IDX_W], box[IDX_H]
        # Ignores yaw
        min_x, max_x = cx - l / 2, cx + l / 2
        min_y, max_y = cy - h / 2, cy + h / 2 # Uses geometric center cy
        min_z, max_z = cz - w / 2, cz + w / 2 # Assuming cz is center z (depth)
        return min_x, max_x, min_y, max_y, min_z, max_z

    min_x1, max_x1, min_y1, max_y1, min_z1, max_z1 = get_aabb_corners(box1)
    min_x2, max_x2, min_y2, max_y2, min_z2, max_z2 = get_aabb_corners(box2)

    # Calculate intersection volume
    inter_min_x = max(min_x1, min_x2)
    inter_max_x = min(max_x1, max_x2)
    inter_min_y = max(min_y1, min_y2)
    inter_max_y = min(max_y1, max_y2)
    inter_min_z = max(min_z1, min_z2)
    inter_max_z = min(max_z1, max_z2)

    inter_l = max(0, inter_max_x - inter_min_x)
    inter_h = max(0, inter_max_y - inter_min_y)
    inter_w = max(0, inter_max_z - inter_min_z)
    intersection_volume = inter_l * inter_h * inter_w

    # Calculate union volume
    vol1 = box1[IDX_L] * box1[IDX_W] * box1[IDX_H]
    vol2 = box2[IDX_L] * box2[IDX_W] * box2[IDX_H]
    # Ensure volumes are positive and non-zero for stable IoU
    vol1 = max(vol1, 1e-6)
    vol2 = max(vol2, 1e-6)
    union_volume = vol1 + vol2 - intersection_volume
    union_volume = max(union_volume, 1e-6) # Avoid division by zero or very small numbers

    iou = intersection_volume / union_volume
    # Clamp IoU to [0, 1] range
    iou = max(0.0, min(iou, 1.0))
    return iou


def calculate_ap(precision, recall):
    """Calculates Average Precision using the PASCAL VOC method (area under monotonic PR curve)."""
    # Convert to numpy arrays first for safety
    if not isinstance(precision, (list, np.ndarray)) or not isinstance(recall, (list, np.ndarray)):
       return 0.0
    precision = np.array(precision)
    recall = np.array(recall)

    if precision.size == 0 or recall.size == 0:
        return 0.0

    # Prepend/Append points for interpolation boundaries
    recall = np.concatenate(([0.], recall, [1.0]))
    precision = np.concatenate(([0.], precision, [0.])) # Start with 0 precision at recall 0, end with 0 at recall 1

    # Make precision monotonically decreasing (handles PR curve 'jiggles')
    for i in range(len(precision) - 2, -1, -1):
        precision[i] = max(precision[i], precision[i+1])

    # Find indices where recall changes (avoids redundant calculations)
    indices = np.where(recall[1:] != recall[:-1])[0] + 1

    # Compute AP using the area under the curve (sum of rectangle areas)
    ap = np.sum((recall[indices] - recall[indices-1]) * precision[indices])
    return ap

def evaluate_detector(gt_boxes_all_samples, pred_boxes_all_samples, classes, iou_threshold):
    """Evaluates a single detector's predictions against ground truth."""
    results_by_class = {}
    sample_ids = list(gt_boxes_all_samples.keys()) # Get fixed order of sample IDs

    for cls in classes:
        all_pred_boxes_cls = []
        num_gt_cls = 0
        pred_sample_indices = [] # Store index from sample_ids for each prediction

        # Collect all GTs and Preds for this class across samples
        for i, sample_id in enumerate(sample_ids):
            # Use .get() with default empty dict/list for safety
            gt_boxes = gt_boxes_all_samples.get(sample_id, {}).get(cls, [])
            pred_boxes = pred_boxes_all_samples.get(sample_id, {}).get(cls, [])

            num_gt_cls += len(gt_boxes)
            for box in pred_boxes:
                all_pred_boxes_cls.append(box)
                pred_sample_indices.append(i) # Store the original sample index

        if not all_pred_boxes_cls: # Handle case with no predictions for this class
             results_by_class[cls] = {
                'precision': np.array([]), # Use empty numpy arrays
                'recall': np.array([]),
                'ap': 0.0,
                'num_gt': num_gt_cls,
                'num_pred': 0
             }
             continue # Skip to next class

        # Sort detections by confidence score (descending)
        # Ensure scores exist and are numeric before sorting
        scores = []
        valid_indices_for_sorting = []
        for idx, box in enumerate(all_pred_boxes_cls):
             if len(box) > IDX_SCORE and isinstance(box[IDX_SCORE], (int, float)):
                 scores.append(-box[IDX_SCORE]) # Use negative score for descending sort with argsort
                 valid_indices_for_sorting.append(idx)
             else:
                  warnings.warn(f"Class {cls}: Prediction missing score or invalid score type. Excluding from evaluation: {box}")

        if not valid_indices_for_sorting: # If filtering removed all boxes
            results_by_class[cls] = {'precision': np.array([]),'recall': np.array([]),'ap': 0.0,'num_gt': num_gt_cls,'num_pred': 0}
            continue

        # Filter lists based on valid scores
        all_pred_boxes_cls = [all_pred_boxes_cls[i] for i in valid_indices_for_sorting]
        pred_sample_indices = [pred_sample_indices[i] for i in valid_indices_for_sorting]
        # Scores list is already built correctly

        # Get the sorted order based on scores
        sorted_indices = np.argsort(scores) # argsort sorts ascending on negative scores -> descending order of original scores

        # Reorder the lists based on sorted scores
        all_pred_boxes_cls = [all_pred_boxes_cls[i] for i in sorted_indices]
        pred_sample_indices = [pred_sample_indices[i] for i in sorted_indices]


        tp = np.zeros(len(all_pred_boxes_cls))
        fp = np.zeros(len(all_pred_boxes_cls))
        # Track matched GTs per sample: gt_matched[sample_idx][gt_box_idx] = True/False
        gt_matched = defaultdict(lambda: defaultdict(bool)) # Indexed by sample_idx, then gt_idx

        # Match predictions
        for det_idx, pred_box in enumerate(all_pred_boxes_cls):
            sample_idx = pred_sample_indices[det_idx] # Get the original sample index (0 to num_samples-1)
            sample_id = sample_ids[sample_idx] # Get the sample_id string using the index
            gt_boxes = gt_boxes_all_samples.get(sample_id, {}).get(cls, [])

            best_iou = -1.0
            best_gt_idx = -1 # Index relative to gt_boxes for this sample/class

            if not gt_boxes: # No GT for this class in this specific sample
                fp[det_idx] = 1
                continue

            for gt_idx, gt_box in enumerate(gt_boxes):
                # Explicitly check class match (belt-and-suspenders)
                if pred_box[IDX_CLASS] == gt_box[IDX_CLASS]:
                     iou = calculate_3d_iou(pred_box, gt_box)
                     if iou > best_iou:
                         best_iou = iou
                         best_gt_idx = gt_idx
                # else: # Should not happen if inputs are correctly filtered by class
                #     pass


            if best_iou >= iou_threshold:
                # Check if this GT box was already matched *in this sample*
                if not gt_matched[sample_idx].get(best_gt_idx, False):
                    tp[det_idx] = 1
                    gt_matched[sample_idx][best_gt_idx] = True # Mark as matched for this sample
                else:
                    fp[det_idx] = 1 # Matched a GT box already covered by a higher-scored prediction
            else:
                fp[det_idx] = 1 # Did not match any available GT box with sufficient IoU

        # Calculate precision/recall
        fp_cumsum = np.cumsum(fp)
        tp_cumsum = np.cumsum(tp)

        # Avoid division by zero if num_gt_cls is 0
        recall = tp_cumsum / num_gt_cls if num_gt_cls > 0 else np.zeros_like(tp_cumsum, dtype=float)

        # Avoid division by zero if no predictions were made or matched (tp + fp = 0)
        denominator = tp_cumsum + fp_cumsum
        precision = np.divide(tp_cumsum, denominator, out=np.zeros_like(tp_cumsum, dtype=float), where=denominator!=0)


        ap = calculate_ap(precision, recall)

        results_by_class[cls] = {
            'precision': precision, # Store as numpy arrays
            'recall': recall,
            'ap': ap,
            'num_gt': num_gt_cls,
            'num_pred': len(all_pred_boxes_cls) # Number of predictions *with valid scores*
        }

    return results_by_class


def plot_pr_curves(results_all_detectors, classes, output_dir):
    """Plots Precision-Recall curves for each class."""
    if not os.path.exists(output_dir):
        try:
            os.makedirs(output_dir)
        except OSError as e:
             print(f"[LOG] Error creating output directory {output_dir} for plots: {e}")
             return # Cannot save plots

    detector_names = list(results_all_detectors.keys())

    for cls in classes:
        plt.figure(figsize=(10, 7))
        any_results_for_class = False # Track if any detector had results for this class

        for detector_name, results_by_class in results_all_detectors.items():
            if cls in results_by_class and results_by_class[cls]['num_pred'] > 0 : # Check if there were predictions
                res = results_by_class[cls]
                precision = res['precision']
                recall = res['recall']
                ap = res['ap']

                # Ensure plotting works even if precision/recall are empty arrays
                if recall.size > 0 and precision.size > 0:
                    # Prepend a point for plotting nicely from recall=0
                    plot_recall = np.concatenate(([0.], recall))
                    # Use precision[0] if available, else 0.
                    plot_precision = np.concatenate(([precision[0] if precision.size > 0 else 0.], precision))
                    plt.plot(plot_recall, plot_precision, marker='.', markersize=4, linestyle='-', label=f'{detector_name} (AP={ap:.3f})')
                    any_results_for_class = True
                else: # Handle case where num_pred > 0 but P/R arrays somehow ended up empty
                     plt.plot([0], [0], marker='s', markersize=5, linestyle='', label=f'{detector_name} (AP={ap:.3f}, No P/R data?)')
                     any_results_for_class = True # Still mark as having results


            elif cls in results_by_class: # Class exists in evaluation, but no predictions were made for it
                 num_gt = results_by_class[cls]['num_gt']
                 if num_gt > 0:
                      # Plot a marker indicating no predictions were made for this GT class
                      plt.plot([0], [0], marker='x', markersize=6, linestyle='', label=f'{detector_name} (No Pred, GT={num_gt})')
                 # else: # No GT and no predictions for this class, don't plot anything specific
                 #     pass
            # else: # Class not even in results dict for this detector (e.g., error during eval?)
                 # Could happen if detector had no files or all files failed parsing for this class
                 # Might indicate an issue, but avoid cluttering plot unless needed.
                 pass


        if any_results_for_class:
            plt.xlabel('Recall')
            plt.ylabel('Precision')
            plt.title(f'Precision-Recall Curve for Class: {cls}')
            plt.legend(loc='lower left')
            plt.grid(True)
            plt.xlim([-0.05, 1.05])
            plt.ylim([-0.05, 1.05])
            plot_path = os.path.join(output_dir, f'pr_curve_{cls}.png')
            try:
                plt.savefig(plot_path)
                print(f"[LOG]  Generated PR curve: {plot_path}")
            except Exception as e:
                print(f"[LOG]  Error saving PR curve plot for class '{cls}': {e}")
            finally:
                 plt.close() # Close the figure regardless of save success
        else:
            # Check if there was any GT data for this class across all detectors
            # Use .get() chain safely
            num_gt_total = sum(results_by_class.get(cls, {}).get('num_gt', 0) for results_by_class in results_all_detectors.values())
            if num_gt_total > 0:
                 print(f"  Skipping PR plot for class '{cls}': No predictions found across detectors (GT={num_gt_total}).")
            else:
                 print(f"  Skipping PR plot for class '{cls}': No ground truth found.")
            plt.close() # Close the empty figure

def main():
    parser = argparse.ArgumentParser(description='Evaluate N 3D Object Detectors using KITTI format labels.')
    parser.add_argument('gt_dir', type=str, help='Directory containing ground truth label files (KITTI format).')
    parser.add_argument('pred_dirs', type=str, nargs='+', help='List of directories containing prediction files (KITTI format).')
    parser.add_argument('output_dir', type=str, help='Directory to save evaluation results.')
    parser.add_argument('--detector_names', type=str, nargs='*', help='Optional names for the detectors (corresponding to pred_dirs). If not provided, uses directory names.')
    parser.add_argument('--iou_threshold', type=float, default=0.5, help='3D IoU threshold for matching (default: 0.5). KITTI examples: 0.5 (Car Easy/Mod), 0.7 (Car Hard), 0.5 (Ped/Cyc Easy/Mod), 0.5 (Ped/Cyc Hard).')
    parser.add_argument('--classes', type=str, nargs='*', default=['Car', 'Pedestrian', 'Cyclist'], help='List of classes to evaluate (default: KITTI common classes). Case sensitive.')
    parser.add_argument('--file_extension', type=str, default='.txt', help='Extension of the label files (default: .txt).')

    args = parser.parse_args()

    ######  Argument Validation and Setup ######
    if not os.path.isdir(args.gt_dir):
      print(f"[LOG] Error: Ground truth directory not found: {args.gt_dir}")
      return
    # Check prediction directories *before* loading GT
    valid_pred_dirs = []
    original_detector_names = args.detector_names if args.detector_names else [os.path.basename(d.rstrip('/\\')) for d in args.pred_dirs]
    final_detector_names = []
    pred_dir_map = {} # Map name back to directory path

    for i, d in enumerate(args.pred_dirs):
       detector_name = original_detector_names[i]
       if not os.path.isdir(d):
          print(f"[LOG] Prediction directory not found: {d}. Skipping detector '{detector_name}'.")
       else:
          valid_pred_dirs.append(d)
          final_detector_names.append(detector_name)
          pred_dir_map[detector_name] = d

    if not valid_pred_dirs:
        print("[LOG] Error: No valid prediction directories provided.")
        return

    # Ensure output directory exists
    try:
        if not os.path.exists(args.output_dir):
            os.makedirs(args.output_dir)
            print(f"[LOG] Created output directory: {args.output_dir}")
    except OSError as e:
        print(f"[LOG] Error creating output directory {args.output_dir}: {e}")
        return

    print("[LOG] Configuration")
    print(f"Ground Truth Dir: {args.gt_dir}")
    print(f"Prediction Dirs: {valid_pred_dirs}") # Show only valid dirs being used
    print(f"Detector Names: {final_detector_names}") # Show corresponding names
    print(f"Output Dir: {args.output_dir}")
    print(f"IoU Threshold: {args.iou_threshold}")
    print(f"Classes: {args.classes}")
    print(f"File Extension: {args.file_extension}")
    print("=====================\n")

    ######  Load Data ######
    print("[LOG] Loading data...")
    try:
        gt_files = sorted([f for f in os.listdir(args.gt_dir) if f.endswith(args.file_extension)])
        if not gt_files:
            print(f"[LOG] No ground truth files found in {args.gt_dir} with extension {args.file_extension}. Cannot evaluate.")
            return

        print(f"Found {len(gt_files)} ground truth files.")
        gt_boxes_all_samples = {} # {sample_id: {class: [boxes]}}
        # Load GT data first
        print("Loading Ground Truth...")
        for filename in gt_files:
            sample_id = os.path.splitext(filename)[0]
            gt_file_path = os.path.join(args.gt_dir, filename)
            gt_boxes_all_samples[sample_id] = load_labels(gt_file_path, is_gt=True)
        print("Ground Truth loaded.")

        # Load prediction data for valid detectors
        pred_boxes_all_detectors = {} # {detector_name: {sample_id: {class: [boxes]}}}
        print("Loading Predictions...")
        for detector_name in final_detector_names:
            pred_dir = pred_dir_map[detector_name] # Get path from map
            print(f"  Loading from {detector_name} ({pred_dir})...")
            pred_boxes_all_detectors[detector_name] = {}
            files_found_count = 0
            for filename in gt_files: # Iterate based on GT files for consistency
                sample_id = os.path.splitext(filename)[0]
                pred_file_path = os.path.join(pred_dir, filename)
                # load_labels handles non-existent files, returns empty dict
                pred_boxes_all_detectors[detector_name][sample_id] = load_labels(pred_file_path, is_gt=False)
                if os.path.exists(pred_file_path):
                    files_found_count += 1
            print(f"    Found {files_found_count} matching prediction files for {detector_name}.")
        print("Predictions loaded.")

        print(f"\n[LOG] Loaded data for {len(gt_files)} samples and {len(final_detector_names)} detector(s).\n")

    except Exception as e:
        print(f"[LOG] Error during data loading: {e}")
        traceback.print_exc()
        return

    ######  Evaluation ######
    print("[LOG] Evaluating detectors...")
    results_all_detectors = {} # {detector_name: {class: {'ap':, 'p':, 'r':, 'gt':, 'pred':}}}
    try:
        for detector_name in final_detector_names: # Use the filtered list of names
            print(f"  Evaluating: {detector_name}")
            # Check if prediction data was actually loaded for this detector
            if detector_name not in pred_boxes_all_detectors:
                 print(f" [LOG]   Skipping {detector_name} - No prediction data  loaded (check LOG)")
                 results_all_detectors[detector_name] = {} # Store empty results
                 continue

            pred_boxes_all_samples = pred_boxes_all_detectors[detector_name]

            if not gt_boxes_all_samples: # Redundant check, but safe
                 print(f"   Skipping {detector_name} - No ground truth data loaded.")
                 results_all_detectors[detector_name] = {}
                 continue

            # Perform evaluation
            results_by_class = evaluate_detector(
                gt_boxes_all_samples,
                pred_boxes_all_samples,
                args.classes, # Pass the user-specified classes
                args.iou_threshold
            )
            results_all_detectors[detector_name] = results_by_class
        print("[LOG] Evaluation complete.\n")
    except Exception as e:
        print(f"[LOG] Error during evaluation: {e}")
        traceback.print_exc()
        return # Stop execution on evaluation error


    ######  Report Results & Save ######
    print("[LOG] Results")
    results_file_path = os.path.join(args.output_dir, 'evaluation_metrics.txt')

    try:
        with open(results_file_path, 'w') as f:
            f.write(f"Evaluation Results (IoU Threshold: {args.iou_threshold})\n")
            f.write(f"Evaluated Classes: {', '.join(args.classes)}\n")
            f.write("="*60 + "\n")

            overall_mAPs = {} # Store mAP for each detector {detector_name: mAP_value}

            # Process results for each detector that was evaluated
            for detector_name in final_detector_names:
                # Check if results exist for this detector (might be skipped if loading failed)
                if detector_name not in results_all_detectors:
                    print(f"\nDetector: {detector_name} (SKIPPED - No results)")
                    f.write(f"\nDetector: {detector_name} (SKIPPED - No results)\n")
                    continue # Skip to the next detector

                results_by_class = results_all_detectors[detector_name]
                print(f"\nDetector: {detector_name}")
                f.write(f"\nDetector: {detector_name}\n")
                f.write(f"{'Class':<15} | {'AP':<10} | {'Num GT':<10} | {'Num Pred':<10}\n")
                f.write("-" * 55 + "\n")

                detector_aps = [] # AP values for classes with GT > 0 for this detector
                evaluated_classes_with_gt = [] # Class names with GT > 0 for this detector

                # Iterate through the classes specified by the user for consistent reporting
                for cls in args.classes:
                    # Check if GT data exists for this class across all samples
                    num_gt_total_for_class = sum(len(gt_boxes.get(cls, [])) for gt_boxes in gt_boxes_all_samples.values())

                    if cls in results_by_class:
                        # Results exist for this class and detector
                        res = results_by_class[cls]
                        ap = res['ap']
                        num_gt = res['num_gt'] # Should match num_gt_total_for_class if evaluated correctly
                        num_pred = res['num_pred'] # Number of valid predictions for this class
                        print(f"{cls:<15} | {ap:<10.4f} | {num_gt:<10} | {num_pred:<10}")
                        f.write(f"{cls:<15} | {ap:<10.4f} | {num_gt:<10} | {num_pred:<10}\n")
                        # Include in mAP calculation only if there were GT boxes for this class
                        if num_gt > 0:
                           detector_aps.append(ap)
                           evaluated_classes_with_gt.append(cls)
                        # If num_gt is 0 for a class, its AP is 0 or NaN, not included in mAP.
                    else:
                        # No results entry for this class, implies 0 valid predictions processed.
                        # Report AP as 0.0000 if GT existed, otherwise N/A.
                        print(f"{cls:<15} | {'0.0000' if num_gt_total_for_class > 0 else 'N/A':<10} | {num_gt_total_for_class:<10} | {'0':<10}")
                        f.write(f"{cls:<15} | {'0.0000' if num_gt_total_for_class > 0 else 'N/A':<10} | {num_gt_total_for_class:<10} | {'0':<10}\n")
                        # If GT existed, this class contributes 0 to the mAP average.
                        if num_gt_total_for_class > 0:
                            detector_aps.append(0.0)
                            evaluated_classes_with_gt.append(cls)

                # Calculate and report mAP for this detector based on evaluated classes with GT
                if detector_aps: # Check if list is not empty
                    mAP = np.mean(detector_aps)
                    overall_mAPs[detector_name] = mAP
                    # Report which classes contributed to the mAP
                    mAP_info = f"(Classes w/ GT: {', '.join(evaluated_classes_with_gt)})" if evaluated_classes_with_gt else ""
                    print("-" * 55)
                    print(f"{'mAP':<15} | {mAP:<10.4f} {mAP_info}")
                    f.write("-" * 55 + "\n")
                    f.write(f"{'mAP':<15} | {mAP:<10.4f} {mAP_info}\n")
                else:
                    overall_mAPs[detector_name] = np.nan # Indicate mAP couldn't be computed
                    print("-" * 55)
                    print(f"{'mAP':<15} | {'N/A (No GT in evaluated classes)':<10}")
                    f.write("-" * 55 + "\n")
                    f.write(f"{'mAP':<15} | {'N/A (No GT in evaluated classes)':<10}\n")


            ######  Overall Comparison Table ######
            # Check if there are results to compare
            if len(final_detector_names) > 0 and results_all_detectors:
                 print("\n[LOG] Overall Class AP Comparison")
                 f.write("\n" + "="*60 + "\n")
                 f.write("Overall Class AP Comparison\n")
                 f.write("="*60 + "\n")
                 # Truncate long detector names for table header
                 header_names = [name[:12] for name in final_detector_names] # Limit name length
                 header = f"{'Class':<15}" + "".join([f" | {name:<12}" for name in header_names])
                 print(header)
                 f.write(header + "\n")
                 print("-" * len(header))
                 f.write("-" * len(header) + "\n")

                 # Iterate through user-specified classes for the table rows
                 for cls in args.classes:
                     aps_str = ""
                     num_gt_total_for_class = sum(len(gt_boxes.get(cls, [])) for gt_boxes in gt_boxes_all_samples.values())

                     # Get AP for each detector for this class
                     for detector_name in final_detector_names:
                         # Retrieve AP using .get chains safely
                         ap_val = results_all_detectors.get(detector_name, {}).get(cls, {}).get('ap', np.nan)

                         # Decide display value based on AP and total GT count
                         if np.isnan(ap_val):
                             # Show 0.0000 if GT existed, else N/A
                             display_val = f"{0.0:<12.4f}" if num_gt_total_for_class > 0 else f"{'N/A':<12}"
                         else:
                             # Format valid AP value
                             display_val = f"{ap_val:<12.4f}"
                         aps_str += f" | {display_val}"

                     line = f"{cls:<15}" + aps_str
                     print(line)
                     f.write(line + "\n")

                 ######  Overall mAP Comparison ######
                 print("-" * len(header))
                 f.write("-" * len(header) + "\n")
                 map_str = ""
                 for detector_name in final_detector_names:
                     map_val = overall_mAPs.get(detector_name, np.nan) # Get calculated mAP
                     map_str += f" | {map_val:<12.4f}" if not np.isnan(map_val) else f" | {'N/A':<12}"
                 map_line = f"{'mAP':<15}" + map_str
                 print(map_line)
                 f.write(map_line + "\n")
            else:
                 print("\n[LOG] Skipping overall comparison table - no results available.")
                 f.write("\n[LOG] Skipping overall comparison table - no results available.\n")


        print(f"\nMetrics saved to: {results_file_path}")

    except Exception as e:
        print(f"[LOG] Error during results reporting/saving: {e}")
        traceback.print_exc()
        # Continue to plotting if possible
        # return # Or stop here

    ######  Plotting ######
    print("\n[LOG] Generating PR curves...")
    try:
        # Check if results_all_detectors has data before plotting
        if results_all_detectors and any(results_all_detectors.values()):
             # Pass only valid detector results to plotting function
             # Filter out detectors that were skipped or had errors resulting in no results dict
             valid_results = {k: v for k, v in results_all_detectors.items() if isinstance(v, dict) and k in final_detector_names}
             if valid_results:
                  plot_pr_curves(valid_results, args.classes, args.output_dir)
                  print(f"[LOG] PR curve plots potentially saved in: {args.output_dir}")
             else:
                  print("[LOG] Skipping plotting - no valid evaluation results were generated.")
        else:
             print("[LOG] Skipping plotting - no evaluation results generated.")

    except Exception as e:
        print(f"Error during plotting: {e}")
        traceback.print_exc()

    print("\n[LOG] Evaluation finished.")


if __name__ == '__main__':
    main()