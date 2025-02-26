# Test Safety and Comfort Metrics Documentation

## Files/Scripts
- `testing/test_comfort_metrics.py`

## Purpose

This script analyzes log files and reports vehicle comfort and pedestrian safety with plots. It extracts:

- **Vehicle Data:** Time, acceleration, heading rate from `behavior.json`.
- **Pedestrian Data:** Time and pedestrian distance to car from `behavior.json`.
- **Pure Pursuit Tracker Data (Optional):** Vehicle time, cross-track error, and position (actual vs. desired) from `PurePursuitTrajectoryTracker_debug.csv`.

The script will include 5 plots:
- Vehicle jerk vs. time.
- Vehicle heading acceleration vs. time.
- Vehicle cross-track error vs. time.
- Vehicle actual vs. desired position.
- Pedestrian distance vs. time.

The script also prints key metrics:
- RMS Jerk
- RMS Heading acceleration
- RMS Cross track error
- RMS Position error
- Minimum pedestrian distance to car

## Usage

1. **Check log directory:**
   - Ensure your log directory contains `behavior.json` (required).
   - Optionally include `PurePursuitTrajectoryTracker_debug.csv` (if missing, some plots are skipped).

2. **Run the script:**

   ```bash
   python test_comfort_metrics.py <log_directory>
   ```
   Replace `<log_directory>` with the path to the folder containing the log files.