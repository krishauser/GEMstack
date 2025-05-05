from flask import Flask, render_template, request, jsonify, send_file
import os
import yaml
import datetime
import functools
import time
from cachelib import SimpleCache
import json
import numpy as np
import platform
import matplotlib
from flask import Response, stream_with_context
from flask_cors import CORS

# Use the 'Agg' backend which is thread-safe and doesn't require a GUI
matplotlib.use("Agg")

import matplotlib.pyplot as plt

app = Flask(__name__)
CORS(app)
CORS(app, origins=["http://localhost:3000"])

# Configure cache
cache = SimpleCache(threshold=500, default_timeout=300)  # 5 minutes cache timeout

LOG_DIR = "../logs"


def generate_behavior_plots(log_folder, behavior_file, target_frame=3):
    """
    Generate a comprehensive visualization plot for vehicle, agents, and trajectory

    Args:
        log_folder (str): Name of the log folder
        behavior_file (str): Path to the behavior.json file
        target_frame (int, optional): Specific frame to filter data. Defaults to None.

    Returns:
        dict: Paths to generated plot files
    """
    target_frame = 3
    # Define output plot directory
    plot_dir = os.path.join("./plots", log_folder, "viz")
    os.makedirs(plot_dir, exist_ok=True)

    # Create cache file to track previous plot generation
    cache_file = os.path.join(plot_dir, f"plot_cache_frame_{target_frame}.json")

    # Check if plots have been previously generated
    if os.path.exists(cache_file):
        with open(cache_file, "r") as f:
            return json.load(f)

    # Initialize data collections
    vehicle_data = []
    agent_data = {}
    trajectory_data = []

    # Parse behavior file
    with open(behavior_file, "r") as f:
        for line in f:
            try:
                entry = json.loads(line.strip())

                # Vehicle state
                if "vehicle" in entry and "data" in entry["vehicle"]:
                    vehicle_state = entry["vehicle"]["data"]["pose"]
                    # Check frame filter if specified
                    if (
                        target_frame is None
                        or vehicle_state.get("frame") == target_frame
                    ):
                        vehicle_data.append(
                            {
                                "time": entry["time"],
                                "x": vehicle_state.get("x", 0),
                                "y": vehicle_state.get("y", 0),
                                "frame": vehicle_state.get("frame"),
                            }
                        )

                # Agent states
                if "agents" in entry:
                    for agent_name, agent_info in entry["agents"].items():
                        agent_state = agent_info["data"]["pose"]
                        # Check frame filter if specified
                        if (
                            target_frame is None
                            or agent_state.get("frame") == target_frame
                        ):
                            if agent_name not in agent_data:
                                agent_data[agent_name] = []

                            agent_data[agent_name].append(
                                {
                                    "time": entry["time"],
                                    "x": agent_state.get("x", 0),
                                    "y": agent_state.get("y", 0),
                                    "frame": agent_state.get("frame"),
                                }
                            )

                # Trajectory
                if "trajectory" in entry and "data" in entry["trajectory"]:
                    traj_points = entry["trajectory"]["data"]["points"]
                    traj_times = entry["trajectory"]["data"]["times"]
                    traj_frames = entry["trajectory"]["data"].get(
                        "frames", [None] * len(traj_points)
                    )

                    trajectory_data = [
                        {"x": point[0], "y": point[1], "time": time, "frame": frame}
                        for point, time, frame in zip(
                            traj_points, traj_times, traj_frames
                        )
                        if target_frame is None or frame == target_frame
                    ]

            except json.JSONDecodeError:
                continue

    # Comprehensive Plot
    plt.figure(figsize=(12, 8))

    # Plot vehicle trajectory
    if vehicle_data:
        vehicle_xs = [v["x"] for v in vehicle_data]
        vehicle_ys = [v["y"] for v in vehicle_data]
        plt.plot(
            vehicle_xs,
            vehicle_ys,
            label="Vehicle Path",
            color="red",
            linewidth=3,
            marker="o",
            markersize=5,
        )

    # Plot agent trajectories
    for agent_name, positions in agent_data.items():
        agent_xs = [a["x"] for a in positions]
        agent_ys = [a["y"] for a in positions]
        plt.plot(agent_xs, agent_ys, label=agent_name, marker="x")

    # Plot planned trajectory
    if trajectory_data:
        traj_xs = [t["x"] for t in trajectory_data]
        traj_ys = [t["y"] for t in trajectory_data]
        plt.plot(
            traj_xs,
            traj_ys,
            label="Planned Trajectory",
            color="green",
            linestyle="--",
            linewidth=2,
        )

    plt.title(f"Comprehensive Movement Visualization (Frame {target_frame})")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.7)

    # Save the plot
    plot_path = os.path.join(
        plot_dir, f"comprehensive_trajectory_frame_{target_frame}.png"
    )
    plt.savefig(plot_path, dpi=300, bbox_inches="tight")
    plt.close()

    # Cache plot file paths
    plot_files = {"comprehensive": plot_path}
    with open(cache_file, "w") as f:
        json.dump(plot_files, f)

    return plot_files


@app.route("/view_log/<path:log_folder>/render")
def render_vis_html(log_folder):
    print(f"Rendering visualization HTML for log folder: {log_folder}")
    return render_template("render.html", log_folder=log_folder)


@app.route("/view_log/<path:log_folder>/get_render")
def render_behavior_visualization(log_folder):
    """
    Render behavior visualization for a specific log folder

    Returns:
        JSON response with plot file paths
    """
    log_folder_path = os.path.join(LOG_DIR, log_folder)

    # Find behavior.json file
    behavior_files = [f for f in os.listdir(log_folder_path) if f == "behavior.json"]

    if not behavior_files:
        return jsonify({"error": "No behavior.json file found"}), 404

    behavior_file_path = os.path.join(log_folder_path, behavior_files[0])

    # Get frame from query parameter, default to None if not specified
    target_frame = request.args.get("frame", type=int)

    try:
        # Generate behavior plots
        plot_files = generate_behavior_plots(
            log_folder, behavior_file_path, target_frame
        )
        print(plot_files)
        # return jsonify(plot_files)
        # return render_template("render.html", image_path=plot_files)
        return send_file(plot_files["comprehensive"], mimetype="image/png")

    except Exception as e:
        return jsonify({"error": str(e)}), 500


# Add a route to serve plot files
@app.route("/plots/<path:log_folder>/viz/<path:filename>")
def serve_plot(log_folder, filename):
    """
    Serve plot files from the plots directory
    """
    plot_dir = os.path.join("./plots", log_folder, "viz")
    return send_file(os.path.join(plot_dir, filename))


def get_cache_key(prefix, *args):
    """Generate a cache key with a prefix and arguments"""
    return f"{prefix}_{hash(str(args))}"


def parse_behavior_data(file_path):
    """
    Parse behavior.json file and extract vehicle and agent positions

    Returns:
    {
        'vehicle': [{'time': float, 'x': float, 'y': float}, ...],
        'agents': {
            'ped1': [{'time': float, 'x': float, 'y': float}, ...],
            'ped2': [...],
            ...
        }
    }
    """
    try:
        with open(file_path, "r") as f:
            # Read file line by line to handle large files
            positions = {"vehicle": [], "agents": {}}

            for line in f:
                try:
                    entry = json.loads(line.strip())

                    # Process Vehicle State
                    if "vehicle" in entry:
                        vehicle_data = entry["vehicle"]["data"]["pose"]
                        positions["vehicle"].append(
                            {
                                "time": entry["time"],
                                "x": vehicle_data.get("x", 0),
                                "y": vehicle_data.get("y", 0),
                            }
                        )

                    # Process Agent States
                    if "agents" in entry:
                        for agent_name, agent_data in entry["agents"].items():
                            if agent_name not in positions["agents"]:
                                positions["agents"][agent_name] = []

                            agent_pose = agent_data["data"]["pose"]
                            positions["agents"][agent_name].append(
                                {
                                    "time": entry["time"],
                                    "x": agent_pose.get("x", 0),
                                    "y": agent_pose.get("y", 0),
                                }
                            )

                except json.JSONDecodeError:
                    # Skip invalid JSON lines
                    continue

            return positions
    except Exception as e:
        print(f"Error parsing behavior data: {e}")
        return None


@functools.lru_cache(maxsize=100)
def load_log_data():
    """Load all log data with caching"""
    cache_key = "all_logs"
    cached_logs = cache.get(cache_key)

    if cached_logs is not None:
        return cached_logs

    start_time = time.time()
    logs = []

    for log_folder in sorted(os.listdir(LOG_DIR), reverse=True):
        log_path = os.path.join(LOG_DIR, log_folder)
        if not os.path.isdir(log_path):
            continue

        meta_path = os.path.join(log_path, "meta.yaml")
        settings_path = os.path.join(log_path, "settings.yaml")

        try:
            with open(meta_path, "r") as meta_file:
                meta_data = yaml.safe_load(meta_file)
            with open(settings_path, "r") as settings_file:
                settings_data = yaml.safe_load(settings_file).get("run", {})
        except Exception as e:
            print(f"Error loading log data: {e}")
            continue

        logs.append(
            {
                "date": log_folder,
                "run_duration": meta_data.get("run_duration", "Unknown"),
                "exit_reason": meta_data.get("exit_reason", "Unknown"),
                "mode": settings_data.get("mode", "Unknown"),
                "launch_command": settings_data.get("log", {}).get(
                    "launch_command", "Unknown"
                ),
                "folder": log_path,
            }
        )

    # Store results in cache
    cache.set(cache_key, logs)
    end_time = time.time()
    print(f"Log data loaded in {end_time - start_time:.2f} seconds")
    print(logs)
    return logs


def filter_logs_by_date(logs, start_date=None, end_date=None):
    """Filter logs by date range"""
    cache_key = get_cache_key("filtered_logs", start_date, end_date)
    cached_result = cache.get(cache_key)

    if cached_result is not None:
        return cached_result

    if start_date:
        start_date = datetime.datetime.strptime(start_date, "%Y-%m-%d")
    if end_date:
        end_date = datetime.datetime.strptime(end_date, "%Y-%m-%d")

    filtered_logs = []
    for log in logs:
        try:
            log_date = datetime.datetime.strptime(log["date"][:10], "%Y-%m-%d")
            if (not start_date or log_date >= start_date) and (
                not end_date or log_date <= end_date
            ):
                filtered_logs.append(log)
        except ValueError:
            # Skip logs with invalid date format
            continue

    # Cache the filtered results
    cache.set(cache_key, filtered_logs)
    return filtered_logs


@functools.lru_cache(maxsize=50)
def get_log_metadata(log_folder_path):
    """Get metadata for a specific log with caching"""
    cache_key = f"metadata_{log_folder_path}"
    cached_metadata = cache.get(cache_key)

    if cached_metadata is not None:
        return cached_metadata

    metadata = {"folder": log_folder_path}

    # Read metadata from files
    meta_path = os.path.join(log_folder_path, "meta.yaml")
    settings_path = os.path.join(log_folder_path, "settings.yaml")

    try:
        if os.path.exists(meta_path):
            with open(meta_path, "r") as meta_file:
                metadata.update(yaml.safe_load(meta_file))
        if os.path.exists(settings_path):
            with open(settings_path, "r") as settings_file:
                log_settings = yaml.safe_load(settings_file).get("run", {})

                metadata.update(
                    {
                        "mode": log_settings.get("mode", "Unknown"),
                        "launch_command": log_settings.get("log", {}).get(
                            "launch_command", "Unknown"
                        ),
                    }
                )
    except Exception as e:
        print(f"Error loading metadata: {e}")

    # Cache the results
    cache.set(cache_key, metadata)
    return metadata


@app.route("/")
def index():
    logs = load_log_data()
    return render_template("index.html", logs=logs)


@app.route("/filter_logs", methods=["POST"])
def filter_logs():
    start_time = time.time()
    logs = load_log_data()
    data = request.json
    start_date = data.get("start_date")
    end_date = data.get("end_date")

    filtered_logs = filter_logs_by_date(logs, start_date, end_date)

    end_time = time.time()
    print(f"Filtered logs in {end_time - start_time:.2f} seconds")
    return jsonify(filtered_logs)


@app.route("/view_log/<path:log_folder>")
def view_log(log_folder):
    log_folder_path = os.path.join(LOG_DIR, log_folder)
    if not os.path.exists(log_folder_path):
        return "Log folder not found!", 404

    # Get metadata with caching
    metadata = get_log_metadata(log_folder_path)

    # Get directory structure
    files = sorted(os.listdir(log_folder_path))

    return render_template(
        "view_log.html", log_folder=log_folder, metadata=metadata, files=files
    )


@app.route("/open_folder/<path:folder>")
def open_folder(folder):
    import os, platform

    full_path = os.path.abspath(os.path.join(LOG_DIR, folder))
    if not full_path.startswith(os.path.abspath(LOG_DIR)):
        return "Invalid path", 400
    if not os.path.isdir(full_path):
        return f"Folder does not exist: {folder}", 404

    try:
        if platform.system() == "Windows":
            os.system(f'explorer "{full_path}"')
        elif platform.system() == "Linux":
            os.system(f'xdg-open "{full_path}"')
        elif platform.system() == "Darwin":
            os.system(f'open "{full_path}"')
        else:
            return "Unsupported platform", 400
    except Exception as e:
        return f"Failed to open folder: {e}", 500

    # if result != 0:
    #     return 'Failed to open folder', 500

    return "Folder opened successfully.", 204


@app.route("/view_file/<path:log_folder>/<path:filename>")
def view_file(log_folder, filename):
    file_path = os.path.join(LOG_DIR, log_folder, filename)
    if not os.path.exists(file_path):
        return jsonify({"error": "File not found!"}), 404

    # Check file size first
    file_size = os.path.getsize(file_path)

    # Define chunk size for pagination (50,000 lines or ~1MB)
    CHUNK_SIZE = 1000

    # Get page number from query parameter (default to 1)
    page = request.args.get("page", 1, type=int)

    try:
        with open(file_path, "r") as f:
            # Skip lines for previous pages
            for _ in range((page - 1) * CHUNK_SIZE):
                f.readline()

            # Read next chunk of lines
            lines = [f.readline() for _ in range(CHUNK_SIZE)]
            # Check if there are more lines
            has_more = bool(f.readline())

            # Prepare response
            return jsonify(
                {
                    "filename": filename,
                    "content": "".join(lines),
                    "total_size": file_size,
                    "page": page,
                    "has_more": has_more,
                }
            )
    except UnicodeDecodeError:
        return jsonify(
            {
                "filename": filename,
                "content": "This file contains binary data and cannot be displayed in the browser.",
                "total_size": file_size,
                "page": page,
                "has_more": False,
            }
        )


@app.route("/raw_logs/<path:log_folder>/<path:filename>")
def stream_raw_log(log_folder, filename):
    file_path = os.path.join(LOG_DIR, log_folder, filename)
    if not os.path.isfile(file_path):
        return "File not found!", 404

    def generate():
        with open(file_path, "r", encoding="utf-8") as f:
            for line in f:
                yield line

    return Response(stream_with_context(generate()), mimetype="text/plain")


@app.route("/parse_behavior/<path:log_folder>/<path:filename>")
def parse_behavior(log_folder, filename):
    """
    Parse behavior.json and return structured position data
    """
    file_path = os.path.join(LOG_DIR, log_folder, filename)

    if not os.path.exists(file_path):
        return jsonify({"error": "File not found!"}), 404

    # Parse behavior data
    behavior_data = parse_behavior_data(file_path)

    if behavior_data is None:
        return jsonify({"error": "Could not parse behavior data"}), 500

    return jsonify(behavior_data)


# Clear cache after certain time period
@app.after_request
def add_header(response):
    # Invalidate cache for certain requests
    if request.path == "/":
        # Clear cache periodically for main page
        current_time = int(time.time())
        last_cleared = cache.get("last_cache_clear") or 0

        if current_time - last_cleared > 300:  # 5 minutes
            # Reset log data cache
            load_log_data.cache_clear()
            cache.set("last_cache_clear", current_time)

    return response


if __name__ == "__main__":
    app.run(debug=True)
