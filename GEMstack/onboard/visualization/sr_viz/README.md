# SR Visualization Dashboard

## How to Use

### 1. Start Backend Server

Navigate to the backend directory:

```
cd ./log_dashboard
```

Install Python dependencies:

```
pip install -r requirements.txt
```

Start the backend server:

```
python app.py
```

The backend server will be running at: [http://localhost:5000](http://localhost:5000)

---

### 2. Start Frontend (Development or Production)

#### Prerequisites

- Node.js (v18.17+) and npm. If you already have a different version of Node.js installed, you can manage multiple versions using [nvm](https://github.com/nvm-sh/nvm) (for Unix-based systems) or [nvm-windows](https://github.com/coreybutler/nvm-windows) (for Windows).

Navigate to the frontend directory:

```
cd ./GEMstack/onboard/visualization/sr_viz/threeD
```

Install frontend dependencies:

```
npm install
```

Run development server:

```
npm run dev
```

Or run production build:

```
npm run build && npm run start
```

The frontend visualizer will be available at: [http://localhost:3000](http://localhost:3000)

---

### 3. Open the Dashboard

Visit: [http://localhost:5000](http://localhost:5000)

If a valid `behavior.json` file is loaded, the **"3D Visualization"** button will appear at the top-right corner.

Clicking it redirects you to the 3D visualizer at: [http://localhost:3000](http://localhost:3000)

---

## Format of Objects in `behavior.json`

### Other Vehicles

```json
{
  "other_vehicles": {
    "<vehicle_id>": {
      "type": "OtherVehicleState",
      "data": {
        "pose": {
          "frame": <int>,
          "t": <float>,
          "x": <float>,
          "y": <float>,
          "z": <float>,
          "yaw": <float>,
          "pitch": <float>,
          "roll": <float>
        },
        "dimensions": [<float>, <float>, <float>],
        "outline": <null | object>,
        "type": <int>,
        "activity": <int>,
        "velocity": [<float>, <float>, <float>],
        "yaw_rate": <float>
      }
    }
  },
  "time": <float>
}
```

### Traffic Lights

```json
{
  "traffic_lights": {
    "<traffic_light_id>": {
      "type": "TrafficLightState",
      "data": {
        "pose": {
          "frame": <int>,
          "t": <float>,
          "x": <float>,
          "y": <float>,
          "z": <float>,
          "yaw": <float>,
          "pitch": <float>,
          "roll": <float>
        },
        "state": <string>
      }
    }
  },
  "time": <float>
}
```

# SR Rosbag Viewer
## How to Use
In the bottom-right corner of the 3D Visualizer, there's a redirect button. Clicking it will take you to the Rosbag Viewer. Currently, it supports viewing uncompressed images and point clouds by topic.

To view both images and point clouds simultaneously:
  1. After uploading the rosbag file and confirming that the upload is complete (as indicated by the loading status in the scrubber), click the menu in the top-right corner.

  2. Choose to split the current panel vertically or horizontally.

  3. In one of the panels, select `Switch to PointCloud Panel`.