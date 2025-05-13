#!/bin/bash
if [ $# -lt 1 ]; then
    echo "Usage: $0 <scene_file>"
    exit 1
fi
SCENE_PATH=$1
echo "Using scene: $SCENE_PATH"
# Inject scene path into the template and create a temporary YAML config
env SCENE_PATH="$SCENE_PATH" envsubst < ./launch/parking_testing.yaml > ./launch/parking_generated.yaml
# Run the simulation
python3 main.py --variant=reeds_shepp ./launch/parking_generated.yaml
python3 main.py --variant=approx_reeds_shepp ./launch/parking_generated.yaml
python3 main.py --variant=euclidian ./launch/parking_generated.yaml
## To run this file with selected scne, ./run_sim.sh ../scenes/parking/demo_2.yaml