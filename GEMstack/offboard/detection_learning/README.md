# YOLOv8 Traffic Light and Sign Model Training and Validation Script

## Overview

The traffic_sign_and_light_model_train.py script was used to train the yolov8 model "sign_model" present at the path "GEMstack/knowledge/detection/sign_model.pt" using the datatset "https://universe.roboflow.com/roboflow-100/road-signs-6ih4y". The dataset includes data.yaml file that specifies details regarding classes and paths to the training, validation, and test sets. Running the validation function on the trained model yeilds model performance metrics that are automatically saved to the device.

## Requirements

- Python 3.x
- ultralytics
