# YOLOv8 Traffic Light and Sign Model Training and Validation Script

## Overview

The "traffic_sign_and_light_model_train.py" script was used to train the yolov8 model "sign_model" present at the path "GEMstack/knowledge/detection/sign_model.pt" using the datatset [road signs Computer Vision Project](https://universe.roboflow.com/roboflow-100/road-signs-6ih4y). The dataset includes data.yaml file that specifies details regarding classes and paths to the training, validation, and test sets. Running the validation function on the trained model yeilds model performance metrics that are automatically saved to the device.

## Requirements

- Python 3.x
- ultralytics

## Usage

1. Install the Ultralytics library:

   ```bash
   pip install ultralytics
2. Download the dataset found at https://universe.roboflow.com/roboflow-100/road-signs-6ih4y in the YOLOv8 format and place it in the same directory as the script

3. Run "traffic_sign_and_light_model_train.py" script 
