# MLOps API Gemstack

This is the `mlops_api_gemstack` package, version 0.3.3. It is an API package designed to connect to a MLOps server relating to the GEMstack project.

[PYPI PAGE](https://pypi.org/project/mlops-api-gemstack/)

## Installation

You can install this package using pip:

```bash
pip install mlops_api_gemstack
```

If you have installed before, use force install to get the latest version.

```bash
pip install mlops_api_gemstack --force-reinstall
```

## Introduction to Package CLI Command

```bash
mlops [OPTIONS] COMMAND [ARGS]...
```

### list-models-info
```bash
mlops list-models-info
```
- **Description:** Print a list of all models information stored in the database.

### model-info MODEL_ID
```bash
mlops model-info MODEL_ID
```
- **Description:** Print details of a specific model based on its ID.
- **Parameters:** 
  - `MODEL_ID`: The unique identifier of the model.

### model-update MODEL_ID NEW_DESCRIPTION
```bash
mlops model-update MODEL_ID NEW_DESCRIPTION
```
- **Description:** Updating description of a specific model based on its ID.
- **Parameters:** 
  - `MODEL_ID`: The unique identifier of the model.
  - `NEW_DESCRIPTION`: New description of the model.

### model-download MODEL_ID
```bash
mlops model-download MODEL_ID
```
- **Description:** Download the model file of a specific model based on ID.
- **Parameters:** 
  - `MODEL_ID`: The unique identifier of the model.

### model-upload FILE_PATH
```bash
mlops model-upload FILE_PATH
```
- **Description:** Uploading a new model to the server. If model name existed, it will update an existing model. 
- **Parameters:** 
  - `FILE_PATH`: The complete path to the file.

### list-datasets-info
```bash
mlops list-datasets-info
```
- **Description:** Print a list of all dataset information stored in the database.

### dataset-info DATASET_ID
```bash
mlops dataset-info DATASET_ID
```
- **Description:** Pirnt details of a specific dataset based on its ID.
- **Parameters:** 
  - `DATASET_ID`: The unique identifier of the dataset.

### dataset-update DATASET_ID NEW_DESCRIPTION NEW_SOURCE
```bash
mlops dataset-update DATASET_ID NEW_DESCRIPTION NEW_SOURCE
```
- **Description:** Updating details of a specific dataset based on its ID.
- **Parameters:** 
  - `DATASET_ID`: The unique identifier of the dataset.
  - `NEW_DESCRIPTION`: New description of the dataset.
  - `NEW_SOURCE`: New source description of the dataset.

### dataset-download DATASET_ID
```bash
mlops mlops dataset-download DATASET_ID
```
- **Description:** Download the dataset file of a specific dataset.
- **Parameters:** 
  - `DATASET_ID`: The unique identifier of the dataset.

### dataset-upload FILE_PATH SOURCE
```bash
mlops dataset-upload FILE_PATH SOURCE
```
- **Description:** Uploading a new dataset to the server. If the dataset name existed, it will update an existing dataset.
- **Parameters:** 
  - `FILE_PATH`: The complete path to the file.
  - `SOURCE`: The source description of the dataset.

### dataset-uploadbag FILE_PATH SOURCE
```bash
mlops dataset-uploadbag FILE_PATH SOURCE
```
- **Description:** Uploading a new bag dataset to the server. If the dataset name existed, it will update an existing dataset.
- **Parameters:** 
  - `FILE_PATH`: The complete path to the file.
  - `SOURCE`: The source description of the bag dataset.

### record-rosbag TOPICS_FILE ROSBAG_FILE_NAME SOURCE --delete_rosbag
```bash
mlops record-rosbag TOPICS_FILE ROSBAG_FILE_NAME SOURCE --delete_rosbag
```
- **Description:** Record and uploading a new rosbag dataset to the server.
- **Parameters:** 
  - `TOPICS_FILE`: A text file containing the topics that you want rosbag to record. One topic per line.
  - `ROSBAG_FILE_NAME`: The name you want the final rosbag file to have.
  - `SOURCE`: The source description of the bag dataset.
  - `--delete_rosbag`: A optional flag if you want to delete the rosbag after uploading.
