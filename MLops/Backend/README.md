# MLOps Backend

Run `pip install -r requirements.txt` to install the required dependencies.

## server.py  
## Introduction to API endpoints in Server

### /models/
- **Description:** This endpoint retrieves a list of all models information stored in the database.
- **Method:** GET
- **Returns:** JSON array containing details of all models.

### /models/`<id>`
- **Description:** This endpoint retrieves details of a specific model based on its ID.
- **Method:** GET
- **Parameters:** 
  - `<id>`: The unique identifier of the model.
- **Returns:** JSON object containing details of the requested model. Returns a 404 error if the model is not found.

### /models/`<id>`
- **Description:** This endpoint allows updating details of a specific model based on its ID.
- **Method:** PUT
- **Parameters:** 
  - `<id>`: The unique identifier of the model.
  - JSON data containing:
    - `ModelName`: Name of the model.
    - `Description`: Description of the model.
- **Returns:** JSON response indicating the success or failure of the update operation.

### /models/retrieval/`<id>`
- **Description:** This endpoint allows downloading the model file of a specific model.
- **Method:** GET
- **Parameters:** 
  - `<id>`: The unique identifier of the model.
- **Returns:** Sends the model file as an attachment for download. Returns a 404 error if the model is not found or if the path to the file is missing.

### /models/upload
- **Description:** This endpoint allows uploading a new model to the server. If model name existed, it will update an existing model. 
- **Method:** POST
- **Parameters:** 
  - `file`: The model file to upload.
- **Returns:** JSON response indicating the success or failure of the upload operation, and the filename of the model.

### /datasets/
- **Description:** This endpoint retrieves a list of all dataset information stored in the database.
- **Method:** GET
- **Returns:** JSON array containing details of all datasets.

### /datasets/`<id>`
- **Description:** This endpoint retrieves details of a specific dataset based on its ID.
- **Method:** GET
- **Parameters:** 
  - `<id>`: The unique identifier of the dataset.
- **Returns:** JSON object containing details of the requested dataset. Returns a 404 error if the dataset is not found.

### /datasets/`<id>`
- **Description:** This endpoint allows updating details of a specific dataset based on its ID.
- **Method:** PUT
- **Parameters:** 
  - `<id>`: The unique identifier of the dataset.
  - JSON data containing:
    - `DataName`: Name of the dataset.
    - `Path`: Path to the dataset file.
    - `Description`: Description of the dataset.
- **Returns:** JSON response indicating the success or failure of the update operation.

### /datasets/retrieval/`<id>`
- **Description:** This endpoint allows downloading the dataset file of a specific dataset.
- **Method:** GET
- **Parameters:** 
  - `<id>`: The unique identifier of the dataset.
- **Returns:** Sends the dataset file as an attachment for download. Returns a 404 error if the dataset is not found or if the path to the file is missing.

### /datasets/upload
- **Description:** This endpoint allows uploading a new dataset to the server. If the dataset name existed, it will update an existing dataset.
- **Method:** POST
- **Parameters:** 
  - `file`: The dataset file to upload.
- **Returns:** JSON response indicating the success or failure of the upload operation, the id and file name of the uploaded dataset.

### /datasets/uploadBag
- **Description:** This endpoint allows uploading a ROS bag file. The file will be processed to extract and convert data, which is then stored and managed as part of the dataset.
- **Method:** POST
- **Parameters:** 
  - `file`: The ROS bag file to upload.
- **Returns:** JSON response including success message and details(including id and file name) of datasets created from the uploaded bag file. Returns an error if the file is not a `.bag` file or if file processing fails.


## preprocess.py
## ROS Bag Preprocessing Tool

This tool is designed to preprocess input ROS bags and store them on a file server. It automates the conversion of various topic types from ROS bags into different formats suitable for further analysis or storage.

### Supported Conversions

#### OAK Front Camera Data:
- **Raw data:** Convert to PNG images and MP4 video.
- **Compressed images:** Convert to JPG images and MP4 video.

#### Ouster Scan Point Cloud Data:
- **Conversion:** Convert to PCD files and BIN files.

#### Septentrio GNSS Data:
- **Conversion:** Convert to GPX files.

#### Septentrio IMU Data:
- **Conversion:** Convert to JSON files.

#### PacMod Topics:
- **Conversion:** Convert to JSON files.

### All the model files will be stored into GEMstack/MLops/model folder
### All the dataset files will be stored into GEMstack/MLops/dataset folder

# Running the Server
Execute `python server.py` to run the Flask server.
The server will run in debug mode, enabling real-time debugging information in the console.
