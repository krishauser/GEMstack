# MLops API

Run `pip install -r requirements.txt`to install the required dependencies.

For now you can use the api from the following code：

```
from api import APIClient

client = APIClient()
```

And then, you can use any of the functions from the client. For example, to upload a dataset, you can run 
`client.dataset_upload('your_complete_path_to_file')`

## Introduction to API Class

### list_models_info
- **Description:** Print a list of all models information stored in the database.

### model_info(id)
- **Description:** Print details of a specific model based on its ID.
- **Parameters:** 
  - `<id>`: The unique identifier of the model.

### model_update_description(id, new_description)
- **Description:** Updating description of a specific model based on its ID.
- **Parameters:** 
  - `<id>`: The unique identifier of the model.
  - `new_description`: New description of the model.

### model_download(id)
- **Description:** Downloading the model file of a specific model based on ID.
- **Parameters:** 
  - `<id>`: The unique identifier of the model.
- **Returns:** Download file in working directory

### model_upload(path)
- **Description:** Uploading a new model to the server. If model name existed, it will update an existing model. 
- **Parameters:** 
  - `path`: The complete path to the file.

### list_datasets_info
- **Description:** Print a list of all dataset information stored in the database.

### dataset_info(id)
- **Description:** Pirnt details of a specific dataset based on its ID.
- **Parameters:** 
  - `<id>`: The unique identifier of the dataset.

### dataset_update_description(id, new_description)
- **Description:** Updating details of a specific dataset based on its ID.
- **Parameters:** 
  - `<id>`: The unique identifier of the dataset.
  - `new_description`: New description of the dataset.

### dataset_download(id)
- **Description:** Downloading the dataset file of a specific dataset.
- **Parameters:** 
  - `<id>`: The unique identifier of the dataset.
- **Returns:** Download file in working directory

### dataset_upload(path)
- **Description:** Uploading a new dataset to the server. If the dataset name existed, it will update an existing dataset.
- **Parameters:** 
  - `path`: The complete path to the file.
