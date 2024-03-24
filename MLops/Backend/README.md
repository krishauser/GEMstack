# MLops Backend

Run `pip install -r requirements.txt`to install the required dependencies.

## Introduction to API endpoints in Server

### /home
* Description: This endpoint serves the home page of the application.
* Method: GET
* Returns: Renders the HTML template for the home page.
### /models/
* Description: This endpoint retrieves a list of all models stored in the database.
* Method: GET
* Returns: JSON array containing details of all models.
### /datasets/
* Description: This endpoint retrieves a list of all datasets stored in the database.
* Method: GET
* Returns: JSON array containing details of all datasets.
### /models/<id>
* Description: This endpoint retrieves details of a specific model based on its ID.
* Method: GET
* Parameters:
  #### `<id>`: The unique identifier of the model.
* Returns: JSON object containing details of the requested model. Returns a 404 error if the model is not found.
### /models/retrieval/<id>
* Description: This endpoint allows downloading the model file associated with a specific model.
* Method: GET
* Parameters:
  #### `<id>`: The unique identifier of the model.
* Returns: Sends the model file as an attachment for download. Returns a 404 error if the model is not found or if the path to the file is missing.
### /datasets/<id>
* Description: This endpoint retrieves details of a specific dataset based on its ID.
* Method: GET
* Parameters:
  #### `<id>`: The unique identifier of the dataset.
* Returns: JSON object containing details of the requested dataset. Returns a 404 error if the dataset is not found.
### /datasets/retrieval/<id>
* Description: This endpoint allows downloading the dataset file associated with a specific dataset.
* Method: GET
* Parameters:
  #### `<id>`: The unique identifier of the dataset.
* Returns: Sends the dataset file as an attachment for download. Returns a 404 error if the dataset is not found or if the path to the file is missing.

# Running the Server
Execute `python app.py` to run the Flask server.
The server will run in debug mode, enabling real-time debugging information in the console.