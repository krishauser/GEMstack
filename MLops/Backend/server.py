from flask import Flask, jsonify, send_file, render_template,request
from werkzeug.utils import secure_filename
from pymongo import MongoClient
from bson.objectid import ObjectId
import json
import os
from pathlib import Path

app = Flask(__name__)
app.config['MODEL_UPLOAD_FOLDER'] = '../model'
app.config['DATASET_UPLOAD_FOLDER'] = '../dataset'
app.config['MAX_CONTENT_LENGTH'] = 10 * 1024 * 1024 * 1024


def load_config(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"The config file {file_path} does not exist.")

    with open(file_path) as config_file:
        config_json = json.load(config_file)

        # Validate necessary keys in the configuration
        required_keys = ['MONGO_URI', 'DATABASE_NAME']
        if not all(key in config_json for key in required_keys):
            raise ValueError(f"Config file is missing required keys. Required keys: {required_keys}")

        return config_json


try:
    config = load_config('config.json')
    client = MongoClient(config['MONGO_URI'])
    db = client[config['DATABASE_NAME']]
except FileNotFoundError as e:
    print(f"Error: {e}")
    exit(1)
except ValueError as e:
    print(f"Error: {e}")
    exit(1)


@app.route('/models', methods=['GET'])
def list_all_models():
    models = db.Models.find({}, {'_id': 1, 'ModelName': 1, 'Path': 1, 'Description': 1})
    models_list = [{
        '_id': str(model['_id']),
        'ModelName': model['ModelName'],
        'Path': model['Path'],
        'Description': model.get('Description', '')
    } for model in models]
    return jsonify(models_list)


@app.route('/models/upload', methods=['POST'])
def upload_model():
    if 'file' not in request.files:
        return jsonify({'error': 'No file part'}), 400

    file = request.files['file']
    if file.filename == '':
        return jsonify({'error': 'No selected file'}), 400

    if file:
        filename = secure_filename(file.filename)
        path = str(Path(app.config['MODEL_UPLOAD_FOLDER']) / filename)
        file.save(path)

        # Check if the model with the same ModelName exists
        existing_model = db.Models.find_one({'ModelName': filename})
        if existing_model:
            return jsonify({'message': 'Model updated successfully', 'filename': filename}), 200
        else:
            model = {
                'ModelName': filename,
                'Path': path,
                'Description': ''
            }
            db.Models.insert_one(model)
            return jsonify({'message': 'Model uploaded successfully', 'filename': filename}), 200


@app.route('/models/<id>', methods=['GET'] )
def list_model_info(id):
    model_info = db.Models.find_one({'_id': ObjectId(id)}, {'_id': 1, 'ModelName': 1, 'Path': 1, 'Description': 1})
    if model_info:
        model_info['_id'] = str(model_info['_id'])
        return jsonify(model_info)
    else:
        return jsonify({"error": "Model not found"}), 404


@app.route('/models/<id>', methods=['PUT'])
def update_model(id):
    data = request.json
    update_data = {}
    if 'Description' in data:
        update_data['Description'] = data['Description']
    res = db.Models.update_one({'_id': ObjectId(id)}, {'$set': update_data})
    if res.modified_count == 0:
        return jsonify({"error": "Not updated"}), 400
    return jsonify({"message": "Model updated successfully"})


@app.route('/models/retrieval/<id>', methods=['GET'])
def retrieve_model(id):
    model = db.Models.find_one({'_id': ObjectId(id)}, {'_id': 1, 'Path': 1})
    if model and 'Path' in model:
        return send_file(model['Path'], as_attachment=True)
    else:
        return jsonify({"error": "Model not found or address missing"}), 404


@app.route('/datasets', methods=['GET'])
def list_all_datasets():
    datasets = db.Data.find({}, {'_id': 1, 'DataName': 1, 'Path': 1, 'Description': 1})
    datasets_list = [{
        '_id': str(dataset['_id']),
        'DataName': dataset['DataName'],
        'Path': dataset['Path'],
        'Description': dataset.get('Description', '')
    } for dataset in datasets]
    return jsonify(datasets_list)


@app.route('/datasets/upload', methods=['POST'])
def upload_dataset():
    if 'file' not in request.files:
        return jsonify({'error': 'No file part'}), 400

    file = request.files['file']
    if file.filename == '':
        return jsonify({'error': 'No selected file'}), 400

    if file:
        filename = secure_filename(file.filename)
        path = str(Path(app.config['DATASET_UPLOAD_FOLDER']) / filename)
        file.save(path)

        # Check if the dataset with the same DataName exists
        existing_dataset = db.Data.find_one({'DataName': filename})

        if existing_dataset:
            return jsonify({'message': 'Dataset updated successfully', 'filename': filename}), 200
        else:
            # Insert new dataset
            dataset = {
                'DataName': filename,
                'Path': path,
                'Description': ''
            }
            db.Data.insert_one(dataset)
            return jsonify({'message': 'Dataset uploaded successfully', 'filename': filename}), 200


@app.route('/datasets/<id>', methods=['GET'])
def list_dataset_info(id):
    dataset = db.Data.find_one({'_id': ObjectId(id)}, {'_id': 1, 'DataName': 1, 'Path': 1, 'Description': 1})
    if dataset:
        dataset['_id'] = str(dataset['_id'])
        return jsonify(dataset)
    else:
        return jsonify({"error": "Dataset not found"}), 404


@app.route('/datasets/<id>', methods=['PUT'])
def update_dataset(id):
    data = request.json
    update_data = {}
    if 'Description' in data:
        update_data['Description'] = data['Description']
    res = db.Data.update_one({'_id': ObjectId(id)}, {'$set': update_data})
    if res.modified_count == 0:
        return jsonify({"error": "Not updated"}), 400
    return jsonify({"message": "Dataset updated successfully"})


@app.route('/datasets/retrieval/<id>', methods=['GET'])
def retrieve_dataset(id):
    dataset = db.Data.find_one({'_id': ObjectId(id)}, {'_id': 1, 'Path': 1})
    if dataset and 'Path' in dataset:
        return send_file(dataset['Path'], as_attachment=True)
    else:
        return jsonify({"error": "Dataset not found or address missing"}), 404


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
