from flask import Flask, jsonify, send_file, render_template
from pymongo import MongoClient
from bson.objectid import ObjectId
import json

app = Flask(__name__)


def load_config(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"The config file {file_path} does not exist.")

    with open(file_path) as config_file:
        config = json.load(config_file)

        # Validate necessary keys in the configuration
        required_keys = ['MONGO_URI', 'DATABASE_NAME']
        if not all(key in config for key in required_keys):
            raise ValueError(f"Config file is missing required keys. Required keys: {required_keys}")

        return config

try:
    config = load_config('config.json')
except FileNotFoundError as e:
    print(f"Error: {e}")
    exit(1)
except ValueError as e:
    print(f"Error: {e}")
    exit(1)


@app.route('/models/')
def get_models():
    models = db.models.find()
    return jsonify([model for model in models])


@app.route('/models/<id>')
def get_model(id):
    # To be implemented
    model_info = db.models.find_one({'_id': ObjectId(id)})
    if model:
        return jsonify(model_info)
    else:
        return jsonify({"error": "Model not found"}), 404


@app.route('/models/retrieval/<id>')
def retrieve_model(id):
    # To be implemented
    model = db.models.find_one({'_id': ObjectId(id)})
    if model and 'path' in model:
        return send_file(model['path'], attachment_filename=id, as_attachment=True)
    else:
        return jsonify({"error": "Model not found or path missing"}), 404


@app.route('/datasets/')
def get_datasets():
    datasets = db.datasets.find()
    return jsonify([dataset for dataset in datasets])


@app.route('/datasets/<id>')
def get_dataset(id):
    # To be implemented
    dataset = db.datasets.find_one({'_id': ObjectId(id)})
    if dataset:
        return jsonify(dataset)
    else:
        return jsonify({"error": "Dataset not found"}), 404


@app.route('/datasets/retrieval/<id>')
def retrieve_dataset(id):
    # To be implemented
    dataset = db.datasets.find_one({'_id': ObjectId(id)})
    if dataset and 'path' in dataset:
        return send_file(dataset['path'], attachment_filename=f"{id}.zip", as_attachment=True)
    else:
        return jsonify({"error": "Dataset not found or path missing"}), 404


if __name__ == '__main__':
    app.run(debug=True)
