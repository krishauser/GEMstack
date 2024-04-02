from flask import Flask, jsonify, send_file, render_template
# from pymongo import MongoClient
import json
import os

app = Flask(__name__)


# def load_config(file_path):
#     if not os.path.exists(file_path):
#         raise FileNotFoundError(f"The config file {file_path} does not exist.")
#
#     with open(file_path) as config_file:
#         config = json.load(config_file)
#
#         # Validate necessary keys in the configuration
#         required_keys = ['MONGO_URI', 'DATABASE_NAME']
#         if not all(key in config for key in required_keys):
#             raise ValueError(f"Config file is missing required keys. Required keys: {required_keys}")
#
#         return config
#
# try:
#     config = load_config('config.json')
#     client = MongoClient(config['MONGO_URI'])
#     db = client[config['DATABASE_NAME']]
# except FileNotFoundError as e:
#     print(f"Error: {e}")
#     exit(1)
# except ValueError as e:
#     print(f"Error: {e}")
#     exit(1)
#

@app.route('/models/')
def get_models():
    models = db.Models.find({}, {'_id': 0, 'ID': 1, 'ModelName': 1, 'Path': 1, 'Description': 1})
    return jsonify(list(models))


@app.route('/models/<int:id>')
def get_model(id):
    model_info = db.Models.find_one({'ID': id}, {'_id': 0, 'ID': 1, 'ModelName': 1, 'Path': 1, 'Description': 1})
    if model_info:
        return jsonify(model_info )
    else:
        return jsonify({"error": "Model not found"}), 404


@app.route('/models/retrieval/<int:id>')
def retrieve_model(id):
    model = db.Models.find_one({'ID': id}, {'_id': 0, 'Path': 1})
    if model and 'Path' in model:
        return send_file(model['Path'], as_attachment=True)
    else:
        return jsonify({"error": "Model not found or address missing"}), 404


@app.route('/datasets/')
def get_datasets():
    datasets = db.Data.find({}, {'_id': 0, 'ID': 1, 'DataName': 1, 'Path': 1, 'Description': 1})
    return jsonify(list(datasets))


@app.route('/datasets/<int:id>')
def get_dataset(id):
    dataset = db.Data.find_one({'ID': id}, {'_id': 0, 'ID': 1, 'DataName': 1, 'Path': 1, 'Description': 1})
    if dataset:
        return jsonify(dataset)
    else:
        return jsonify({"error": "Dataset not found"}), 404


@app.route('/datasets/retrieval/<int:id>')
def retrieve_dataset(id):
    dataset = db.Data.find_one({'ID': id}, {'_id': 0, 'Path': 1})
    if dataset and 'Path' in dataset:
        return send_file(dataset['Path'], as_attachment=True)
    else:
        return jsonify({"error": "Dataset not found or address missing"}), 404


@app.route('/models/test')
def test():
    path = '../model/YOLO/yolov8n.pt'
    return send_file(path, as_attachment=True)


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
