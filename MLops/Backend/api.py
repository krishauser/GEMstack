import server
import json

def listModelInfo():
    json_models = server.get_models()
    models = [json.load(model) for model in json_models]
    return models
    
def getModelInfo(id):
    model = server.get_model(id)
    return json.load(model)

def getModel(id):
    return server.retrieve_model(id)

def listDatasetInfo():
    json_datasets = server.get_datasets()
    datasets = [json.load(dataset) for dataset in json_datasets]
    return datasets

def getDatasetInfo(id):
    dataset = server.get_dataset(id)
    return json.load(dataset)

def getDataset(id):
    return server.retrieve_dataset(id)


