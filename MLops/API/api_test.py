from api import APIClient
import os

client = APIClient()  

wd = os.getcwd()

client.list_models_info()
