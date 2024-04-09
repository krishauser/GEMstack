from api import APIClient
import os

id = '6611bfa1432e9a2516c1cf66'
base_url = 'http://127.0.0.1:5000'

client = APIClient(base_url)  

wd = os.getcwd()

client.dataset_upload(f"{wd}/yolov8n_test1.pt")
