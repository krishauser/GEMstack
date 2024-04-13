from api import APIClient
import os

client = APIClient(base_url='http://10.181.133.35:5000')  

wd = os.getcwd()

client.dataset_download('6615d9bb5d51d6cf4319c2f1')
