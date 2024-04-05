import requests

url = 'http://10.192.207.120:5000/model/upload'
filepath = '../model/YOLO/yolov8n.pt'
with open(filepath, 'rb') as f:
    response = requests.post(url, files={'file': (filepath, f)})
    print(response)