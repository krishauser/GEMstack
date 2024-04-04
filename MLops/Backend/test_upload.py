import requests
url = 'http://localhost:5000/models/upload'
filepath = 'Schema.js'
with open(filepath, 'rb') as f:
    response = requests.post(url, files={'file': (filepath, f)})
    print(response)

