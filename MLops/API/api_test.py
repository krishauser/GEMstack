from api import APIClient

temp = 'http://e3e7-130-126-255-56.ngrok-free.app/models/test'

client = APIClient('http://localhost:5000')  

client.get_file(temp)# Replace with your server's URL
