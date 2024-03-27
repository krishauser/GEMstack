from flask import Flask, request, send_file
from YOLOv8 import YOLOv8  
import cv2
import numpy as np
import io

app = Flask(__name__)

model = YOLOv8()

@app.route('/detect', methods=['POST'])
def detect_objects():
    if 'image' not in request.files:
        return 'No image provided', 400

    file_stream = request.files['image'].read()

    npimg = np.frombuffer(file_stream, np.uint8)
    
    image = cv2.imdecode(npimg, cv2.IMREAD_COLOR)

    result_image = model.object_detect_all(image)

    is_success, buffer = cv2.imencode(".jpg", result_image)
    io_buf = io.BytesIO(buffer)

    return send_file(io_buf, mimetype='image/jpeg')

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
