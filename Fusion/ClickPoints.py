import os
import cv2
import time
import numpy as np





if __name__ == "__main__":
    imgPath = os.path.join(
        os.path.dirname(__file__),
        r"./Pics/FrontCam.png"
    )
    
    clickPoints(imgPath)
