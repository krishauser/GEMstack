import os
import cv2
import time
import numpy as np


def clickPoints(imgPath=None, numPoints=4):
    clicked_pts = []

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(param) < numPoints:
            param.append((x, y))
            print(f"点击第 {len(param)} 个点: ({x}, {y})")

    # 初始化图像
    if imgPath:
        img = cv2.imread(imgPath, flags=cv2.IMREAD_COLOR)
    else:
        img = np.ones((600, 800, 3), dtype=np.uint8) * 255  # 白色背景

    cv2.namedWindow("Click Points")
    cv2.setMouseCallback("Click Points", mouse_callback, clicked_pts)

    while True:
        img_copy = img.copy()

        # 画点击点
        for pt in clicked_pts:
            cv2.circle(img_copy, pt, 5, (0, 0, 255), -1)

        if len(clicked_pts) == numPoints:
            cv2.polylines(img_copy, [np.array(clicked_pts)], isClosed=True, color=(0, 255, 0), thickness=2)

        cv2.imshow("Click Points", img_copy)

        key = cv2.waitKey(10)
        if key == 27 or key == ord("q"):  # ESC
            break
        elif key == ord('r'):
            clicked_pts.clear()
        
    cv2.destroyAllWindows()

    # 输出
    if len(clicked_pts) == numPoints:
        print("\n最终点击的坐标：")
        for i, pt in enumerate(clicked_pts):
            print(f"点 {i+1}: {pt}")

    return np.array(clicked_pts, dtype=np.float32)


if __name__ == "__main__":
    imgPath = os.path.join(
        os.path.dirname(__file__),
        r"./Pics/FrontCam.png"
    )
    
    clickPoints(imgPath)
