import cv2

# 鼠标回调函数
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f'Pixel Coordinates: (x={x}, y={y})')

# 读取图片
image = cv2.imread('data/hw3_data/color1.png')

# 创建窗口并绑定鼠标回调函数
cv2.namedWindow('Image')
cv2.setMouseCallback('Image', click_event)

# 显示图片
cv2.imshow('Image', image)

# 等待键盘事件
cv2.waitKey(0)
cv2.destroyAllWindows()
