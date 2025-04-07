import cv2

for i in [0, 1]:
    cap = cv2.VideoCapture(f'/dev/video{i}')
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f'/dev/video{i} 可用，分辨率: {frame.shape[1]}x{frame.shape[0]}')
            cv2.imshow(f'video{i}', frame)
            cv2.waitKey(0)
        else:
            print(f'/dev/video{i} 打开但无法读取图像')
        cap.release()
    else:
        print(f'/dev/video{i} 无法打开')
