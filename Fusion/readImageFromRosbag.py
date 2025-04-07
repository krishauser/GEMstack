import rosbag
import cv2
from cv_bridge import CvBridge
import os


def readImageFromRosbag(rosbagPath, imageTopic="/oak/rgb/image_raw", outputPath="output_image.png"):
    # 设置路径和参数
    # rosbagPath =  os.path.join(os.path.dirname(__file__), r"../logs/stitch/front.bag")
    # 初始化 cv_bridge
    bridge = CvBridge()

    # 打开 rosbag 文件
    with rosbag.Bag(rosbagPath, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[imageTopic]):
            try:
                # 将ROS图像消息转为OpenCV格式
                cvImage = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                # 保存图像
                if outputPath:
                    cv2.imwrite(outputPath, cvImage)
                    print(f"图片保存到: {outputPath}")
                break  # 只取一张就break
            except Exception as e:
                print(f"转换图像失败: {e}")
    
    return cvImage




