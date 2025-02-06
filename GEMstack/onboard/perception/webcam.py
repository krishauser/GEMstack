#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def webcam_publisher():
    # 初始化 ROS 节点
    rospy.init_node('webcam_publisher', anonymous=True)
    
    # 创建一个发布者，发布图像消息到 /webcam/image_raw 话题
    image_pub = rospy.Publisher('/webcam', Image, queue_size=10)
    
    # 初始化 OpenCV 摄像头
    cap = cv2.VideoCapture(0)  # 0 表示默认摄像头
    if not cap.isOpened():
        rospy.logerr("无法打开摄像头")
        return
    
    # 初始化 CvBridge
    bridge = CvBridge()
    
    # 设置发布频率
    rate = rospy.Rate(30)  # 30 Hz
    
    try:
        while not rospy.is_shutdown():
            # 读取摄像头帧
            ret, frame = cap.read()
            if not ret:
                rospy.logerr("无法读取摄像头帧")
                break
            
            # 显示摄像头图像
            cv2.imshow("Webcam", frame)
            
            # 检查是否按下 'q' 键
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.loginfo("用户按下 'q' 键，请求退出")
                break
            
            # 将 OpenCV 图像转换为 ROS 图像消息
            try:
                ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            except Exception as e:
                rospy.logerr(f"转换图像失败: {e}")
                continue
            
            # 发布图像消息
            image_pub.publish(ros_image)
            rospy.loginfo("发布了一帧图像")
            
            # 按频率休眠
            rate.sleep()
    
    except KeyboardInterrupt:
        # 捕获 Ctrl+C 信号
        rospy.loginfo("用户请求退出")
    
    finally:
        # 释放摄像头资源
        cap.release()
        cv2.destroyAllWindows()
        rospy.loginfo("摄像头资源已释放")

if __name__ == '__main__':
    webcam_publisher()