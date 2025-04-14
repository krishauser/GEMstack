import rosbag
import cv2
from cv_bridge import CvBridge
import os

# === 参数设置 ===
bag_file = "/home/lzy//Projects/GEMstack/logs/stitch/stitching_front3.bag"

# topic_name = "/oak/rgb/image_raw"
# topic_name = "/camera_fl/arena_camera_node/image_raw"
topic_name = "/camera_fr/arena_camera_node/image_raw"

output_dir = "./Pics"
max_images = 1

# 创建输出目录
os.makedirs(output_dir, exist_ok=True)

# 初始化 CvBridge
bridge = CvBridge()

# 开始读取 rosbag
count = 0
with rosbag.Bag(bag_file, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)  # 转换颜色空间
            
            # 构造图像文件名
            filename = os.path.join(output_dir, f"fr.png")
            cv2.imwrite(filename, cv_img)
            print(f"Saved image {count} to {filename}")
            
            count += 1
            if count >= max_images:
                break

        except Exception as e:
            print(f"Error converting image: {e}")

print("Done extracting images.")
