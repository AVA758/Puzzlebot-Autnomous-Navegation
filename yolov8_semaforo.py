#!/usr/bin/env python3
#ORIGINALLLL


from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from yolo_msg.msg import InferenceResult
from yolo_msg.msg import Yolov8Inference
from std_msgs.msg import String

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Initialize the YOLO model
        self.model = YOLO('~/ros2_ws/src/yolov8_ros2/yolov8_ros2/semaforos.pt')
        
        self.yolov8_inference = Yolov8Inference()
        self.bridge = CvBridge()
        
        # Subscription to the camera image topic
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)

        self.image_width = 800
        self.image_height = 800
        self.img = np.ndarray((self.image_height, self.image_width, 3))
        
        # Publishers for inference results and image
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.state_pub = self.create_publisher(String, "/traffic_color", 1)
        
        # Timer to periodically perform inference
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        results = self.model(self.img)
        
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        detected_objects = []  # Collect detected objects' names

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                class_name = self.model.names[int(c)]
                detected_objects.append(class_name)  # Add detected object's name

        # Publish the names of detected objects one at a time
        for detected_object in detected_objects:
            object_name_msg = String()
            object_name_msg.data = detected_object

            if object_name_msg.data == "rojo" or object_name_msg.data == "amarillo" or object_name_msg.data == "verde":
                self.state_pub.publish(object_name_msg)

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # Rotate the image to correct the initial upside-down orientation
        # img = cv2.rotate(img, cv2.ROTATE_180)
        
        # Crop the image based on the specified coordinates
        # For example, to crop the region from row 140 to 180 and column 173 to 260

        #cropped_img = img[20:180, 90:260]
        
        # Resize the cropped image to 800x800 (or other desired size if needed)
        
        self.img = cv2.resize(img, (self.image_width, self.image_height))
        

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

