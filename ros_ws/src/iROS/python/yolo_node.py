#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import sensor_msgs.msg
# import cv_bridge  # REMOVED

import numpy as np
import cv2
import threading
import PIL
import re
import torch
from ultralytics import YOLO


def imgmsg_to_cv2(img_msg, encoding="bgr8"):
    """Convert ROS Image message to OpenCV image without cv_bridge"""
    dtype = np.uint8
    
    if encoding == "bgr8" or encoding == "rgb8":
        channels = 3
    elif encoding == "mono8":
        channels = 1
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")
    
    # Convert bytes to numpy array
    img_array = np.frombuffer(img_msg.data, dtype=dtype)
    
    # Reshape to image dimensions
    cv_image = img_array.reshape((img_msg.height, img_msg.width, channels)) if channels > 1 else img_array.reshape((img_msg.height, img_msg.width))
    
    return cv_image

def cv2_to_imgmsg(cv_image, encoding="bgr8"):
    """Convert OpenCV image to ROS Image message without cv_bridge"""
    img_msg = sensor_msgs.msg.Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = encoding
    
    if encoding == "bgr8":
        img_msg.step = cv_image.shape[1] * 3
    elif encoding == "mono8":
        img_msg.step = cv_image.shape[1]
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")
    
    img_msg.data = cv_image.tobytes()
    img_msg.is_bigendian = 0
    
    return img_msg

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.declare_parameter('iphone_name','iPhone')
        self.check_topics()
        self.pubs = {}
        self.subs = []
        # In __init__:
        self.model = YOLO('yolo11n.pt')  # Same size options as v8
        #self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.to('cuda' if torch.cuda.is_available() else 'cpu')
        # self.bridge = cv_bridge.CvBridge()  # REMOVED

    def check_topics(self):        
        for node_name in self.get_node_names():
            for topic_name, _ in self.get_publisher_names_and_types_by_node(node_name, ''):
                if re.match(r"^/iphone/.*/eo$",topic_name):
                    if not topic_name in self.pubs.keys():
                        self.get_logger().info(f"Creating subscription for {topic_name}!")
                        self.pubs[topic_name] = self.create_publisher(
                            sensor_msgs.msg.Image,
                            topic_name.replace("/eo","/eo_yolo"),
                            10
                        )
                        self.subs.append(
                            self.create_subscription(
                                sensor_msgs.msg.Image,
                                topic_name,
                                lambda msg, topic=topic_name: self.process_frame(topic, msg),
                                10
                            )
                        )
        self.timer = threading.Timer(1,self.check_topics).start()

    def process_frame( self, topic_name, frame ):
        im_cv = imgmsg_to_cv2(frame, encoding="bgr8")

        results = self.model(im_cv)
        self.get_logger().info(f"Received frame, {topic_name} {results}!")

        detections = results.pandas().xyxy[0]
        for index, row in detections.iterrows():
            x1, y1, x2, y2 = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
            label = row['name']
            confidence = row['confidence']
            print(f"Detected: {label} with confidence {confidence:.2f} at [{x1}, {y1}, {x2}, {y2}]")

            # Optionally, draw bounding boxes on the image using OpenCV
            cv2.rectangle(im_cv, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.putText(im_cv, f"{label} {confidence:.2f}", (x1, y1 - 5), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)
        self.pubs[topic_name].publish(cv2_to_imgmsg(im_cv))

def main(args=None):
    rclpy.init(args=args)

    yn = YoloNode()

    try:
        rclpy.spin(yn)
    except KeyboardInterrupt:
        pass
    finally:
        yn.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
