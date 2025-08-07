import rclpy
from rclpy.node import Node
import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge
import time

class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.bridge = CvBridge()
        self.default_image_path = get_package_share_directory('demo_python_service') + '/resource/face1.jpg'
        self.get_logger().info(f'Face Detect Client Node Statr ---')
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.image = cv2.imread(self.default_image_path)

    def send_request(self):
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info(f'wait Server start ---')
        
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'get response, total {response.number} faces')
        self.show_response(response)

    def show_response(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.bottom[i]
            cv2.rectangle(self.image,(left,top),(right,bottom),(255,0,0),4)
            cv2.imshow('face detect result', self.image)
            cv2.waitKey(0)


def main():
    rclpy.init()
    node = FaceDetectClientNode()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()



