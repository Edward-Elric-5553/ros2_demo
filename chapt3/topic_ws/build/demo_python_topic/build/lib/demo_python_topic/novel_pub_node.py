import rclpy 
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue 

class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name}, start !')
        self.novel_queue_ = Queue()
        self.novel_publisher_ = self.create_publisher(String, 'novel', 10)
        self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        if self.novel_queue_.qsize()>0:
            line = self.novel_queue_.get()
            msg = String()
            msg.data = line
            self.novel_publisher_.publish(msg)
            self.get_logger().info(f'publish : {msg}')

    def download(self, url):
        response = requests.get(url)
        response.encoding = 'utf-8'
        text = response.text
        self.get_logger().info(f'download{url}, {len(text)}')
        for line in text.splitlines():
            self.novel_queue_.put(line)
        #self.novel_publisher_.publish()

def main():
    rclpy.init()
    node = NovelPubNode('novel_pub')
    node.download('http://0.0.0.0:8000/novel1.txt')
    rclpy.spin(node)
    rclpy.shutdown()

