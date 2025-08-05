import espeakng
import rclpy 
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue 
import threading
import time

class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name}, start !')
        self.novels_queue_ = Queue()
        self.novel_subscriber_ = self.create_subscription(String, 'novel', self.novel_callback, 10)
        self.speech_thread_ = threading.Thread(target=self.speaker_thread)
        self.speech_thread_.start()

    def novel_callback(self, msg):
        self.novels_queue_.put(msg.data)

    def speaker_thread(self):
        speaker = espeakng.Speaker()
        speaker.voice = 'us'
        
        while rclpy.ok():
            if self.novels_queue_.qsize()>0:
                text = self.novels_queue_.get()
                self.get_logger().info(f'speak: {text}')
                speaker.say(text)
                speaker.wait()
            else:
                time.sleep(1)

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
    node = NovelSubNode('novel_sub')
    rclpy.spin(node)
    rclpy.shutdown()