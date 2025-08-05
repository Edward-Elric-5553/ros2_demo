import rclpy
from rclpy.node import Node 

def main():
    rclpy.init()
    
    node = Node("python_node")
    node.get_logger().info('hello python node')
    node.get_logger().warn('hello python node')
    node.get_logger().error('hello python node')

    rclpy.spin(node)

    rclpy.shutdown()

    
if __name__ == '__main__':
    main()

