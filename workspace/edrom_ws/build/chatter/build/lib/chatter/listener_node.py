#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  
     
class ListenerNode(Node):
    """
    'Hello World!' + counter subscriber node
    """ 
    def __init__(self):
        super().__init__("listener")
        # create a subscriber object
        self.subscription = self.create_subscription(
            String,                     
            'chatter',                  
            self.listener_callback,     
            10)            
        # prevent unused variable warning  
        self.subscription  
    
    def listener_callback(self, msg):
        # display log info in the console
        self.get_logger().info(f"Messages heard: '{msg.data}'")
    
     
def main(args=None):
    rclpy.init(args=args)
    listener_node = ListenerNode()
    rclpy.spin(listener_node)
    listener_node.destroy_node()
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()