#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  
     
class TalkerNode(Node): 
    """ 
    'Hello World!' + counter node
    """
    def __init__(self):
        super().__init__("talker")
        # create a publisher object which '10' is the queue size
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        # timer period to call the timer_callback function
        timer = 1.0
        self.timer_ = self.create_timer(timer, self.timer_callback)
        # message counter
        self.counter = 0 
     
    def timer_callback(self):
        # create String type message, note: String =! str
        msg = String()
        # message content
        msg.data = f"Hello World! This is the {self.counter} message."
        #  publish the message in the topic
        self.publisher_.publish(msg)
        #  display log info in the console
        self.get_logger().info(f"Publishing: '{msg.data}'")
        #  increment the counter
        self.counter += 1
        
def main(args=None):
    rclpy.init(args=args)
    talker_node = TalkerNode()
    rclpy.spin(talker_node)
    talker_node.destroy_node()
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()