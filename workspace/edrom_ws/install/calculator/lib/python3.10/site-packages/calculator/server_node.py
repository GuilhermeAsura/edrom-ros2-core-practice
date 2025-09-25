#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
     
class CalculatorServerNode(Node): 
    """
    Server node which sum two integers
    """
    def __init__(self):
        super().__init__("calculator_server")
        # create a service object
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints', 
            self.add_two_ints_callback
            )
        # 'I'm ready' log 
        self.get_logger().info("Ready to add two integers.")
     
    def add_two_ints_callback(self, request, response):
        """
        Callback function which process the request and return the response
        """
        # sum the two integers
        response.sum = request.a + request.b
        # display log info in the console
        self.get_logger().info(
            f"Incoming request:\n    {request.a} + {request.b} = {response.sum}"
            )
        return response

def main(args=None):
    rclpy.init(args=args)
    calculator_server = CalculatorServerNode()
    rclpy.spin(calculator_server)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()