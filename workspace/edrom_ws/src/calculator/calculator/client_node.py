#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
     
class CalculatorClientNode(Node): 
    def __init__(self):
        super().__init__("calculator_client")
        # create a client object
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        # wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again . . .')
        # create an empty request object
        self.req = AddTwoInts.Request()
        
    def send_request(self, a, b):
        """
        Function which send the request to the server
        """
        self.req.a = a
        self.req.b = b
        # call the service asynchronously
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    # check if number arguments was passed
    if len(sys.argv) != 3:
        print("Uso: ros2 run calculator_py calculator_client <numero1> <numero2>")
        return
    # create the client node   
    calculator_client = CalculatorClientNode()
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    response = calculator_client.send_request(a, b)
    if response:
        calculator_client.get_logger().info(
            f"Result: {a} + {b} = {response.sum}"
            )
    else:
        calculator_client.get_logger().error(
            "Service call failed!"
            )        

    rclpy.spin(calculator_client)
    calculator_client.destroy_node()
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()