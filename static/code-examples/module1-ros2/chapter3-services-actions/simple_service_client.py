import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')
        
        # Create a client for the add_two_ints service
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to become available...')
        
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send a request to add two integers.
        
        Args:
            a: First integer to add
            b: Second integer to add
            
        Returns:
            Future object that will hold the response
        """
        self.req.a = a
        self.req.b = b
        
        self.get_logger().info(
            f'Sending request: {a} + {b}'
        )
        
        # Asynchronously call the service
        future = self.client.call_async(self.req)
        return future


def main(args=None):
    rclpy.init(args=args)
    
    service_client = SimpleServiceClient()
    
    # Send a request for addition
    a = 10
    b = 20
    
    future = service_client.send_request(a, b)
    
    try:
        # Wait for the response (blocking until response arrives)
        rclpy.spin_until_future_complete(service_client, future)
        
        if future.result() is not None:
            response = future.result()
            
            service_client.get_logger().info(
                f'Result of {a} + {b} = {response.sum}'
            )
        else:
            service_client.get_logger().error(
                f'Exception while calling service: {future.exception()}'
            )
    
    except KeyboardInterrupt:
        service_client.get_logger().info('Interrupted during service call')
    finally:
        service_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()