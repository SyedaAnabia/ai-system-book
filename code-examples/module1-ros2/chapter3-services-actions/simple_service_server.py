from rclpy.node import Node
import rclpy

# Using the common AddTwoInts service as an example
from example_interfaces.srv import AddTwoInts


class SimpleAddServiceServer(Node):
    def __init__(self):
        super().__init__('simple_add_service_server')
        
        # Create the service server
        self.service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        
        self.get_logger().info('Simple add service is ready.')

    def add_two_ints_callback(self, request, response):
        """
        Callback function that adds two integers.
        
        Args:
            request: The incoming request with a and b values
            response: The response object to be populated with sum
            
        Returns:
            response: The filled response object with the sum
        """
        self.get_logger().info(
            f'Received add request: a={request.a}, b={request.b}'
        )
        
        # Calculate the sum
        response.sum = request.a + request.b

        self.get_logger().info(
            f'Service calculation completed: {request.a} + {request.b} = {response.sum}'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    service_server = SimpleAddServiceServer()
    
    try:
        rclpy.spin(service_server)
    except KeyboardInterrupt:
        pass
    finally:
        service_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()