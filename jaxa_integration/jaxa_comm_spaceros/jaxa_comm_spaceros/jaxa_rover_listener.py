import rclpy
from rclpy.node import Node
from racs2_msg.msg import RACS2UserMsg
from jaxa_comm_spaceros.RACS2Bridge_std_msgs_pb2 import RACS2Bridge_std_msgs
from std_srvs.srv import Empty

class JaxaRoverListener(Node):
    def __init__(self):
        super().__init__('jaxa_rover_listener')
        
        # Create service clients
        self.clients_dict = {
            'move_stop': self.create_client(Empty, '/move_stop'),
            'move_forward': self.create_client(Empty, '/move_forward'),
            'turn_left': self.create_client(Empty, '/turn_left'),
            'turn_right': self.create_client(Empty, '/turn_right'),
            'open_arm': self.create_client(Empty, '/open_arm'),
            'close_arm': self.create_client(Empty, '/close_arm'),
            'mast_open': self.create_client(Empty, '/mast_open'),
            'mast_close': self.create_client(Empty, '/mast_close')
        }

        # Set up subscription
        self.subscription = self.create_subscription(
            RACS2UserMsg,
            '/Recv/RACS2Bridge',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info('Subscribing: [/Recv/RACS2Bridge]')
        
        # Initialize the message object
        message = RACS2Bridge_std_msgs()
        message.ParseFromString(b''.join(msg.body_data))
        
        if message.HasField("string_data"):
            command = int(message.string_data.split()[-1])
            self.get_logger().info(f'Received command: {command}')
            
            service_name = self.get_service_name(command)
            if service_name:
                self.get_logger().info(f'Calling service: {service_name}')
                self.call_service_async(self.clients_dict[service_name])
            else:
                self.get_logger().warning(f'Unknown command: {command}')

    def get_service_name(self, command):
        service_map = {
            0: 'move_stop',
            1: 'move_forward',
            2: 'turn_left',
            3: 'turn_right',
            4: 'open_arm',
            5: 'close_arm',
            6: 'mast_open',
            7: 'mast_close'
        }
        return service_map.get(command, None)

    def call_service_async(self, client):
        """
        Initiates an asynchronous service call and registers a callback to handle the result.
        
        Args:
            client (rclpy.client.Client): The ROS 2 service client to call.
        """
        if not client.service_is_ready():
            self.get_logger().info(f'Waiting for service {client.service_name} to be available...')
            client.wait_for_service()
        
        future = client.call_async(Empty.Request())
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """
        Callback function to handle the result of an asynchronous service call.
        
        Args:
            future (rclpy.future.Future): The future representing the result of the service call.
        """
        try:
            result = future.result()
            self.get_logger().info('Service call successful.')
        except Exception as e:
            self.get_logger().error(f'Exception during service call: {e}')

def main(args=None):
    rclpy.init(args=args)
    jaxa_rover_listener = JaxaRoverListener()
    rclpy.spin(jaxa_rover_listener)
    jaxa_rover_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

