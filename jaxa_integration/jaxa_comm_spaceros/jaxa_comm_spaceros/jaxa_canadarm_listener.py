import rclpy
from rclpy.node import Node
from racs2_msg.msg import RACS2UserMsg
from jaxa_comm_spaceros.RACS2Bridge_std_msgs_pb2 import RACS2Bridge_std_msgs
from std_srvs.srv import Empty
import re

class CanadarmListener(Node):
    """
    A ROS 2 subscriber node for controlling the Canadarm.

    This node listens on the '/Recv/RACS2Bridge' topic and processes
    incoming `RACS2UserMsg` messages. Based on the message content,
    it calls the appropriate ROS 2 services to control the Canadarm.
    """

    def __init__(self):
        """Initialize the CanadarmListener node and set up the subscription."""
        super().__init__('canadarm_listener')

        # Create a dictionary of service clients
        self.clients_dict = {
            'open_arm': self.create_client(Empty, '/open_arm'),
            'close_arm': self.create_client(Empty, '/close_arm'),
            'random_arm': self.create_client(Empty, '/random_arm')
        }

        # Set up subscription
        self.subscription = self.create_subscription(
            RACS2UserMsg,
            '/Recv/RACS2Bridge',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        """
        Callback function executed upon receiving a message.

        Args:
            msg (RACS2UserMsg): The message received from the subscribed topic.
        """
        self.get_logger().info('Received message on /Recv/RACS2Bridge')

        # Initialize the message object
        message = RACS2Bridge_std_msgs()
        message.ParseFromString(b''.join(msg.body_data))

        # Check if the parsed message contains string data
        if message.HasField("string_data"):
            command = int(message.string_data.split()[-1])
            self.get_logger().info(f'Received command: {command}')
            # Determine which service to call based on the command
            service_name = self.get_service_name(command)
            if service_name:
                self.get_logger().info(f'Calling service: {service_name}')
                self.call_service_async(self.clients_dict[service_name])
        else:
            self.get_logger().warning(f'Unknown command: {command}')

    def get_service_name(self, command):
        """Return the service name based on the command."""
        service_map = {
            0: 'close_arm',
            1: 'open_arm',
            2: 'random_arm',
        }
        return service_map.get(command, None)

    def call_service_async(self, client):
        """
        Initiates an asynchronous service call and registers a callback to handle the result.

        Args:
            client (rclpy.client.Client): The ROS 2 service client to call.
        """
        if not client.service_is_ready():
            self.get_logger().info(f'Waiting for service {client.srv_name} to be available...')
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
    """
    Main function to run the CanadarmListener node.

    Initializes the ROS 2 client library, spins the node, and handles node shutdown.
    """
    rclpy.init(args=args)

    # Create and spin the CanadarmListener node
    canadarm_listener = CanadarmListener()
    rclpy.spin(canadarm_listener)

    # Destroy the node explicitly before shutting down
    canadarm_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

