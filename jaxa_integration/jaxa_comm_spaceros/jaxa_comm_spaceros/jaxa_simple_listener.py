"""
JAXA Simple Listener Node

This ROS 2 node subscribes to messages from the JAXA RACS2 bridge and processes
incoming data.

Author: Minahil Raza
Email: minahilrz@gmail.com

This script subscribes to the '/Recv/RACS2Bridge' topic, receives messages
of type `RACS2UserMsg`, parses the message content, and logs any string data
that is extracted.

Usage:
    This script is intended to be run within a ROS 2 environment. It should be
    included in a ROS 2 package and can be launched using the `ros2 run` command
    or through a ROS 2 launch file.

"""

import rclpy
from rclpy.node import Node
from racs2_msg.msg import RACS2UserMsg
from jaxa_comm_spaceros.RACS2Bridge_std_msgs_pb2 import RACS2Bridge_std_msgs


class JaxaSimpleListener(Node):
    """
    A simple ROS 2 subscriber node for receiving and processing messages
    from the JAXA RACS2 bridge.

    This node listens on the '/Recv/RACS2Bridge' topic and processes
    incoming `RACS2UserMsg` messages. It extracts and logs any string
    data contained within the messages.
    """

    def __init__(self):
        """Initialize the JaxaSimpleListener node and set up the subscription."""
        super().__init__('jaxa_simple_listener')
        self.subscription = self.create_subscription(
            RACS2UserMsg,
            '/Recv/RACS2Bridge',
            self.listener_callback,
            10
        )
        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        """
        Callback function executed upon receiving a message.

        Parses the incoming `RACS2UserMsg` message to extract string data
        and logs it.

        Args:
            msg (RACS2UserMsg): The message received from the subscribed topic.
        """
        self.get_logger().info('Subscribing: [/Recv/RACS2Bridge]')
        
        # Initialize the message object
        message = RACS2Bridge_std_msgs()
        
        # Parse the incoming message body data
        message.ParseFromString(b''.join(msg.body_data))
        
        # Check if the parsed message contains string data and log it
        if message.HasField("string_data"):
            self.get_logger().info(f'Received string data: {message.string_data}')


def main(args=None):
    """
    Main function to run the JaxaSimpleListener node.

    Initializes the ROS 2 client library, spins the node, and handles node shutdown.
    """
    
    rclpy.init(args=args)

    # Create and spin the JaxaSimpleListener node
    jaxa_simple_listener = JaxaSimpleListener()
    rclpy.spin(jaxa_simple_listener)

    # Destroy the node explicitly before shutting down
    jaxa_simple_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':    
    main()

