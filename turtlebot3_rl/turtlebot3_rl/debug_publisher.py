import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DebugPublisher(Node):
    def __init__(self):
        super().__init__('debug_publisher')
        self.debug_pub = self.create_publisher(String, 'debug_info', 10)

    def publish_debug_info(self, scan_data, action, reward, done):
        # Truncate scan data for display and round for clarity
        scan_str = ', '.join([f'{x:.2f}' for x in scan_data[:10]])
        action_str = ', '.join([f'{x:.2f}' for x in action])
        
        # Format the message
        debug_msg = String()
        debug_msg.data = (
            f"Scan Data: [{scan_str}]...\n"
            f"Action: [{action_str}]\n"
            f"Reward: {reward:.2f}\n"
            f"Done: {done}\n"
            "=============================="
        )
        
        self.debug_pub.publish(debug_msg)
