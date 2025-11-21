import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist # /cmd_vel
from example_interfaces.msg import Float32, UInt8 # /compass, /floor_sensor
import math


# Subscribes to all raw input messages (compass, cmd_vel, and floor_sensor)
# Periodically publishes the path:
#  1 - List of displacements from start based on cmd_vel & compass
#  2 - Corresponding observation (light or dark) based on floor_sensor readings

class DispatchNode(Node):
    def __init__(self, name: str = 'dispatch_node'):
        super().__init__(name)
        # Publishers & subscribers
        self.path_publisher = None
        self.cmd_vel_sub = None
        self.compass_sub = None
        self.floor_sensor_sub = None

        # Stored variables with global scope in the node
        self.StoredPath = []  # List to store path displacements & observations. Will be published.
        self.current_compass = 0.0 # Current compass heading
        self.floor_sensors_raw = [] # Should have timestamps
        self.displacements = [(0,0)]    # Should have timestamps
        
        # Used to calculate displacement from velocity
        self.last_vel = 0.0
        self.last_time = 0.0

        self.get_logger().info('DispatchNode constructed')
    
    def initialize(self):
        """Initialize publishers, subscriptions, and timers for the node."""
        self._pub = self.create_publisher(String, 'dispatch_out', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_handler, 10)
        self.compass_sub = self.create_subscription(Float32, '/compass', self.compass_handler, 10)
        self.floor_sensor_sub = self.create_subscription(UInt8, '/floor_sensor', self.floor_sensor_handler, 10)
        self._timer = self.create_timer(1.0, self._on_timer)
        self.get_logger().info('DispatchNode initialized')
    
    def cmd_vel_handler(self, msg: Twist):
        #self.get_logger().info(f'Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}')
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 if msg.header #Time from header
        vel = msg.linear.x
        if self.last_time != 0.0:
            dt = time - self.last_time
            displacement = self.last_vel * dt
            # Calculate new position based on current compass
            dx = displacement * math.cos(self.current_compass)
            dy = displacement * math.sin(self.current_compass)
            last_x, last_y = self.displacements[-1]
            new_x = last_x + dx
            new_y = last_y + dy
            self.displacements.append((new_x, new_y))
        
        self.last_vel = vel
        self.last_time = time

    
    def compass_handler(self, msg: Float32):
        #self.get_logger().info(f'Received compass: heading={msg.data}')
        self.current_compass = msg.data
    
    def floor_sensor_handler(self, msg: UInt8):
        self.get_logger().info(f'Received floor_sensor: value={msg.data}')
        # Process floor_sensor message to update path (placeholder logic)
        
    
    def _on_timer(self):
        # Periodically publish the stored path
        hb = String()
        hb.data = 'heartbeat'
        self._pub.publish(hb)


if __name__ == '__main__':
    rclpy.init()
    node = DispatchNode()
    try:
        node.initialize()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()