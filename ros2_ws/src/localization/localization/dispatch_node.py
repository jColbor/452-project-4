import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist # /cmd_vel
from geometry_msgs.msg import Point # Publishes displacement represented as a point

from example_interfaces.msg import Float32, UInt8 # /compass, /floor_sensor
import math
DARK_CENTER = 110
LIGHT_CENTER = 145


# Subscribes to all raw input messages (compass, cmd_vel, and floor_sensor)
# Periodically publishes a displacement & observation:
#  1 - Displacement from cmd_vel & compass
#  2 - Corresponding observation (light or dark) based on floor_sensor readings

# Take the floor sensor reading (between ~100-160) and normalize it to a value between 0 (dark) and 1 (light)
# Note: Higher sensor values typically indicate darker surfaces, so we invert the mapping
def normalize_floor_sensor(reading):
    if reading < DARK_CENTER:
        return 1.0  # Low reading = light surface
    elif reading > LIGHT_CENTER:
        return 0.0  # High reading = dark surface
    else:
        # Invert: higher reading -> lower normalized value (darker)
        return 1.0 - ((reading - DARK_CENTER) / (LIGHT_CENTER - DARK_CENTER))

# Compute average of floor sensor values.
def avg_floor(floor_sensor_values):
    if not floor_sensor_values:
        return -1.0 #indicates no reading.
    # otherwise compute average and send to normalize_floor_sensor
    avg = sum(floor_sensor_values) / len(floor_sensor_values)
    return normalize_floor_sensor(avg)


class DispatchNode(Node):
    def __init__(self, name: str = 'dispatch_node'):
        super().__init__(name)
        # Publishers & subscribers
        self.publisher = None
        self.cmd_vel_sub = None
        self.compass_sub = None
        self.floor_sensor_sub = None

        # Stored variables with global scope in the node
        self.StoredPath = []  # List to store path displacements & observations. Will be published.
        self.current_compass = 0.0 # Current compass heading
        self.floor_sensors_raw = [] # Should get more of these than cmd_vel updates
        self.displacement = (0,0)
        
        # Used to calculate displacement from velocity
        self.last_vel = 0.0
        self.last_time = 0.0

        # self.debug_file = open('debug.txt', 'w')

        self.get_logger().info('DispatchNode constructed')
    
    # Updates displacement based on last_vel, current_comass, and last_time. Sets last_time to current.
    def update_displacement(self, time):
        dt = time - self.last_time
        displacement = self.last_vel * dt
        dx = displacement * math.cos(self.current_compass)
        dy = displacement * math.sin(self.current_compass)
        self.displacement = (self.displacement[0] + dx, self.displacement[1] + dy)
        self.last_time = time

    def initialize(self):
        """Initialize publishers, subscriptions, and timers for the node."""
        self.point_publisher = self.create_publisher(Point, '/dispatch_out', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_handler, 10)
        self.compass_sub = self.create_subscription(Float32, '/compass', self.compass_handler, 10)
        self.floor_sensor_sub = self.create_subscription(UInt8, '/floor_sensor', self.floor_sensor_handler, 10)
        self._timer = self.create_timer(0.5, self._on_timer)
        self.get_logger().info('DispatchNode initialized')
    

    def cmd_vel_handler(self, msg: Twist):
        #self.get_logger().info(f'Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}')
        time = self.get_clock().now().nanoseconds * 1e-9
        self.update_displacement(time)
        self.last_vel = msg.linear.x
        self.last_time = time

    
    def compass_handler(self, msg: Float32):
        #self.get_logger().info(f'Received compass: heading={msg.data}')
        self.update_displacement(self.get_clock().now().nanoseconds * 1e-9)
        self.current_compass = msg.data
    
    def floor_sensor_handler(self, msg: UInt8):
        # self.get_logger().info(f'Received floor_sensor: value={msg.data}')
        # Store RAW sensor values, not normalized ones (normalization happens in avg_floor)
        self.floor_sensors_raw.append(msg.data)
        # self.debug_file.write(f'{msg.data},\n')

    # Periodically publish displacement & observation data
    def _on_timer(self):
        self.update_displacement(self.get_clock().now().nanoseconds * 1e-9)
        
        # Only publish if we have sensor readings (even if no movement)
        if len(self.floor_sensors_raw) == 0:
            return # No sensor readings, no need to publish
        
        # Prepare the message to publish
        displacement_observation_pt = Point()
        displacement_observation_pt.x = self.displacement[0]
        displacement_observation_pt.y = self.displacement[1]
        observation_value = avg_floor(self.floor_sensors_raw)
        self.get_logger().info(f'Sending displacement (dx={self.displacement[0]:.3f}, dy={self.displacement[1]:.3f}) with {len(self.floor_sensors_raw)} floor sensor readings, observation={observation_value:.3f}')
        displacement_observation_pt.z = observation_value
        self.floor_sensors_raw = [] # Clear stored sensor readings after sending
        # Publish the message
        self.point_publisher.publish(displacement_observation_pt)
        # Reset displacement after publishing
        self.displacement = (0, 0)
        

def main():
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

if __name__ == '__main__':
    main()