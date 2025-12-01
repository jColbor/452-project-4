import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32
from geometry_msgs.msg import Point, Pose2D # Received from dispatch_node
# Point received from dispatch_node: x = displacement in x, y = displacement in y, z = observation (0=dark, 1=light)
# Pose2D to publish best estimate of current pose.
from nav_msgs.msg import OccupancyGrid # OccupancyGrid to publish the map

from localization.helper_functions import extract_map, scorePoint, scorePath, publish_map

import math

class ParticleFilterNode(Node):
    def __init__(self, name: str = 'particle_filter'):
        super().__init__(name)
        # Publishers & subscribers
        self.displacement_sub = None
        self.compass_sub = None
        self.map_pub = None
        self.pose_pub = None

        # Stored variables with global scope in the node
        self.current_compass = 0.0 # Taken from raw compass input, -pi to +pi in radians. Needed for Pose2D
        self.particles = []  # List to store particles?
        self.map = None # Extract map from file once & store here?

        # self.debug_file = open('debug.txt', 'w')

        self.get_logger().info('ParticleFilterNode constructed')

    def initialize(self):
        """Initialize publishers, subscriptions, and timers for the node."""
        self.displacement_sub = self.create_subscription(Point, '/dispatch_out', self.displacement_handler, 10)
        self.compass_sub = self.create_subscription(Float32, '/compass', self.compass_handler, 10)

        self.publish_map_timer = self.create_timer(5.0, self.publish_map) # Published every 5 seconds as instructed

        self.map_pub = self.create_publisher(OccupancyGrid, '/floor', 10)
        self.pose_pub = self.create_publisher(Pose2D, '/estimated_pose', 10)

        self.map = extract_map(self) # Uses helper fxn to extract map corresponding to bag_in parameter.
        self.get_logger().info('ParticleFilterNode initialized')
    
    
    def publish_map(self): #Called every 5 seconds by publish_map_timer
        # Need to publish the map using OccupancyGrid.
        publish_map(self.map, self.map_pub)
        self.get_logger().info(f'Published map')
    

    def compass_handler(self, msg: Float32):
        #self.get_logger().info(f'Received compass: heading={msg.data}')
        self.current_compass = msg.data

    
    def displacement_handler(self, msg: Point):
        # Handle incoming displacement and observation data.
        dx = msg.x
        dy = msg.y
        observation = msg.z # Float from 0-1, 0=dark, 1=light.

        # Update particles based on displacement (transition model).
        # Remember to add random noise to each particle's movement independently.


        # Decide when to do resampling. Don't need to do it every time, but that might still work.
        pass
        


def main():
    rclpy.init()
    node = ParticleFilterNode()
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