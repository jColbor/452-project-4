import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32
from geometry_msgs.msg import Point, Pose2D # Received from dispatch_node
# Point received from dispatch_node: x = displacement in x, y = displacement in y, z = observation (0=dark, 1=light)
# Pose2D to publish best estimate of current pose.
from nav_msgs.msg import OccupancyGrid # OccupancyGrid to publish the map
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from localization.helper_functions import extract_map, scorePoint, scorePath, publish_map
import math
import random
import time

NUM_PARTICLES = 300
MIN_MOVEMENT_FOR_ELIMINATION = 0.1
RESAMPLE_NOISE_STD_DEV = 0.1
WEIGHT_PENALTY_CONSTANT = 10.0
WEIGHT_PENALTY_PERCENTAGE = 0.3
RANDOM_RESAMPLING_PERCENTAGE = 0.97
MOTION_NOISE_ANGLE_DEG = 15.0  # Maximum angle noise in degrees for particle motion
MOTION_SCALE_MIN = 0.8  # Minimum scale factor for distance uncertainty
MOTION_SCALE_MAX = 1.2  # Maximum scale factor for distance uncertainty

class Particle:
    def __init__(self, initial_x, initial_y, observation_history=None):
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.x = initial_x
        self.y = initial_y
        self.weight = 0.0
        self.path = [(initial_x, initial_y)]  # Each particle has its own path history
        self.observation_history = observation_history  # Reference to shared observation history
    
    def __repr__(self):
        return f"Particle(x={self.x:.2f}, y={self.y:.2f}, weight={self.weight:.2f})"

class ParticleFilterNode(Node):
    def __init__(self, name: str = 'particle_filter'):
        super().__init__(name)
        self.current_compass = 0.0
        self.particles = []
        self.map = None
        self.num_particles = NUM_PARTICLES
        self.path_history_length = 10
        self.observation_history = []  # Shared observation history for all particles
        self.max_history_length = 500
        self.robot_path_x = 0.0
        self.robot_path_y = 0.0
        self.get_logger().info('ParticleFilterNode constructed')

    def initialize(self):
        """Initialize publishers, subscriptions, and timers for the node."""
        self.displacement_sub = self.create_subscription(Point, '/dispatch_out', self.displacement_handler, 10)
        self.compass_sub = self.create_subscription(Float32, '/compass', self.compass_handler, 10)
        self.publish_map_timer = self.create_timer(5.0, self.publish_map) # Published every 5 seconds as instructed
        self.publish_particles_timer = self.create_timer(0.5, self.publish_particles)
        self.publish_observations_timer = self.create_timer(0.5, self.publish_observation_timeline)
        self.map_pub = self.create_publisher(OccupancyGrid, '/floor', 10)
        self.pose_pub = self.create_publisher(Pose2D, '/estimated_pose', 10)
        self.particles_pub = self.create_publisher(MarkerArray, '/particles', 10)
        self.observations_pub = self.create_publisher(MarkerArray, '/observation_timeline', 10)
        self.map = extract_map(self) # Uses helper fxn to extract map corresponding to bag_in parameter.
        self.clear_all_visualizations()
        self.initialize_particles()
        publish_map(self.map, self.map_pub)
        self.get_logger().info('ParticleFilterNode initialized - map published')
    
    def initialize_particles(self):
        if self.map is None:
            return
        resolution = self.map['resolution']
        width = self.map['width']
        height = self.map['height']
        self.particles = []
        
        # Place one particle at the center of each map cell
        for row in range(height):
            for col in range(width):
                # Calculate center position of this cell in meters
                # Cell (col, row) has center at (col + 0.5) * resolution, (row + 0.5) * resolution
                x = (col + 0.5) * resolution
                y = (row + 0.5) * resolution
                
                # Create particle at center of cell
                particle = Particle(x, y, self.observation_history)
                self.particles.append(particle)
        
        # Update number of particles to match map size (one per cell)
        self.num_particles = width * height
        
        self.get_logger().info(f'Initialized {self.num_particles} particles (one per map cell) in {width}x{height} grid (resolution={resolution}m)')
    
    def clear_all_visualizations(self):
        clear_particles = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.frame_id = 'map'
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        clear_particles.markers.append(clear_marker)
        self.particles_pub.publish(clear_particles)
        clear_observations = MarkerArray()
        clear_marker_obs = Marker()
        clear_marker_obs.header.frame_id = 'map'
        clear_marker_obs.header.stamp = self.get_clock().now().to_msg()
        clear_marker_obs.action = Marker.DELETEALL
        clear_observations.markers.append(clear_marker_obs)
        self.observations_pub.publish(clear_observations)
        self.observation_history = []
        self.robot_path_x = 0.0
        self.robot_path_y = 0.0
        time.sleep(0.1)
        self.get_logger().info('Cleared all previous visualizations')
    
    def publish_map(self): #Called every 5 seconds by publish_map_timer
        # Need to publish the map using OccupancyGrid.
        publish_map(self.map, self.map_pub)
        self.get_logger().info(f'Published map')
    
    def publish_particles(self):
        if not self.particles:
            return
        marker_array = MarkerArray()
        for i, particle in enumerate(self.particles):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(particle.x)
            marker.pose.position.y = float(particle.y)
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA()
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            marker_array.markers.append(marker)
        self.particles_pub.publish(marker_array)
    
    def publish_observation_timeline(self):
        if not self.observation_history:
            return
        marker_array = MarkerArray()
        square_size = 0.15
        for i, obs_data in enumerate(self.observation_history):
            if len(obs_data) == 3:
                x, y, observation = obs_data
                observation = float(observation)
            else:
                self.get_logger().warn(f'Invalid observation data at index {i}: {obs_data}')
                continue
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = square_size
            marker.scale.y = square_size
            marker.scale.z = square_size
            marker.color = ColorRGBA()
            if observation < 0.5:
                marker.color.r = 0.2
                marker.color.g = 0.2
                marker.color.b = 0.2
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.8
            marker.color.a = 0.9
            marker_array.markers.append(marker)
        if len(self.observation_history) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = 'map'
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.id = 30000
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = 0.05
            line_marker.color.r = 0.5
            line_marker.color.g = 0.5
            line_marker.color.b = 1.0
            line_marker.color.a = 0.6
            for x, y, _ in self.observation_history:
                pt = Point()
                pt.x = float(x)
                pt.y = float(y)
                pt.z = 0.1
                line_marker.points.append(pt)
            marker_array.markers.append(line_marker)
        self.observations_pub.publish(marker_array)
    
    def compass_handler(self, msg: Float32):
        #self.get_logger().info(f'Received compass: heading={msg.data}')
        self.current_compass = msg.data
    
    def displacement_handler(self, msg: Point):
        # Handle incoming displacement and observation data.
        dx = msg.x
        dy = msg.y
        observation = float(msg.z)
        # Update particles based on displacement (transition model).
        # Remember to add random noise to each particle's movement independently.

        # Decide when to do resampling. Don't need to do it every time, but that might still work.
        displacement_magnitude = math.sqrt(dx * dx + dy * dy)
        
        if len(self.observation_history) < 5:
            self.get_logger().info(f'Received observation: {observation} (dx={dx:.3f}, dy={dy:.3f})')
        
        self.robot_path_x += dx
        self.robot_path_y += dy
        self.observation_history.append((self.robot_path_x, self.robot_path_y, observation))
        if len(self.observation_history) > self.max_history_length:
            self.observation_history.pop(0)
        
        if displacement_magnitude < MIN_MOVEMENT_FOR_ELIMINATION:
            return
        
        for i, particle in enumerate(self.particles):
            noise_angle = random.uniform(-MOTION_NOISE_ANGLE_DEG, MOTION_NOISE_ANGLE_DEG) * math.pi / 180.0
            cos_a = math.cos(noise_angle)
            sin_a = math.sin(noise_angle)
            noisy_dx = dx * cos_a - dy * sin_a
            noisy_dy = dx * sin_a + dy * cos_a
            scale = random.uniform(MOTION_SCALE_MIN, MOTION_SCALE_MAX)
            noisy_dx *= scale
            noisy_dy *= scale
            particle.x += noisy_dx
            particle.y += noisy_dy
            resolution = self.map['resolution']
            map_width_m = self.map['width'] * resolution
            map_height_m = self.map['height'] * resolution
            if particle.x < 0 or particle.x >= map_width_m or particle.y < 0 or particle.y >= map_height_m:
                particle.x = random.uniform(0, map_width_m)
                particle.y = random.uniform(0, map_height_m)
                particle.weight = particle.weight / 2.0
            
            particle.path.append((particle.x, particle.y))
            if len(particle.path) > self.path_history_length:
                particle.path = particle.path[-self.path_history_length:]
        self.update_weights()
        self.resample_particles()
        self.publish_estimated_pose()
    
    def update_weights(self):
        for particle in self.particles:
            if len(particle.path) > 0 and particle.observation_history and len(particle.observation_history) > 0:
                # Use particle's own path and shared observation history
                # Note: scorePath may need to match path length with observation history length
                path = particle.path
                observations = [obs[2] for obs in particle.observation_history[-len(path):]] if len(particle.observation_history) >= len(path) else [obs[2] for obs in particle.observation_history]
                if len(observations) == len(path):
                    score = scorePath(path, observations, self.map)
                    particle.weight += score
                else:
                    particle.weight = 0.0
            else:
                particle.weight = 0.0
    
    def resample_particles(self):
        if not self.particles:
            Error('Particles not initialized')
        if self.map is None:
            Error('Map is None')
        total_weight = sum(p.weight for p in self.particles)
        if total_weight <= 0:
            self.get_logger().warn('All particle weights <= 0, using uniform resampling')
            return
        resolution = self.map['resolution']
        map_width_m = self.map['width'] * resolution
        map_height_m = self.map['height'] * resolution
        cumulative = []
        cumsum = 0.0
        for particle in self.particles:
            cumsum += particle.weight
            cumulative.append(cumsum)
        new_particles = []
        random_particles = int(self.num_particles * RANDOM_RESAMPLING_PERCENTAGE)
        
        for i in range(self.num_particles):
            if i < random_particles:
                r = random.uniform(0, total_weight)
                selected_idx = 0
                for j, cum in enumerate(cumulative):
                    if r <= cum:
                        selected_idx = j
                        break
                old_particle = self.particles[selected_idx]
                noise_x = random.gauss(0.0, RESAMPLE_NOISE_STD_DEV)
                noise_y = random.gauss(0.0, RESAMPLE_NOISE_STD_DEV)
                new_x = old_particle.x + noise_x
                new_y = old_particle.y + noise_y
                # Clamp to map boundaries
                new_x = max(0.0, min(new_x, map_width_m - 1e-6))
                new_y = max(0.0, min(new_y, map_height_m - 1e-6))
                new_particle = Particle(new_x, new_y, self.observation_history)
                new_particle.x = new_x
                new_particle.y = new_y
                new_particle.weight = old_particle.weight
                # Copy path from old particle (or start fresh)
                new_particle.path = [(new_x, new_y)]
            else:
                new_x = random.uniform(0, map_width_m)
                new_y = random.uniform(0, map_height_m)
                new_particle = Particle(new_x, new_y, self.observation_history)
                new_particle.x = new_x
                new_particle.y = new_y
                new_particle.weight = 0.0
            new_particles.append(new_particle)
        self.particles = new_particles
        self.get_logger().info(f'Resampled {self.num_particles} particles')
    
    def publish_estimated_pose(self):
        if not self.particles:
            return
        total_weight = sum(p.weight for p in self.particles)
        if total_weight <= 0:
            avg_x = sum(p.x for p in self.particles) / len(self.particles)
            avg_y = sum(p.y for p in self.particles) / len(self.particles)
        else:
            avg_x = sum(p.x * p.weight for p in self.particles) / total_weight
            avg_y = sum(p.y * p.weight for p in self.particles) / total_weight
        pose_msg = Pose2D()
        pose_msg.x = float(avg_x)
        pose_msg.y = float(avg_y)
        pose_msg.theta = float(self.current_compass)
        self.pose_pub.publish(pose_msg)


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
