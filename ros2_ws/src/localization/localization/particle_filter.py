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
import threading
from concurrent.futures import ThreadPoolExecutor

NUM_PARTICLES = 1000
MIN_MOVEMENT_FOR_ELIMINATION = 0.1 # without this amount of movement no resampling nor assigning weights (deprecated)
RESAMPLE_NOISE_STD_DEV = 0.1 # deviation of particles
RANDOM_RESAMPLING_PERCENTAGE = 0.01 # percentage of completely random particles
MOTION_NOISE_ANGLE_DEG = 5.0  # Maximum angle noise in degrees for particle forward motion
MOTION_SCALE_MIN = 0.8  # Minimum scale factor for distance uncertainty
MOTION_SCALE_MAX = 1.2  # Maximum scale factor for distance uncertainty
PARTICLE_BASE_WEIGHT = 0.01 # Base/minimum weight for each particle
RESEEDED_WEIGHT_PENALTY = 0.2 # one-time penalty to weight of a randomly re-seeded particle. Without this, light.world and dark.world are broken by too many random particles.
MAX_HISTORY_LENGTH = 10 # Max length of observation & particle path history to keep for scoring
NOISY_RESAMPLE_POSITIONS = False # Whether to shift particle positions when resampling

class Particle:
    def __init__(self, initial_x, initial_y):
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.x = initial_x
        self.y = initial_y
        self.weight = PARTICLE_BASE_WEIGHT
        self.path = [(initial_x, initial_y)] 
    
    def __repr__(self):
        return f"Particle(x={self.x:.2f}, y={self.y:.2f}, weight={self.weight:.2f})"

class ParticleFilterNode(Node):
    def __init__(self, name: str = 'particle_filter'):
        super().__init__(name)
        self.current_compass = 0.0
        self.particles = []
        self.map = None
        self.num_particles = NUM_PARTICLES
        self.observation_history = []  # Shared observation history for all particles. Max length of MAX_HISTORY_LENGTH
        self.robot_path_x = 0.0
        self.robot_path_y = 0.0
        self.particle_lock = threading.Lock()
        self.executor = ThreadPoolExecutor(max_workers=8)
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
        
        # Calculate the number of particles to place for each unit distance
        area = width * height * (resolution ** 2)
        distance_step = math.sqrt(area / float(NUM_PARTICLES))
        particle_rows = int(height * resolution / distance_step)
        particle_cols = int(width * resolution / distance_step)
        
        y = distance_step / 2.0
        for row in range(particle_rows):
            x = distance_step / 2.0
            for col in range(particle_cols):
                # Calculate center position of this cell in meters
                # Cell (col, row) has center at (col + 0.5) * resolution, (row + 0.5) * resolution
                # x = (col + 0.5) * resolution
                # y = (row + 0.5) * resolution
                
                # Create particle at position
                particle = Particle(x, y)
                self.particles.append(particle)
                x += distance_step
            y += distance_step
        
        # Update number of particles to match map size (one per cell)
        self.num_particles = particle_rows * particle_cols
        
        self.get_logger().info(f'Initialized {self.num_particles} particles with distance step {distance_step:.5f} in {width}x{height} grid (resolution={resolution}m)')
    
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
        publish_map(self.map, self.map_pub) #This is from helper_functions.py
        self.get_logger().info(f'Published map')
    
    # Publish location of particles for visualization (not required output)
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
    
    # Required to publish pose best estimate. Computed with a weighted average of particle positions with their weights.
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
    
    def update_particle_position(self, particle, dx, dy):
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
        particle.path.append((particle.x, particle.y))
        if len(particle.path) > MAX_HISTORY_LENGTH:
            particle.path = particle.path[-MAX_HISTORY_LENGTH:]
        return particle
    
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
        if len(self.observation_history) > MAX_HISTORY_LENGTH:
            self.observation_history.pop(0)
        
        # Update all particles in parallel using thread pool
        with self.particle_lock:
            futures = []
            for particle in self.particles:
                future = self.executor.submit(self.update_particle_position, particle, dx, dy)
                futures.append(future)
            
            # Wait for all particle updates to complete
            for future in futures:
                future.result()
        
        # Only resample every MAX_HISTORY_LENGTH steps.
        if len(self.observation_history) >= MAX_HISTORY_LENGTH:
            self.update_weights()
            self.show_max_weight_particle_info()
            self.resample_particles()
            self.observation_history = [] # Clear observation history
        self.publish_estimated_pose()
    
    def update_weights(self):
        for particle in self.particles:
            if len(particle.path) > 0 and self.observation_history and len(self.observation_history) > 0:
                # Use particle's own path and shared observation history
                # Note: scorePath may need to match path length with observation history length
                path = particle.path
                # Get the most recent observations corresponding to the particle's path length.
                observations = [obs[2] for obs in self.observation_history[-len(path):]] if len(self.observation_history) >= len(path) else [obs[2] for obs in self.observation_history]
                if len(observations) == len(path):
                    if len(particle.path) != len(observations):
                        self.get_logger().warn(f'update weights: Particle path length is {len(particle.path)} < MAX_HISTORY_LENGTH.')
                    score = scorePath(path, observations, self.map) # Average score of all points in the particles path.
                    particle.weight += score # will be PARTICLE_BASE_WEIGHT + average of scores, along with -RESEEDED_WEIGHT_PENALTY if applicable
                    #self.get_logger().info(f'Particle scored with {len(path)} path points.')
                else:
                    particle.weight = 0.0
                    self.get_logger().warn(f'Particle path length {len(path)} does not match observations length {len(observations)}. Should not happen.')
            else:
                particle.weight = 0.0
    
    def resample_particles(self):
        if not self.particles:
            Error('Particles not initialized')
        if self.map is None:
            Error('Map is None')
        # total_weight = sum(p.weight for p in self.particles)
        # if total_weight <= 0:
        #     self.get_logger().warn('All particle weights <= 0, using uniform resampling') # This only happens if most particles don't match and many are out of bounds. Something is seriously wrong.
        #     return
        resolution = self.map['resolution']
        map_width_m = self.map['width'] * resolution
        map_height_m = self.map['height'] * resolution
        cumulative = []
        cumsum = 0.0
        for particle in self.particles:
            cumsum += max(particle.weight, 0.0) # Negative weight particles are allowed but treated as zero weight for resampling to avoid affecting particles after them in the list.
            cumulative.append(cumsum)
        new_particles = []
        random_particles = int(self.num_particles * RANDOM_RESAMPLING_PERCENTAGE) # Number of re-seeded particles
        
        for i in range(self.num_particles):
            if i > random_particles:
                # resampling particle close the high weight particles

                # Select a random particle based on their weights
                r = random.uniform(0, cumsum)
                selected_idx = 0
                for j, cum in enumerate(cumulative):
                    if r <= cum:
                        selected_idx = j
                        break
                old_particle = self.particles[selected_idx]

                new_x = old_particle.x
                new_y = old_particle.y
                if NOISY_RESAMPLE_POSITIONS:
                    # Generate some noise for the resampled particle. May be unnecessary?
                    noise_x = random.gauss(0.0, RESAMPLE_NOISE_STD_DEV)
                    noise_y = random.gauss(0.0, RESAMPLE_NOISE_STD_DEV)
                    new_x += noise_x
                    new_y += noise_y
                    # Clamp to map boundaries
                    new_x = max(0.0, min(new_x, map_width_m - 1e-6))
                    new_y = max(0.0, min(new_y, map_height_m - 1e-6))
                new_particle = Particle(new_x, new_y)
                new_particle.x = new_x
                new_particle.y = new_y
                # Copy initial positions so we can keep track of where the particle started.
                new_particle.initial_x = old_particle.initial_x
                new_particle.initial_y = old_particle.initial_y

                new_particle.weight = PARTICLE_BASE_WEIGHT
                # Copy path from old particle with last point updated to new position
                # new_particle.path = old_particle.path[:-1] + [(new_x, new_y)] if old_particle.path else [(new_x, new_y)]
                new_particle.path = [(new_x, new_y)]
            else:
                # completely random particles ("re-seeding")
                new_x = random.uniform(0, map_width_m)
                new_y = random.uniform(0, map_height_m)
                new_particle = Particle(new_x, new_y)
                new_particle.x = new_x
                new_particle.y = new_y
                new_particle.initial_x = -69.0 # Indicate re-seeded particle
                new_particle.initial_y = -69.0
                new_particle.weight = PARTICLE_BASE_WEIGHT - RESEEDED_WEIGHT_PENALTY # We want to favour existing particles over re-seeded ones because otherwise they'll break light.world and dark.world
            new_particles.append(new_particle)
        self.particles = new_particles
        self.get_logger().info(f'Resampled {self.num_particles} particles')

    def show_max_weight_particle_info(self):
        if not self.particles:
            return
        max_particle = max(self.particles, key=lambda p: p.weight)
        self.get_logger().info(f'Max weight particle: {max_particle}. \nPath length: {len(max_particle.path)} \nInitial position: ({max_particle.initial_x:.2f}, {max_particle.initial_y:.2f})')

def main():
    rclpy.init()
    node = ParticleFilterNode()
    try:
        node.initialize()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.executor.shutdown(wait=True)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
