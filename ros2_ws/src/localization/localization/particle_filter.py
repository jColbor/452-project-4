import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32
from geometry_msgs.msg import Point, Pose2D
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from localization.helper_functions import extract_map, scorePoint, scorePath, publish_map
import math
import random
import time

NUM_PARTICLES = 300
ELIMINATE_LOW_WEIGHT_PARTICLES = True
ELIMINATION_WEIGHT_THRESHOLD = -300.0
ELIMINATION_PERCENTAGE = 0.1
MIN_PARTICLES_AFTER_ELIMINATION = 150
MIN_MOVEMENT_FOR_ELIMINATION = 0.1
JITTER_STD_DEV = 0.02
RESAMPLE_NOISE_STD_DEV = 0.1
WEIGHT_PENALTY_CONSTANT = 10.0
WEIGHT_PENALTY_PERCENTAGE = 0.3
FULLY_RANDOM_RESAMPLING_PERCENTAGE = 0.97

class Particle:
    def __init__(self, initial_x, initial_y):
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.x = initial_x
        self.y = initial_y
        self.weight = 0.0
    
    def __repr__(self):
        return f"Particle(x={self.x:.2f}, y={self.y:.2f}, weight={self.weight:.2f})"

class ParticleFilterNode(Node):
    def __init__(self, name: str = 'particle_filter'):
        super().__init__(name)
        self.current_compass = 0.0
        self.particles = []
        self.map = None
        self.num_particles = NUM_PARTICLES
        self.particle_paths = {}
        self.particle_observations = {}
        self.path_history_length = 10
        self.observation_history = []
        self.max_history_length = 500
        self.robot_path_x = 0.0
        self.robot_path_y = 0.0
        self.get_logger().info('ParticleFilterNode constructed')

    def initialize(self):
        self.displacement_sub = self.create_subscription(Point, '/dispatch_out', self.displacement_handler, 10)
        self.compass_sub = self.create_subscription(Float32, '/compass', self.compass_handler, 10)
        self.publish_map_timer = self.create_timer(5.0, self.publish_map)
        self.publish_particles_timer = self.create_timer(0.5, self.publish_particles)
        self.publish_observations_timer = self.create_timer(0.5, self.publish_observation_timeline)
        self.jitter_timer = self.create_timer(0.1, self.apply_jitter)
        self.map_pub = self.create_publisher(OccupancyGrid, '/floor', 10)
        self.pose_pub = self.create_publisher(Pose2D, '/estimated_pose', 10)
        self.particles_pub = self.create_publisher(MarkerArray, '/particles', 10)
        self.observations_pub = self.create_publisher(MarkerArray, '/observation_timeline', 10)
        self.map = extract_map(self)
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
        map_width_m = width * resolution
        map_height_m = height * resolution
        self.particles = []
        for i in range(self.num_particles):
            x = random.uniform(0, map_width_m)
            y = random.uniform(0, map_height_m)
            particle = Particle(x, y)
            self.particles.append(particle)
            self.particle_paths[i] = [(x, y)]
            self.particle_observations[i] = []
        self.get_logger().info(f'Initialized {self.num_particles} particles across map ({map_width_m:.2f}m x {map_height_m:.2f}m)')
    
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
    
    def publish_map(self):
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
    
    def apply_jitter(self):
        if not self.particles or self.map is None:
            return
        resolution = self.map['resolution']
        map_width_m = self.map['width'] * resolution
        map_height_m = self.map['height'] * resolution
        for particle in self.particles:
            jitter_x = random.gauss(0.0, JITTER_STD_DEV)
            jitter_y = random.gauss(0.0, JITTER_STD_DEV)
            new_x = particle.x + jitter_x
            new_y = particle.y + jitter_y
            if 0 <= new_x < map_width_m and 0 <= new_y < map_height_m:
                particle.x = new_x
                particle.y = new_y
    
    def compass_handler(self, msg: Float32):
        self.current_compass = msg.data
    
    def displacement_handler(self, msg: Point):
        dx = msg.x
        dy = msg.y
        observation = float(msg.z)
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
            noise_angle = random.uniform(-15.0, 15.0) * math.pi / 180.0
            cos_a = math.cos(noise_angle)
            sin_a = math.sin(noise_angle)
            noisy_dx = dx * cos_a - dy * sin_a
            noisy_dy = dx * sin_a + dy * cos_a
            scale = random.uniform(0.8, 1.2)
            noisy_dx *= scale
            noisy_dy *= scale
            offset_magnitude = 0.05
            noisy_dx += random.uniform(-offset_magnitude, offset_magnitude)
            noisy_dy += random.uniform(-offset_magnitude, offset_magnitude)
            particle.x += noisy_dx
            particle.y += noisy_dy
            resolution = self.map['resolution']
            map_width_m = self.map['width'] * resolution
            map_height_m = self.map['height'] * resolution
            if particle.x < 0 or particle.x >= map_width_m or particle.y < 0 or particle.y >= map_height_m:
                particle.x = random.uniform(0, map_width_m)
                particle.y = random.uniform(0, map_height_m)
                particle.weight = 0.0
                self.particle_paths[i] = [(particle.x, particle.y)]
                self.particle_observations[i] = []
            else:
                map_data = self.map['data']
                resolution = self.map['resolution']
                col = int(particle.x / resolution)
                row = int(particle.y / resolution)
                if 0 <= row < self.map['height'] and 0 <= col < self.map['width']:
                    map_value = map_data[row][col]
                    obs_binary = 1 if observation > 0.5 else 0
                    if map_value == (1 - obs_binary):
                        particle.weight += 50.0
                    else:
                        penalty = WEIGHT_PENALTY_CONSTANT + abs(particle.weight) * WEIGHT_PENALTY_PERCENTAGE
                        particle.weight -= penalty
                        particle.weight = max(particle.weight, -200.0)
            if i not in self.particle_paths:
                self.particle_paths[i] = []
            if i not in self.particle_observations:
                self.particle_observations[i] = []
            self.particle_paths[i].append((particle.x, particle.y))
            self.particle_observations[i].append(observation)
            if len(self.particle_paths[i]) > self.path_history_length:
                self.particle_paths[i] = self.particle_paths[i][-self.path_history_length:]
                self.particle_observations[i] = self.particle_observations[i][-self.path_history_length:]
        self.update_weights()
        if ELIMINATE_LOW_WEIGHT_PARTICLES:
            self.eliminate_low_weight_particles()
        self.resample_particles()
        self.publish_estimated_pose()
    
    def eliminate_low_weight_particles(self):
        if not self.particles or len(self.particles) < MIN_PARTICLES_AFTER_ELIMINATION:
            return
        if self.map is None:
            return
        resolution = self.map['resolution']
        map_width_m = self.map['width'] * resolution
        map_height_m = self.map['height'] * resolution
        
        num_to_eliminate = int(len(self.particles) * ELIMINATION_PERCENTAGE)
        min_particles = max(MIN_PARTICLES_AFTER_ELIMINATION, int(self.num_particles * 0.5))
        num_to_eliminate = min(num_to_eliminate, len(self.particles) - min_particles)
        if num_to_eliminate <= 0:
            return
        
        min_weight = min(p.weight for p in self.particles)
        max_weight = max(p.weight for p in self.particles)
        weight_range = max_weight - min_weight
        if weight_range < 0.001:
            weight_range = 1.0
        
        inverse_weights = []
        for particle in self.particles:
            normalized_weight = (particle.weight - min_weight) / weight_range
            inverse_weight = 1.0 - normalized_weight + 0.1
            inverse_weights.append(inverse_weight)
        
        total_inverse = sum(inverse_weights)
        if total_inverse <= 0:
            return
        
        cumulative = []
        cumsum = 0.0
        for inv_w in inverse_weights:
            cumsum += inv_w
            cumulative.append(cumsum)
        
        eliminated_indices = set()
        attempts = 0
        max_attempts = num_to_eliminate * 10
        while len(eliminated_indices) < num_to_eliminate and attempts < max_attempts:
            r = random.uniform(0, total_inverse)
            selected_idx = 0
            for j, cum in enumerate(cumulative):
                if r <= cum:
                    selected_idx = j
                    break
            eliminated_indices.add(selected_idx)
            attempts += 1
        
        for i in eliminated_indices:
            particle = self.particles[i]
            particle.x = random.uniform(0, map_width_m)
            particle.y = random.uniform(0, map_height_m)
            particle.weight = 0.0
            self.particle_paths[i] = [(particle.x, particle.y)]
            self.particle_observations[i] = []
        
        if len(eliminated_indices) > 0:
            self.get_logger().debug(f'Eliminated and resampled {len(eliminated_indices)} particles')
    
    def update_weights(self):
        for i, particle in enumerate(self.particles):
            if i in self.particle_paths and len(self.particle_paths[i]) > 0:
                path = self.particle_paths[i]
                observations = self.particle_observations[i]
                score = scorePath(path, observations, self.map)
                particle.weight += score
            else:
                particle.weight = 0.0
    
    def resample_particles(self):
        if not self.particles:
            return
        if self.map is None:
            return
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
        new_paths = {}
        new_observations = {}
        half_particles = int(self.num_particles * FULLY_RANDOM_RESAMPLING_PERCENTAGE)
        
        for i in range(self.num_particles):
            if i < half_particles:
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
                if new_x < 0 or new_x >= map_width_m or new_y < 0 or new_y >= map_height_m:
                    new_x = old_particle.x
                    new_y = old_particle.y
                new_particle = Particle(new_x, new_y)
                new_particle.x = new_x
                new_particle.y = new_y
                new_particle.weight = old_particle.weight
            else:
                new_x = random.uniform(0, map_width_m)
                new_y = random.uniform(0, map_height_m)
                new_particle = Particle(new_x, new_y)
                new_particle.x = new_x
                new_particle.y = new_y
                new_particle.weight = 0.0
            new_particles.append(new_particle)
            new_paths[i] = [(new_particle.x, new_particle.y)]
            new_observations[i] = []
        self.particles = new_particles
        self.particle_paths = new_paths
        self.particle_observations = new_observations
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
