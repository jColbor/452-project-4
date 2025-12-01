import os
import yaml
from nav_msgs.msg import OccupancyGrid # OccupancyGrid to publish the map
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node

bag_num_to_map_file = {
    '11': 'light.world',
    '12': 'dark.world',
    '13': 'windy.world',
    '14': 'windy.world',
    '15': 'code.world', 
    '16': 'code.world',
    '17': 'cave.world'
    #TODO: UPDATE THIS FOR FINAL 3 MAPS WHEN PUBLISHED
}
def extract_map(node):
    # Get the parameter value.
    bag_parameter_name = "bag_in"
    data_folder_name = "project4-data"

    node.declare_parameter(bag_parameter_name, "")
    value = node.get_parameter(bag_parameter_name).get_parameter_value().string_value

    if value == '':
        raise ValueError(f'No value given for parameter: {bag_parameter_name}')

    # Extract bag number from filename by splitting after data_folder_name/
    if(data_folder_name + os.sep in value): #for paths like "../project4-data/11/"
        base_path, bag_num = value.split(data_folder_name + os.sep)
        base_path += data_folder_name + os.sep
    elif value.startswith('./'): # for paths like "./11/"
        bag_num = value[2:]
        base_path = './'
        base_path = bag_num.split('./')[0]
    else: # for paths like "11/"
        bag_num = value
        base_path = './' # Current directory
    # Remove trailing slash if present
    if bag_num[-1] == '/':
        bag_num = bag_num[:-1]
    
    if bag_num not in bag_num_to_map_file:
        raise ValueError(f'No map file found for bag number: {bag_num}')
    map_file = bag_num_to_map_file[bag_num]
    full_map_path = os.path.join(base_path, map_file)
    node.get_logger().info(f'Extracted map file path: {full_map_path}')

    # Parse YAML file
    try:
        with open(full_map_path, 'r') as f:
            map_data = yaml.safe_load(f)
    except FileNotFoundError:
        raise FileNotFoundError(f'Map file not found: {full_map_path}')
    except Exception as e:
        raise ValueError(f'Error parsing map file: {e}')
    
    # Extract resolution and map string
    resolution = map_data.get('resolution', 1.0)
    map_string = map_data.get('map', '')
    
    # Parse map string into 2D array
    # Map is stored as rows, where each row is a string
    # # = dark (1), . = light (0)
    # Origin is bottom-left, so we need to reverse rows
    map_lines = map_string.strip().split('\n')
    map_2d = []
    
    for line in map_lines:
        line = line.strip()
        if not line:  # Skip empty lines
            continue
        row = []
        for char in line:
            if char == '#':
                row.append(1)  # Dark
            elif char == '.':
                row.append(0)  # Light
            elif char == ' ':  # Skip spaces
                continue
            else:
                row.append(-1)  # Unknown
        if row:  # Only add non-empty rows
            map_2d.append(row)
    
    # Reverse rows so origin is at bottom-left (first row is bottom)
    map_2d.reverse()
    
    # Return map as dict with 2D array and resolution
    map_dict = {
        'data': map_2d,
        'resolution': resolution,
        'width': len(map_2d[0]) if map_2d else 0,
        'height': len(map_2d)
    }
    
    node.get_logger().info(f'Loaded map: {map_dict["width"]}x{map_dict["height"]}, resolution={resolution}m')
    return map_dict

    
def scorePoint(point, observation, map_dict):
    if map_dict is None or 'data' not in map_dict:
        return -10.0
    
    map_data = map_dict['data']
    resolution = map_dict['resolution']
    width = map_dict['width']
    height = map_dict['height']
    
    col = int(point[0] / resolution)
    row = int(point[1] / resolution)
    
    if row < 0 or row >= height or col < 0 or col >= width:
        return -10.0
    
    map_value = map_data[row][col]
    
    if map_value == 0:
        expected_light = 1.0
        if observation > 0.5:
            match_score = 1.0
        else:
            match_score = 1.0 - abs(observation - 0.5) * 2.0
    else:
        expected_light = 0.0
        if observation < 0.5:
            match_score = 1.0
        else:
            match_score = 1.0 - abs(observation - 0.5) * 2.0
    
    adjacent_bonus = 0.0
    for dr in [-1, 0, 1]:
        for dc in [-1, 0, 1]:
            if dr == 0 and dc == 0:
                continue
            r = row + dr
            c = col + dc
            if 0 <= r < height and 0 <= c < width:
                adj_value = map_data[r][c]
                if adj_value == 0:
                    adj_expected = 1.0
                    if observation > 0.5:
                        adj_match = 1.0
                    else:
                        adj_match = 1.0 - abs(observation - 0.5) * 2.0
                else:
                    adj_expected = 0.0
                    if observation < 0.5:
                        adj_match = 1.0
                    else:
                        adj_match = 1.0 - abs(observation - 0.5) * 2.0
                adjacent_bonus += adj_match * 0.1
    
    return match_score * 5.0 + adjacent_bonus

def scorePath(points, observations, map_dict):
    if len(points) != len(observations):
        return -10.0
    
    if len(points) == 0:
        return 0.0
    
    total_score = 0.0
    for i in range(len(points)):
        total_score += scorePoint(points[i], observations[i], map_dict)
    
    return total_score / len(points)

def publish_map(map_dict, map_pub):
    """
    Convert map dictionary to OccupancyGrid message and publish.
    map_dict: dict with 'data' (2D array), 'resolution', 'width', 'height'
    map_pub: ROS publisher for OccupancyGrid
    """
    if map_dict is None or 'data' not in map_dict:
        return
    
    map_data = map_dict['data']
    resolution = map_dict['resolution']
    width = map_dict['width']
    height = map_dict['height']
    
    map_msg = OccupancyGrid()
    
    # Header
    map_msg.header = Header()
    map_msg.header.frame_id = 'map'
    map_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    
    # Map metadata
    map_msg.info.resolution = resolution
    map_msg.info.width = width
    map_msg.info.height = height
    
    # Origin: bottom-left corner at (0, 0)
    map_msg.info.origin.position.x = 0.0
    map_msg.info.origin.position.y = 0.0
    map_msg.info.origin.position.z = 0.0
    map_msg.info.origin.orientation.w = 1.0
    
    # Convert 2D map to 1D array (row-major order, bottom to top)
    # OccupancyGrid uses 0-100: 0=free, 100=occupied, -1=unknown
    # Our map: 0=light (free), 1=dark (occupied)
    map_msg.data = []
    for row in map_data:
        for cell in row:
            if cell == 1:  # Dark = occupied
                map_msg.data.append(100)
            elif cell == 0:  # Light = free
                map_msg.data.append(0)
            else:  # Unknown
                map_msg.data.append(-1)
    
    map_pub.publish(map_msg)