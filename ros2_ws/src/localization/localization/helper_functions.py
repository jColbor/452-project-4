import os
import yaml
from nav_msgs.msg import OccupancyGrid # OccupancyGrid to publish the map
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

    map_dict = {'data': [], 'resolution': 1.2}
    
    try:
        with open(full_map_path, 'r') as f:
            data = yaml.safe_load(f)
        
        resolution = data['resolution']
        map_dict['resolution'] = resolution
        map_text = data['map']
        
        map_lines = []
        for line in map_text.strip().split('\n'):
            stripped = line.strip()
            if stripped: 
                map_lines.append(stripped)
        
        # Reverse the lines so the first line is at the bottom (origin at bottom-left)
        map_lines.reverse()
        
        # Convert to 2D array: '#' = dark (1), '.' = light (0)
        for line in map_lines:
            row = [1 if char == '#' else 0 for char in line]
            if row:
                map_dict['data'].append(row)
        
        node.get_logger().info(f'Loaded map with resolution {resolution}m')
    
    except FileNotFoundError:
        node.get_logger().error(f'Map file not found: {full_map_path}')
    except Exception as e:
        node.get_logger().error(f'Error parsing map file: {str(e)}')
    
    return map_dict

    
def scorePoint(point, observation, map):
    if not map or 'data' not in map or not map['data'] or len(map['data']) == 0 or len(map['data'][0]) == 0:
        return -10000
    
    resolution = map.get('resolution', 1.2)
    map_data = map['data']
    
    col = int(point[0] / resolution)
    row = int(point[1] / resolution)
    
    if row < 0 or row >= len(map_data) or col < 0 or col >= len(map_data[0]):
        return -10000
    
    def cell_score(r, c):
        if r < 0 or r >= len(map_data) or c < 0 or c >= len(map_data[0]):
            return None 
        map_value = map_data[r][c]
        expected_observation = 1 - map_value
        error = abs(observation - expected_observation)
        return 1.0 - error
    
    center_score = cell_score(row, col)
    if center_score is None:
        return -10000
    
    adjacent_offsets = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),           (0, 1),
        (1, -1),  (1, 0),  (1, 1)
    ]
    
    adjacent_scores = []
    for dr, dc in adjacent_offsets:
        adj_score = cell_score(row + dr, col + dc)
        if adj_score is not None:
            adjacent_scores.append(adj_score)
    
    if center_score > 0.5:
        if adjacent_scores:
            avg_adjacent = sum(adjacent_scores) / len(adjacent_scores)
            final_score = 0.7 * center_score + 0.3 * avg_adjacent
        else:
            final_score = center_score
    else:
        if adjacent_scores:
            best_adjacent = max(adjacent_scores)
            final_score = max(center_score, best_adjacent * 0.8)
        else:
            final_score = center_score
    
    return final_score

def scorePath(points, observations, map):
    total_score = 0
    for i in range(len(points)):
        total_score += scorePoint(points[i], observations[i], map)
    return total_score

def publish_map(map, map_pub):
    map_msg = OccupancyGrid()
    
    map_msg.header.frame_id = 'world'
    
    if map and 'data' in map and map['data']:
        map_data = map['data']
        resolution = map.get('resolution', 1.2)
        
        map_msg.info.resolution = resolution
        map_msg.info.width = len(map_data[0]) if map_data else 0
        map_msg.info.height = len(map_data)
        
        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        flat_data = []
        for row in map_data:
            for cell in row:
                if cell == 1:  
                    flat_data.append(100)
                else:  
                    flat_data.append(0)
        
        map_msg.data = flat_data
    
    map_pub.publish(map_msg)