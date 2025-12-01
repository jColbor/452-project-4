import os
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

    map = []
    #TODO: Use some YAML library to read the map file and extract it into an easy-to-use format.
    return map

    
def scorePoint(point, observation, map):
    # TODO: implement scoring function for a single point + observation.
    return 0

def scorePath(points, observations, map):
    total_score = 0
    for i in range(len(points)):
        total_score += scorePoint(points[i], observations[i], map)
    return total_score

def publish_map(map, map_pub):
    map_msg = OccupancyGrid()
    #code here
    map_pub.publish(map_msg)