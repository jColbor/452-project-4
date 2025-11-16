# 452-project-4
Required outputs
You should construct a ROS system that publishes these outputs:

    On the topic /floor
    , a message of type nav_msgs/msg/OccupancyGrid
    correctly showing the map expressed in the given world file. You can check whether the map message is formed properly or not by visualizing it in rviz. This should be published every five seconds.
    On the topic /estimated_pose
    , a message of type geometry_msgs/msg/Pose2D
    providing your algorithm's best estimate of the robot's current pose. Your program should publish its single best estimate of the robot's pose accounting for all of the available information, even if that information is insufficient to be particularly sure of the correct state. This should be published every two seconds. 

You should make good choices about the architecture of your system — for example, possible division into multiple nodes, use of launch files, use of parameters, etc. — but (unlike, say, Project 3) there are no specific required elements in the architecture of your system apart from the required output topics. 
