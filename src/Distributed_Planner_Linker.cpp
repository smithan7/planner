#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d.h>

#include "Agent.h"

int main(int argc, char *argv[])
{
	// initialization
	ros::init(argc, argv, "Dist_Planner");
	ros::NodeHandle nHandle("~");
	std::string world_param_file = "/home/nvidia/catkin_ws/src/distributed_planner/agent_params.xml";
	ROS_INFO("Distributed_Planner::initializing Planner");
	Agent *agent = new Agent(nHandle, world_param_file);

	// return the control to ROS
	ros::spin();

	return 0;
}
