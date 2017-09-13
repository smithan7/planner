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
	int test_environment_number = 3;
	int test_scenario_number = 0;
	int agent_index = 0;

	ros::param::get("test_environment_number", test_environment_number);
	ros::param::get("test_scenario_number", test_scenario_number);	
	ros::param::get("agent_index", agent_index);

	ROS_INFO("Distributed_Planner::initializing Planner");
	ROS_INFO("   test_environment_number %i", test_environment_number);
	ROS_INFO("   test_scenario_number %i", test_scenario_number);
	ROS_INFO("   agent_index %i", agent_index);


	
	Agent *agent = new Agent(nHandle, test_environment_number, test_scenario_number, agent_index);

	// return the control to ROS
	ros::spin();

	return 0;
}
