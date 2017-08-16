#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d.h>

#include "Costmap.h"

int main(int argc, char *argv[])
{
	// initialization
	ros::init(argc, argv, "Costmap");
	ros::NodeHandle nHandle("~");
	char* param_file;
	sprintf(param_file, "andy");
	Costmap *costmap = new Costmap(nHandle, param_file);

	// return the control to ROS
	ros::spin();

	return 0;
}