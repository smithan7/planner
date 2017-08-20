/*
 * Agent.h
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */

#ifndef SRC_Agent_H_
#define SRC_Agent_H_

// useful ROS libs
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h> // action server
#include <visualization_msgs/Marker.h> // for making marks in rviz
#include <costmap_2d/costmap_2d.h>
#include "geometry_msgs/PoseStamped.h"

// talk to the DJI Bridge
#include <custom_messages/DJI_Bridge_Status_MSG.h>

// talk to the planner
#include <custom_messages/Costmap_Bridge_Status_MSG.h>
#include <custom_messages/Costmap_Bridge_Travel_Path_MSG.h>

// talk to other planners
#include <custom_messages/Planner_Update_MSG.h>

// talk to everyone
#include <custom_messages/Planner_Status_MSG.h>

// opencv stuff
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


// normal c++ libs
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <utility>
#include <queue>
#include <fstream>
#include <math.h>


using namespace std;
using namespace cv;

class Goal;
class Agent_Coordinator;
class Agent_Planning;
class World;

class Agent{
public:

	Agent(ros::NodeHandle nh, std::string world_param_file);//, std::string param_file);
	~Agent();

	// take in other agents' updates and include them
	ros::Subscriber planner_update_subscriber;
	void planner_update_callback( const custom_messages::Planner_Update_MSG& updates );

	// proivde the rest of the team with my updates
	ros::Publisher planner_update_publisher;
	void publish_planner_updates();

	// subscribe to DJI_Bridge status
	ros::Subscriber quad_status_subscriber;
	void DJI_Bridge_status_callback( const custom_messages::DJI_Bridge_Status_MSG& status_in);

	// subscribe to Costmap_Bridge status
	ros::Subscriber costmap_status_subscriber;
	void Costmap_Bridge_status_callback( const custom_messages::Costmap_Bridge_Status_MSG& status_in);
	

	// publish wp path to Costmap Bridge
	ros::Publisher path_publisher;
	void publish_travel_path();


	// publish to the planner and dji_bridge
	ros::Publisher status_publisher;
	void publish_Agent_Status();

	// publish stuff to RVIZ
	ros::Publisher marker_publisher;
	void publish_rviz_path(const std::vector<Point2d> &path);
	void publishRvizMarker(const Point2d &loc, const double &radius, const int &color, const int &id);

	// status of the quad	
	bool travelling, waiting, emergency_stopped, planning;

	// status of the costmap
	bool locationInitialized, costmapInitialized;

	// for timing
	ros::Time act_time, plot_time, status_time;
	ros::Duration act_interval, plot_interval, status_interval;

	/********************************************************************/
	/*********************** Begin My Stuff *****************************/
	/********************************************************************/
	void get_prm_location(const cv::Point2d &gps_in, cv::Point2i &edge_out, double &progress);

	bool at_node(int node);
	void act();
	
	// access private variables
	int get_index() { return this->index; };
	cv::Point2d get_loc2d();
	int get_loc() { return this->edge.x; };

	Goal* get_goal() { return this->goal_node; };
	Agent_Coordinator* get_coordinator() { return this->coordinator; };
	Agent_Planning* get_planner() { return this->planner; };
	double get_arrival_time() { return this->arrival_time; };
	int get_type() { return this->type; };
	double get_travel_vel() { return this->travel_vel; };
	double get_travel_step() { return this->travel_step; };

	cv::Scalar get_color() { return this->color; };
	cv::Point get_edge() { return this->edge; };
	std::string get_task_selection_method() { return this->task_selection_method; };
	std::string get_task_claim_method() { return this->task_claim_method; };
	std::string get_task_claim_time() { return this->task_claim_time; };

	double get_work_done() { return this->work_done; };
	double get_travel_done() { return this->travel_done; };

private:
	// location in lat/lon
	cv::Point2d loc, goal, published_goal;
	std::vector< cv::Point2d > path;

	// planning and coordinator
	Goal* goal_node;
	Agent_Planning* planner;
	Agent_Coordinator* coordinator;
	World* world;

	std::string task_selection_method; // how do I select tasks
	std::string task_claim_method; // how / when do I claim tasks
	std::string task_claim_time; // when do I claim my task

	double arrival_time; // for travel and arrival plan
	double expected_value;

	double work_done; // accumulated reward
	double travel_done; // distance I have travelled

	cv::Point edge; // x:=where am I? y:=where am I going?
	double edge_progress; // how far along the edge am I?
	
	int index; // who am I in the world?
	int type; // what type of agent am I?
	double travel_vel; // how fast can I move?
	double travel_step; // how far do I move in one time step
	cv::Scalar color; // what color am I plotted?
	int n_tasks; // how many tasks are there

	// functions
	bool at_node(); // am I at a node, by edge_progress
	bool at_goal(); // am I at my goal node?
	
	// working and planning
	void work_on_task(); // work on the task I am at
	
	// moving and path planning 
	void move_along_edge(); // move along the current edge
	void select_next_edge(); // have a goal, select next edge to get to goal

	void find_path_and_publish(); // find path to goal and publish it to costmap_bridge
	void publish_travel_path(const std::vector<Point2d> &path);
	bool find_path( std::vector<cv::Point2d> &wp_path);
};

#endif /* SRC_Agent_H_ */
