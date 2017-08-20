/*
 * Agent.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */


#include "Agent.h"
#include "World.h"
#include "Map_Node.h"
#include "Agent_Coordinator.h"
#include "Agent_Planning.h"
#include "Goal.h"

using namespace std;
using namespace cv;

Agent::Agent(ros::NodeHandle nHandle, std::string world_param_file){
	
	this->world = new World(world_param_file);;	
	this->n_tasks =  this->world->get_n_nodes();

	// set these in param file
	this->index = index;
	this->type = type;
	this->travel_vel = travel_vel;
	this->travel_step = travel_vel *  this->world->get_dt();
	this->color = color;

	// initialize classes
	this->goal_node = new Goal();
	this->planner = new Agent_Planning(this, world);
	this->coordinator = new Agent_Coordinator(this, n_tasks);


	ROS_INFO("In");

	// initialize cell locs
	this->cell_loc= Point(-1,-1);
	this->cell_goal = Point(50,50);
	// initialize local locs
	this->local_loc = Point2d(-1.0, -1.0);
	this->local_goal = Point2d(50.0, 50.0);
	this->published_local_goal = Point2d(-1, -1);
	// set initial flags false
	this->locationInitialized = false;
	this->costmapInitialized = false;

	this->travelling = false;
	this->waiting = false;
	this->emergency_stopped = false;

	this->status_time = ros::Time::now(); // when did I last publish a status report
	this->status_interval = ros::Duration(1.0);
	this->act_time = ros::Time::now();
	this->act_interval = ros::Duration(5.0); // how often should I replan if I don't get an update or request
	this->plot_time = ros::Time::now(); // when did I last display the plot
	this->plot_interval = ros::Duration(1.0); // plot at 1 Hz

	/////////////////////// Subscribers /////////////////////////////
	// quad status callback
	this->quad_status_subscriber = nHandle.subscribe("/dji_bridge_status", 0, &Agent::DJI_Bridge_status_callback, this);
	// costmap_bridge status callback
	this->costmap_status_subscriber = nHandle.subscribe("/costmap_bridge_status", 0, &Agent::Costmap_Bridge_status_callback, this);

	/////////////////////// Publishers //////////////////////////////
	// tell the Costmap Bridge where I am going
	this->path_publisher =  nHandle.advertise<custom_messages::Costmap_Bridge_Travel_Path_MSG>("/wp_travel_path", 10);
	// tell everyone my status
	this->status_publisher = nHandle.advertise<custom_messages::Planner_Status_MSG>("/costmap_bridge_status", 10);
	// publish marker to RVIZ
	this->marker_publisher = nHandle.advertise<visualization_msgs::Marker>("/RVIZ", 10);
}

Agent::~Agent(){
	delete this->planner;
	delete this->coordinator;
	delete this->goal_node;
}


void Agent::DJI_Bridge_status_callback( const custom_messages::DJI_Bridge_Status_MSG& status_in){	
	// move these two to DJI_Bridge status callback, find out which node I am on
	cv::Point2d gps_in(status_in.longitude, status_in.latitude);
	this->get_prm_location(gps_in, this->edge, this->edge_progress);
	
	locationInitialized = true;

	if(ros::Time::now() - this->plot_time > this->plot_interval){
		this->plot_time = ros::Time::now();

		this->utils.build_prm_plot();
		Scalar blue = Scalar(255,0,0);
		this->utils.add_agent_to_prm_plot( blue, this->path, this->loc);
		Scalar orange = Scalar(0,165,255);
		this->utils.add_agent_to_prm_plot( orange, this->path, this->goal);
		this->utils.display_prm_plot();
	}

	if(ros::Time::now() - this->status_time > this->status_interval){
		this->status_time = ros::Time::now();
		publish_Agent_Status();
	}
	this->act();
}

void Agent::act() {
	// am  I at a node?
	if (this->at_node()) {
		// I am at a node, am I at my goal and is it active still?
		if (this->at_goal() &&  this->world->get_nodes()[this->goal_node->get_index()]->is_active()) {
			this->work_done +=  this->world->get_nodes()[this->goal_node->get_index()]->get_acted_upon(this); // work on my goal
		}
		else {
			this->planner->plan(); // I am not at my goal, select new goal
			this->coordinator->advertise_task_claim(this->world); // select the next edge on the path to goal 
			this->select_next_edge(); // plan the next edge
			this->find_path_and_publish(); // on the right edge, move along edge
		}
	}
	else { // not at a node
		this->find_path_and_publish(); // move along edge
	}
}

void Agent::get_prm_location(const cv::Point2d &gps_in, cv::Point2i &edge_out, double &progress){
	// go through the PRM and find the 2 nodes I am closest to. edge_out.x = closest, edge_out.y = 2nd closest. progress is (dist to x)/(dist between x and y)
	this->world->get_prm_location(gps_in, edge_out, progress);
}

void Agent::Costmap_Bridge_status_callback( const custom_messages::Costmap_Bridge_Status_MSG& status_in){	
	// I don't know that I need anything from them ;(
}

void Agent::find_path_and_publish(){
	if( ros::Time::now() - this->actTimer > this->act_interval || this->published_local_goal != this->local_goal ){
		this->published_local_goal = this->local_goal;
		this->actTimer = ros::Time::now();

		bool flag = false;
		if( this->locationInitialized ){
			if( this->costmapInitialized ){
				flag = true;
			}
			else{
				ROS_WARN("Agent::act::waiting on costmap callback");
			}
		}
		else{
			ROS_WARN("Agent::act::waiting on location callback");
		}

        if( flag || true ){
        	ROS_INFO("Agent::act::publishing path to costmap_bridge");
			this->wp_path.clear();
			ROS_WARN("Fake Goal");
			this->wp_path.push_back(this->goal); // this needs to be ERASED for trials
        	if(this->find_path(this->wp_path)){
        		this->publish_travel_path(this->wp_path);	
        	}
		}
	}
}

void Agent::publish_travel_path(const std::vector<Point2d> &path){
	// initialize msg
	custom_messages::Costmap_Bridge_Travel_Path_MSG path_msg;
	// for length of path
	for(size_t i=0; i<path.size(); i++){
		// go from cells to loc_x, loc_y
		// set path
		path_msg.longitudes.push_back(double(path[i].x));
		path_msg.latitudes.push_back(double(path[i].y));
	}
	this->path_publisher.publish(path_msg);
}

bool Agent::find_path( std::vector<cv::Point2d> &wp_path ){
	ROS_WARN("Agent::find_path::TODO complete");
	// ensure starting fresh
	wp_path.clear();

	

	return true;
}

void Agent::publish_Agent_Status(){
	custom_messages::Planner_Status_MSG msg;
	msg.longitude = this->local_loc.x;
	msg.latitude = this->local_loc.y;
	msg.altitude = this->set_alt;
	msg.heading = 0.0;
	msg.goal_longitude = this->local_goal.x;
	msg.goal_latitude = this->local_goal.y;

	msg.travelling = this->travelling;
	msg.emergency_stopped = this->emergency_stopped;
	msg.waiting = this->waiting;
	this->status_publisher.publish(msg);
}


bool Agent::at_node(int node) {
	ROS_WARN("Agent::at_node::TODO make distance based");
	if (this->edge_progress >= 0.9 && this->edge.y == node) {
		return true;
	}
	else if (this->edge_progress <= 0.1 && this->edge.x == node) {
		return true;
	}
	else {
		return false;
	}
}

bool Agent::at_node() { // am I at a node, by edge progress?
	ROS_WARN("Agent::at_node::TODO make distance based");
	if (this->edge_progress <= 0.1 || this->edge_progress >= 0.9) {
		return true;
	}
	else {
		return false;
	}
}

bool Agent::at_goal() { // am I at my goal node?
	ROS_WARN("Agent::at_goal::TODO make distance based");
	if (this->edge_progress >= 0.9 && this->edge.y == this->goal_node->get_index()) {
		return true;
	}
	else if (this->edge_progress =< 0.1 && this->edge.x == this->goal_node->get_index()) {
		return true;
	}
	else{
		return false;
	}
}



