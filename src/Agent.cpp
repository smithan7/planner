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

Agent::Agent(ros::NodeHandle nHandle, const int &test_environment_number, const int &test_scenario_number, const int &agent_index){
	ROS_INFO("Agent::Agent::initializing planner");
	ROS_INFO("     test_environment_number: %i", test_environment_number);
	ROS_INFO("     test_scenario_number: %i", test_scenario_number);
	ROS_INFO("     agent_index: %i", agent_index);
		
	// load from param file
	if(!this->load_agent_params(agent_index)){
		ROS_ERROR("Dist Planner::Agent::init::could not initialize agent");
	}

	this->world = new World();
	if(!this->world->init(test_environment_number, test_scenario_number)){
		ROS_ERROR("Dist Planner::Agent::init::World failed to initialize");		
	}
	ROS_INFO("Dist Planner::Agent::init::World initialized");
	this->n_tasks =  this->world->get_n_nodes();
	this->travel_step = this->travel_vel *  this->world->get_dt();

	// initialize classes
	this->goal_node = new Goal();
	this->planner = new Agent_Planning(this, world);
	this->coordinator = new Agent_Coordinator(this, this->n_tasks);

	// initialize local locs
	this->loc = Point2d(-1.0, -1.0);
	this->goal = Point2d(50.0, 50.0);
	this->published_goal = Point2d(-1, -1);
	// set initial flags false
	this->locationInitialized = false;
	this->costmapInitialized = false;

	this->travelling = false;
	this->waiting = false;
	this->emergency_stopped = false;
	this->planning = false;

	this->status_time = ros::Time::now(); // when did I last publish a status report
	this->status_interval = ros::Duration(1.0);
	this->act_time = ros::Time::now();
	this->act_interval = ros::Duration(1.0); // how often should I replan if I don't get an update or request
	this->plot_time = ros::Time::now(); // when did I last display the plot
	this->plot_interval = ros::Duration(1.0); // plot at 1 Hz

	this->start_time = ros::Time::now();
	this->world->set_c_time(this->start_time.toSec());
	ROS_ERROR("start time: %.2f", this->start_time.toSec());

	/////////////////////// Subscribers /////////////////////////////
	// quad status callback, get position
	this->quad_status_subscriber = nHandle.subscribe("/dji_bridge_status", 1, &Agent::DJI_Bridge_status_callback, this);
	// costmap_bridge status callback, currently does nothing
	this->costmap_status_subscriber = nHandle.subscribe("/costmap_bridge_status", 1, &Agent::Costmap_Bridge_status_callback, this);

	// hear everyone else's plans
	this->planner_update_subscriber = nHandle.subscribe("/agent_plans", 10, &Agent::planner_update_callback, this);
	// hear everyone else's status and completed tasks
	this->planner_status_subscriber = nHandle.subscribe("/dist_planner_status", 10, &Agent::planner_status_callback, this);


	/////////////////////// Publishers //////////////////////////////
	// tell the Costmap Bridge where I am going
	this->path_publisher =  nHandle.advertise<custom_messages::Costmap_Bridge_Travel_Path_MSG>("/wp_travel_path", 10);
	// tell everyone my status
	this->status_publisher = nHandle.advertise<custom_messages::Planner_Status_MSG>("/dist_planner_status", 10);
	// publish plan to everyone
	this->plan_publisher = nHandle.advertise<custom_messages::Planner_Update_MSG>("/agent_plans", 1);	
	// publish marker to RVIZ
	this->marker_publisher = nHandle.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
}

bool Agent::load_agent_params(const int &agent_index){
	this->index = agent_index;
	char agent_file[200];
	sprintf(agent_file, "/home/nvidia/catkin_ws/src/distributed_planner/params/agent%i_params.xml", agent_index);
	//sprintf(agent_file, "/home/andy/catkin_ws/src/distributed_planner/params/agent%i_params.xml", agent_index);
    cv::FileStorage f_agent(agent_file, cv::FileStorage::READ);
    if (!f_agent.isOpened()){
        ROS_ERROR("Dist planner::Agent::init::Failed to open %s", agent_file);
        return false;
    }
	ROS_INFO("Dist planner::Agent::init::Opened: %s", agent_file);
    
	f_agent["type"] >> this->type;
	this->color = cv::Scalar(0,0,255);
	
	f_agent["travel_vel"] >> this->travel_vel;
	this->travel_step = 0.2 * this->travel_vel;
	f_agent["flight_altitude"] >> this->flight_altitude;
	f_agent["pay_obstacle_cost"] >> this->pay_obstacle_costs;
	f_agent["task_selection_method"] >> this->task_selection_method;
	f_agent["task_claim_method"] >> this->task_claim_method;
	f_agent["task_claim_time"] >> this->task_claim_time;
	f_agent["obs_radius"] >> this->obs_radius;
	
	f_agent.release();
	return true;
}


Agent::~Agent(){
	delete this->planner;
	delete this->coordinator;
	delete this->goal_node;
}

void Agent::planner_update_callback( const custom_messages::Planner_Update_MSG& msg ){
	if(this->index != msg.agent_index){
		// make sure it isn't me
		this->world->add_stop_to_agents_path(msg.agent_index, msg.node_index, msg.probability, msg.time);
	}
}


void Agent::DJI_Bridge_status_callback( const custom_messages::DJI_Bridge_Status_MSG& status_in){
	this->world->set_c_time(ros::Time::now().toSec());
	// move these two to DJI_Bridge status callback, find out which node I am on
	//this->loc = cv::Point2d(status_in.local_x, status_in.local_y);
	//ROS_WARN("status_in loc: %0.6f, %0.6f", status_in.longitude, status_in.latitude);
	this->lonlat = cv::Point2d(status_in.longitude, status_in.latitude);
	this->loc = this->world->global_to_local(this->lonlat);
	this->world->get_prm_location(this->loc, this->edge, this->edge_progress, this->obs_radius, this->visiting_nodes);
	//ROS_WARN("loc: %.2f, %.2f", this->loc.x, this->loc.y);
	//ROS_INFO("prm loc: edge: %i, %i", this->edge.x, this->edge.y);
	//ROS_INFO("node loc: %.2f, %.2f", this->world->get_nodes()[this->edge.x]->get_local_loc().x, this->world->get_nodes()[this->edge.x]->get_local_loc().y);
	locationInitialized = true;

	if(ros::Time::now() - this->plot_time > this->plot_interval){
		this->plot_time = ros::Time::now();
		this->world->display_world(this);
	}

	if(ros::Time::now() - this->status_time > this->status_interval){
		this->status_time = ros::Time::now();
		publish_Agent_Status();
	}

	this->act();
}

void Agent::act() {
	//ROS_INFO("in act");
	// attempt to complete tasks
	this->do_work();
	this->planner->plan(); // select goal
	//ROS_INFO("advertise");
	this->coordinator->advertise_task_claim(this->world); // select the next edge on the path to goal 
	//ROS_INFO("path and publish");
	this->find_path_and_publish(); // on the right edge, move along edge

	custom_messages::Planner_Update_MSG msg;
	if(this->coordinator->get_plan(msg)){
		this->publish_plan(msg);
		this->coordinator->reset_prob_actions();
	}
}

void Agent::publish_plan(const custom_messages::Planner_Update_MSG &msg){
	plan_publisher.publish(msg);
}

void Agent::do_work(){
	// I am at a node, if active, work on it it
	for(size_t i=0; i<this->visiting_nodes.size(); i++){
		if (this->world->get_nodes()[visiting_nodes[i]]->is_active()) {
			this->work_done +=  this->world->get_nodes()[this->visiting_nodes[i]]->get_acted_upon(this); // work on my goal
		}
	}
}

void Agent::Costmap_Bridge_status_callback( const custom_messages::Costmap_Bridge_Status_MSG& status_in){	
	// I don't know that I need anything from them ;(
}

void Agent::find_path_and_publish(){
	if( ros::Time::now() - this->act_time > this->act_interval || this->published_goal != this->goal ){
		this->published_goal = this->goal;
		this->act_time = ros::Time::now();

		bool flag = false;
		if( this->locationInitialized ){
			flag = true;
		}
		else{
			ROS_WARN("Agent::act::waiting on location callback");
		}

        if( flag ){
        	this->path.clear();
        	if(this->find_path(this->path)){
				ROS_WARN("Dist_Planner::Agent::find_path_and_publish::path length: %i", int(this->path.size()));
        		this->publish_travel_path_to_costmap(this->path);
				this->publish_rviz_path(this->path);
				this->publish_plan();
        	}
		}
	}
}

void Agent::publish_travel_path_to_costmap(const std::vector<Point2d> &path){
	// initialize msg
	custom_messages::Costmap_Bridge_Travel_Path_MSG path_msg;
	// for length of path
	for(size_t i=0; i<path.size(); i++){
		// go from cells to loc_x, loc_y
		// set path
		path_msg.local_xs.push_back(double(path[i].x));
		path_msg.local_ys.push_back(double(path[i].y));
	}
	path_msg.altitude = this->flight_altitude;
	this->path_publisher.publish(path_msg);
}

void Agent::publish_plan(){
	custom_messages::Planner_Update_MSG msg;
	msg.agent_index = this->index;

	msg.node_index.push_back(this->goal_node->get_index());
	ROS_WARN("Agent::publish_plan::assuming greedy selection");	
	msg.probability.push_back(1.0);
	msg.time.push_back(this->goal_node->get_completion_time());
	plan_publisher.publish(msg);
}


bool Agent::find_path( std::vector<cv::Point2d> &wp_path ){
	// ensure starting fresh
	wp_path.clear();
	std::vector<int> path;
	double length = 0;
	if(this->world->a_star(this->edge.x, this->goal_node->get_index(), path, length)){
		for(size_t i=path.size()-1; i>0; i--){
			wp_path.push_back(this->world->get_nodes()[path[i]]->get_local_loc());
		}
		wp_path.push_back(this->world->get_nodes()[this->goal_node->get_index()]->get_local_loc());
		return true;
	}
	return false;
}

void Agent::publish_Agent_Status(){
	custom_messages::Planner_Status_MSG msg;
	for(int i=0; i<this->world->get_n_nodes(); i++){
		if(!this->world->get_nodes()[i]->is_active()){
			msg.completed_tasks.push_back(i);
		}
	}
	msg.longitude = this->lonlat.x;
	msg.latitude = this->lonlat.y;
	msg.goal_longitude = this->goal.x;
	msg.goal_latitude = this->goal.y;
	msg.index = this->index;

	msg.travelling = this->travelling;
	msg.emergency_stopped = this->emergency_stopped;
	msg.waiting = this->waiting;
	msg.planning = this->planning;
	this->status_publisher.publish(msg);
}

void Agent::planner_status_callback( const custom_messages::Planner_Status_MSG& msg ){
	if(msg.index != this->index){
		for(int i=0; i<msg.completed_tasks.size(); i++){ // make sure all tasks are deactivated7
			if(this->world->get_nodes()[msg.completed_tasks[i]]->is_active()){
				this->world->get_nodes()[msg.completed_tasks[i]]->deactivate();
				ROS_ERROR("Task complete");
			}
		}
	}
}

bool Agent::at_node(int node) {

	for(size_t i=0; i<this->visiting_nodes.size(); i++){
		if(this->visiting_nodes[i] == node){
			return true;
		}
	}
	return false;
}

bool Agent::at_node() { // am I at a node, by edge progress?
	if (this->visiting_nodes.size() > 0) {
		return true;
	}
	else {
		return false;
	}
}

bool Agent::at_goal() { // am I at my goal node?
	if (this->edge_progress >= 0.9 && this->edge.y == this->goal_node->get_index()) {
		return true;
	}
	else if (this->edge_progress <= 0.1 && this->edge.x == this->goal_node->get_index()) {
		return true;
	}
	else{
		return false;
	}
}

void Agent::publish_rviz_path(const std::vector<Point2d> &path){
	visualization_msgs::MarkerArray marker_array;

	int marker_cntr = 500;

	for(size_t i=0; i<path.size(); i++){
		visualization_msgs::Marker marker = makeRvizMarker(path[i], 2.0, 1, marker_cntr);
		marker_array.markers.push_back(marker);		
		marker_cntr++;
	}
	visualization_msgs::Marker marker = makeRvizMarker(this->loc, 5.0, 2, marker_cntr);
	marker_array.markers.push_back(marker);
	marker_cntr++;
	
	this->marker_publisher.publish(marker_array);
}

visualization_msgs::Marker Agent::makeRvizMarker(const Point2d &loc, const double &radius, const int &color, const int &id){
	visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = id;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = loc.x;
    marker.pose.position.y = loc.y;
    marker.pose.position.z = 1.5;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;

    // Set the color -- be sure to set alpha to something non-zero!

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    if(color == 0){
    	marker.color.r = 1.0f;
    }
    else if(color == 1){
    	marker.color.g = 1.0f;
    }
    else if(color == 2){
    	marker.color.b = 1.0f;
    }

    marker.lifetime = ros::Duration(1);
	return marker;    
}



