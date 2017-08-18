/*
 * Costmap.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */


#include "Costmap.h"

using namespace std;
using namespace cv;

Costmap::Costmap(ros::NodeHandle nHandle){//, std::string param_file){

	ROS_INFO("In");
	this->costmapInitialized = this->utils.initialize_costmap();// param_file );
		

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
	// update the costmap
	this->costmap_subscriber = nHandle.subscribe("/map", 0, &Costmap::costmap_callback, this);
	// get team member map updates
	this->costmap_update_subscriber = nHandle.subscribe("/team_map_update", 0, &Costmap::costmap_update_callback, this);
	// quad status callback
	this->quad_status_subscriber = nHandle.subscribe("/dji_bridge_status", 0, &Costmap::DJI_Bridge_status_callback, this);
	// get goal from Dist MCTS Goal callback
	this->dist_planner_goal_subscriber = nHandle.subscribe("/dist_mcts_goal", 0, &Costmap::dist_planner_goal_callback, this);

	/////////////////////// Publishers //////////////////////////////
	// provide team with my observations
	this->costmap_update_publisher = nHandle.advertise<custom_messages::Costmap_Bridge_Team_Map_Update_MSG>("/team_map_update", 10);
	// tell the DJI Bridge where I am going
	this->path_publisher =  nHandle.advertise<custom_messages::DJI_Bridge_Travel_Path_MSG>("/travel_path", 10);
	// tell everyone my status
	this->status_publisher = nHandle.advertise<custom_messages::Costmap_Bridge_Status_MSG>("/costmap_bridge_status", 10);
	// publish marker to RVIZ
	this->marker_publisher = nHandle.advertise<visualization_msgs::Marker>("/RVIZ", 10);
}

Costmap::~Costmap(){}

void Costmap::costmap_callback(const nav_msgs::OccupancyGrid& cost_in ){

	ROS_INFO("OSU::Map height/width: %d / %d", cost_in.info.height, cost_in.info.width);

	if( !this->costmapInitialized ){ // have i initialized the costmap?
		ROS_ERROR("Costmap_Bridge::Costmap::costmap_callback::costmap not initialized");
		return;
	}
	
	// update my costmap
	//vector<int8_t> occGrid = cost_in.data;
	std::vector<cv::Point> u_pts;
	std::vector<int> u_types;
	this->utils.update_cells( cost_in.data, u_pts, u_types );
	// publish my updates to team
	this->publish_map_updates(u_pts, u_types);
	// plan path on costmap and publish it to the quad
	this->find_path_and_publish();
}

void Costmap::publish_map_updates(const std::vector<cv::Point> &u_pts, const std::vector<int> &u_types){
	// convert to message format
	custom_messages::Costmap_Bridge_Team_Map_Update_MSG tmu;
	for(size_t i=0; i<u_pts.size(); i++){
		tmu.xs.push_back(u_pts[i].x);
		tmu.ys.push_back(u_pts[i].y);
		tmu.tps.push_back(u_types[i]);	
	}
	// publish
	costmap_update_publisher.publish(tmu);
}

void Costmap::costmap_update_callback( const custom_messages::Costmap_Bridge_Team_Map_Update_MSG &update){
	if( !this->costmapInitialized ){ // have i initialized the costmap?
		ROS_ERROR("Costmap_Bridge::Costmap::costmap_callback::costmap not initialized");
		return;
	}


	std::vector<int> xs = update.xs;
	std::vector<int> ys = update.ys;
	std::vector<int> tps = update.tps;
	
	this->utils.team_map_update(xs, ys, tps);
}

void Costmap::DJI_Bridge_status_callback( const custom_messages::DJI_Bridge_Status_MSG& status_in){	
	locationInitialized = true;
	this->local_loc = Point2d(status_in.local_x, status_in.local_y);
	cv:;Point t;	
	this->utils.local_to_cells(this->local_loc, t);

	if(this->utils.point_in_cells(t)){
		this->cell_loc = t;
	}
	else{
		ROS_ERROR("Costmap_Bridge::Costmap::DJI_Bridge_Status::off map");
		return;	
	}

	if(ros::Time::now() - this->plot_time > this->plot_interval){
		this->plot_time = ros::Time::now();

		this->utils.build_cells_plot();
		Scalar blue = Scalar(255,0,0);
		this->utils.add_agent_to_costmap_plot( blue, this->cells_path, this->cell_loc);
		Scalar orange = Scalar(0,165,255);
		this->utils.add_agent_to_costmap_plot( orange, this->cells_path, this->cell_goal);
		this->utils.display_costmap();
	}

	if(ros::Time::now() - this->status_time > this->status_interval){
		this->status_time = ros::Time::now();
		publish_Costmap_Bridge_Status();
	}
	this->find_path_and_publish();
}

void Costmap::dist_planner_goal_callback( const custom_messages::DJI_Bridge_Travel_Path_MSG& path_in){
	this->wp_path.clear();
	this->cells_path.clear();

	for(size_t i=0; i<path_in.local_xs.size(); i++){
		Point2d l_wp(path_in.local_xs[i], path_in.local_ys[i]);
		Point c_wp;
		this->utils.local_to_cells(l_wp, c_wp);

		if(!this->utils.point_in_cells(c_wp)){
			ROS_ERROR("Costmap_Bridge::Dist_Planner_Callback::Wp off map");
			return;
		}

		this->wp_path.push_back(l_wp);
		this->cells_path.push_back(c_wp);
	}

	this->local_goal = this->wp_path.back();
	this->cell_goal = this->cells_path.back();
	this->find_path_and_publish();
}

void Costmap::publish_travel_path(const std::vector<Point2d> &path){
	// initialize msg
	custom_messages::DJI_Bridge_Travel_Path_MSG path_msg;
	cv::Point2d local, next_local;
	// for length of path
	for(size_t i=0; i<path.size(); i++){
		// go from cells to loc_x, loc_y
		// set path
		path_msg.local_xs.push_back(double(local.x));
		path_msg.local_ys.push_back(double(local.y));
		path_msg.altitudes.push_back(this->set_alt);
		// handle heading
		if(i < path.size() - 1){
			// if not last point, heading is from current to next
			next_local = path[i+1];
			double heading = this->utils.get_local_heading(next_local, local);
			path_msg.headings.push_back(heading);
		}
		else{
			// last point, keep current heading
			path_msg.headings.push_back(path_msg.headings.back());
		}
		// set local as next_local and move to next pt
		local = next_local;
	}
	// publish path
	this->path_publisher.publish(path_msg);
}

void Costmap::find_path_and_publish(){
	if( ros::Time::now() - this->actTimer > this->act_interval || this->published_local_goal != this->local_goal ){
		this->published_local_goal = this->local_goal;
		this->actTimer = ros::Time::now();

		bool flag = false;
		if( this->locationInitialized ){
			if( this->costmapInitialized ){
				flag = true;
			}
			else{
				ROS_WARN("Costmap::act::waiting on costmap callback");
			}
		}
		else{
			ROS_WARN("Costmap::act::waiting on location callback");
		}

        if( flag || true ){
        	ROS_INFO("Costmap::act::publishing path to quad");
			this->wp_path.clear();
			ROS_WARN("Fake Goal");
			this->wp_path.push_back(this->local_goal); // this nneds to be ERASED for trials
			this->wp_path.push_back(cv::Point(100,50));
			this->utils.local_to_cells(this->local_goal, this->cell_goal);
        	if(this->find_path(this->cells_path)){
				//ROS_INFO("Path length: %i", int(this->cells_path.size()));
        		std::vector<Point2d> local_path;
        		this->utils.cells_to_local_path(this->cells_path, local_path);

				// assemble plot and display it
				this->utils.build_cells_plot();
				Scalar blue = Scalar(255,0,0);
				this->utils.add_agent_to_costmap_plot( blue, this->cells_path, this->cell_loc);
				Scalar orange = Scalar(0,165,255);
				this->utils.add_agent_to_costmap_plot( orange, this->cells_path, this->cell_goal);
				this->utils.display_costmap();

        		this->publish_travel_path(local_path);
        		this->publish_rviz_path(local_path);	
        	}
		}
	}
}

bool Costmap::find_path( std::vector<cv::Point> &cells_path ){

	// ensure starting fresh
	cells_path.clear();
	// convert wp_path to cells wp
	Point s_wp_cells = this->cell_loc;
	for(size_t i=0; i<this->wp_path.size(); i++){
		// get end point in cells space
		Point e_wp_cells;
		this->utils.local_to_cells(this->wp_path[i], e_wp_cells);
		std::vector<Point> cp;
		double length=0.0;
		if(this->utils.a_star_path(s_wp_cells, e_wp_cells, cp, length)){
			// a* found a path 
			if(cp.size() > 1){
				cells_path.insert(cells_path.end(), cp.begin()+1, cp.end());
			}
		}
		else{
			// a* could not find a path, will also return path to current point
			return false;
		}
		// seed in next starting point as current point
		s_wp_cells = e_wp_cells;
	}
	return true;
}

void Costmap::publish_Costmap_Bridge_Status(){
	custom_messages::Costmap_Bridge_Status_MSG msg;
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

void Costmap::publish_rviz_path(const std::vector<Point2d> &path){
	for(size_t i=0; i<path.size(); i+=5){
		publishRvizMarker(path[i], 2.0, 1, 3+round(i/5));
	}
	publishRvizMarker(this->local_loc, 5.0, 2, 1);
	publishRvizMarker(this->local_goal, 5.0, 0, 2);
}

void Costmap::publishRvizMarker(const Point2d &loc, const double &radius, const int &color, const int &id){
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

    marker.lifetime = ros::Duration(5);
    this->marker_publisher.publish(marker);
}
