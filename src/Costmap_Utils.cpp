/*
 * Costmap.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */




#include "Costmap_Utils.h"

bool pointCompare(const cv::Point &a, const cv::Point &b);
bool pointOnMat(const cv::Point &a, const cv::Mat &b);
std::vector<cv::Point> get_image_points_at_intensity(const cv::Mat &image, const int &intensity);
double lin_interp(const double &p_min, const double &p_max, const double &p);


Costmap_Utils::Costmap_Utils(){

	// I need to be initialized
	this->need_initialization = true;

	// set heuristic for A*
	this->a_star_heuristic = 3.0; // 1->inf get greedier

	// ros's occupancy grid values
	this->ros_unknown = -1;
	this->ros_occupied = 100;
	this->ros_free = 0;

	// values I use to track everything
	this->obsFree = 1;
	this->infFree = 2;
	this->obsOccupied = 201;
	this->infOccupied = 202;

	// A* costs for travelling over everything


	// annoying but it works to seed plot colors
	cv::Vec3b a(255,255,255);
	this->cObsFree = a;
	a = cv::Vec3b(200,200,200);
	this->cInfFree = a;
	a = cv::Vec3b(0,0,0);
	this->cObsOccupied = a;
	a = cv::Vec3b(50,50,50);
	this->cInfOccupied = a;

}

Costmap_Utils::~Costmap_Utils() {}

bool Costmap_Utils::initialize_costmap(){
	std::string filename = "/home/nvidia/catkin_ws/src/costmap_bridge/hardware_params.xml";	
	/*
	ROS_INFO("writing param file");
	std::string filename = "/home/nvidia/catkin_ws/src/costmap_bridge/hardware_params.xml";
    cv::FileStorage fr(filename, cv::FileStorage::WRITE);
    fr << "cells_width" << 1000;
    fr << "cells_height" << 1000;
	fr << "nw_longitude" <<-123.251004;
	fr << "nw_latitude" << 44.539847;
	fr << "se_longitude" << -123.247446;
	fr << "se_latitude" << 44.538552;
    fr << "obstacle_image" << "/home/nvidia/catkin_ws/src/costmap_bridge/hardware_obstacles.png";
    fr << "pay_obstacle_costs" << 1;
    fr.release();
    ROS_INFO("wrote param file");
	*/
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()){
        ROS_ERROR("Costmap_Utils::initialize_costmap::Failed to open /home/nvidia/catkin_ws/src/costmap_bridge/hardware_params.xml");
        return false;
    }
	ROS_INFO("Costmap_Utils::initialize_costmap::Opened /home/nvidia/catkin_ws/src/costmap_bridge/hardware_params.xml");


    int toll = (int) fs["pay_obstacle_costs"];
    if(toll == 1){
    	this->pay_obstacle_costs = true;
    }
    else{
    	this->pay_obstacle_costs = false;
    }
    
    fs["cells_width"] >> this->map_size_cells.x;
    fs["cells_height"] >> this->map_size_cells.y;
	ROS_INFO("cells size: %i, %i", this->map_size_cells.x, this->map_size_cells.y);
	fs["nw_longitude"] >> this->NW_Corner.x;
	fs["nw_latitude"] >> this->NW_Corner.y;
	fs["se_longitude"] >> this->SE_Corner.x;
	fs["se_latitude"] >> this->SE_Corner.y;
	std::string img_name;
	fs["obstacle_img"] >> img_name;
	fs.release();
	ROS_INFO("Costmap_Utils::initialize_costmap::origin: %0.12f / %0.12f", this->NW_Corner.x, this->NW_Corner.y);
		

	// initialize cells
	cv::Mat a = cv::Mat::ones( this->map_size_cells.y, this->map_size_cells.x, CV_16S)*this->infFree;
	this->cells = a.clone();
	ROS_INFO("cells size: %i, %i", this->cells.cols, this->cells.rows);
	
	// seed euclid distance, makes everything faster
	a = cv::Mat::ones( this->cells.size(), CV_32FC1)*-1;
	this->euclidDist = a.clone();


	// seed into cells satelite information
	this->seed_img();

	ROS_INFO("nw / se: (%0.6f, %0.6f) / (%0.6f, %0.6f)", this->NW_Corner.x, this->NW_Corner.y, this->SE_Corner.x, this->SE_Corner.y);
	// set map width / height in meters
	double d = this->get_global_distance(this->NW_Corner, this->SE_Corner);
	double b = this->get_global_heading(this->NW_Corner, this->SE_Corner);
	this->map_size_meters.x = abs(d*sin(b));
	this->map_size_meters.y = abs(d*cos(b));
	
	ROS_INFO("map size: %0.2f, %0.2f (m)", this->map_size_meters.x, this->map_size_meters.y);
	
	// set cells per meter
	this->meters_per_cell.x = this->map_size_meters.x / double(this->map_size_cells.x);
	this->meters_per_cell.y = this->map_size_meters.y / double(this->map_size_cells.y);

	// set meters per cell
	this->cells_per_meter.x = double(this->map_size_cells.x) / this->map_size_meters.x;
	this->cells_per_meter.y = double(this->map_size_cells.y) / this->map_size_meters.y;

	if(pay_obstacle_costs){
		this->obsFree_cost = 0.0;
		this->infFree_cost = 0.05;
		this->infOcc_cost = 0.5;
		this->obsOcc_cost = 10;
	}
	else{
		this->obsFree_cost = 0.0;
		this->infFree_cost = 0.0;
		this->infOcc_cost = 0.0;
		this->obsOcc_cost = 0.0;
	}

	// announce I am initialized!
	this->need_initialization = false;

	ROS_INFO("Costmap_Utils::initialize_costmap::complete");
	return true;
}

void Costmap_Utils::seed_img(){
	cv::Mat seed = cv::imread("/home/nvidia/catkin_ws/src/costmap_bridge/hardware_obstacles.png", CV_LOAD_IMAGE_GRAYSCALE);
	if(!seed.data){
		ROS_ERROR("Costmap::seed_img::Could NOT load img");
		return;
	}

	/*
	cv::namedWindow("seed", CV_WINDOW_NORMAL);
	cv::imshow("seed", seed);
	cv::waitKey(10);
	*/
	cv::resize(seed, seed, this->cells.size());
	
	/*
	cv::namedWindow("seed2", CV_WINDOW_NORMAL);
	cv::imshow("seed2", seed);
	cv::waitKey(0);
	*/
	// go through every pixel of the image and occupancy map
	for(int i=0; i<seed.cols; i++){
		for(int j=0; j<seed.rows; j++){
			//ROS_INFO("seed size (%i,%i) and point (%i,%i)", seed.cols, seed.rows, i, j);
			cv::Point p(i,j);
			//cv::Point c(round(double(i)*cells_per_pixel.x), round(double(j)*cells_per_pixel.y));
			//ROS_INFO("cells size (%i,%i) and point (%i,%i)", this->cells.cols, this->cells.rows, c.x, c.y);	
			if(seed.at<uchar>(p) >= 127 ){
				this->cells.at<short>(p) = this->infOccupied;
			}
			else{
				this->cells.at<short>(p) = this->infFree;
			}
		}
	}
	/*
	ROS_INFO("seeded cells");
	this->build_cells_plot();
	this->display_costmap();// show nice display plot and number it
	cv::waitKey(0);
	*/
}

void Costmap_Utils::update_cells( const std::vector<int8_t> &occupancy_grid_array, std::vector<cv::Point> &u_pts, std::vector<int> &u_types){
	
	u_pts.clear();
	u_types.clear();
	// if I haven't been initialized then don't include
	if( this->need_initialization ){
		return;
	}

	for(size_t i=0; i<occupancy_grid_array.size(); i++){
		 cv::Point p = get_cell_index( i );

		if(occupancy_grid_array[i] == ros_unknown){
		 	// do nothing!!!!
		 	//if( cells.at<short>(p) != unknown){
		 	//	cells.at<short>(p) = unknown;
		 	//}
		}
		else if(occupancy_grid_array[i] < ros_occupied / 4){//ros_free){
		 	if( cells.at<short>(p) != obsFree){
		 		cells.at<short>(p) = obsFree;
		 		u_pts.push_back(p);
		 		u_types.push_back(this->obsFree);
		 	}
		}
		else{
		 	if( cells.at<short>(p) != obsOccupied){
		 		cells.at<short>(p) = obsOccupied;
		 		u_pts.push_back(p);
		 		u_types.push_back(this->obsOccupied);
		 	}
		}
	}
}

void Costmap_Utils::team_map_update( const std::vector<int> &xs, const std::vector<int> &ys, const std::vector<int> &ts){

	if( this->need_initialization ){
		return;
	}

	for(size_t i=0; i<xs.size(); i++){
		cv::Point p(xs[i], ys[i]);
		if(this->point_in_cells(p)){
			if(ts[i] == this->obsFree){
			 	if( cells.at<short>(p) != obsFree){
			 		cells.at<short>(p) = obsFree;
			 	}
			}
			else if(ts[i] == this->obsOccupied){
			 	if( cells.at<short>(p) != obsOccupied){
			 		cells.at<short>(p) = obsOccupied;
			 	}
			}
		}
	}
}

 cv::Point Costmap_Utils::get_cell_index(const int &l){
 	// this is from a single vector representation of the map
	cv::Point p;
	p.x = floor( l / this->map_size_cells.x );
	p.y = l % this->map_size_cells.y;

	return p;
}

void Costmap_Utils::cells_to_local_path(const std::vector<cv::Point> &cells_path, std::vector<cv::Point2d> &local_path){
	local_path.clear();

	for(size_t i=0; i<cells_path.size(); i++){
		cv::Point2d l;
		this->cells_to_local(cells_path[i], l);
		local_path.push_back(l);
	}
}

void Costmap_Utils::cells_to_local(const cv::Point &cell, cv::Point2d &loc){
	// convert from cell to local
	loc.x = double(cell.x) * this->meters_per_cell.x;
	loc.y = double(cell.y) * this->meters_per_cell.y;
}


void Costmap_Utils::local_to_cells(const cv::Point2d &loc, cv::Point &cell){
	// move from local x/y meters to costmap cell
	cell.x = round(this->cells_per_meter.x * loc.x);
	cell.y = round(this->cells_per_meter.y * loc.y);
}

double Costmap_Utils::get_local_heading(const cv::Point2d &l1, const cv::Point2d &l2){
	double heading = atan2(l2.x-l1.x,l2.y-l1.y);
	return heading;
}

double Costmap_Utils::get_local_euclidian_distance(const cv::Point2d &a, const cv::Point2d &b){
	return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) );
}

double Costmap_Utils::get_cells_euclidian_distance(const cv::Point &a, const cv::Point &b){
	int dx = abs(a.x - b.x);
	int dy = abs(a.x - b.y);

	if(euclidDist.at<float>(dx,dy) == -1){
		euclidDist.at<float>(dx,dy) = sqrt(pow(dx,2) + pow(dy,2));
	}
	return(euclidDist.at<float>(dx,dy) );
}


bool Costmap_Utils::a_star_path(const cv::Point &sLoc, const cv::Point &gLoc, std::vector<cv::Point> &path, double &length){

	if(this->need_initialization){
		return false;
	}

	// ensure that 
	path.clear();
	length = 0.0;

	if(sLoc == gLoc){
		path.clear();
		length = 0.0;
		return true;
	}

	cv::Mat cSet = cv::Mat::zeros(cells.size(), CV_16S); // 1 means in closed set, 0 means not
	cv::Mat oSet = cv::Mat::zeros(cells.size(), CV_16S); // 1 means in open set, 0 means not
	cv::Mat cameFromX = cv::Mat::ones(cells.size(), CV_16S)*-1; // each square has a vector of the location it came from
	cv::Mat cameFromY = cv::Mat::ones(cells.size(), CV_16S)*-1; // each square has a vector of the location it came from

	cv::Mat gScore = cv::Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n
	cv::Mat fScore = cv::Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n

	std::vector<cv::Point> oVec;
	oVec.push_back(sLoc);
	oSet.at<short>(sLoc) = 1; // starting node has score 0
	gScore.at<float>(sLoc)  = 0.0; // starting node in open set
	fScore.at<float>(sLoc) = this->a_star_heuristic * this->get_cells_euclidian_distance(sLoc, gLoc);
	fScore.at<float>(gLoc) = 0.0;

	// for nbrs
	int nx[8] = {-1,-1,-1,0, 0,1,1, 1};
	int ny[8] = { 1, 0,-1,1,-1,1,0,-1};
	double neighbor_distance[8] = {1.414214, 1, 1.414214, 1, 1, 1.414214, 1, 1.414214};

	while(oVec.size() > 0){
		// find node with lowest fScore and make current
		double min = INFINITY;
		int mindex = -1;
		for(size_t i=0; i<oVec.size(); i++){
			if(fScore.at<float>(oVec[i]) < min){
				min = fScore.at<float>(oVec[i]);
				mindex = i;
			}
		}
		// I did NOT find mindex! Search over!
		if( mindex < 0){
			return false;
		}

		// I did find mindex, do maintenance
		cv::Point cLoc = oVec[mindex];
		oVec.erase(oVec.begin() + mindex);
		oSet.at<short>(cLoc) = 0;
		cSet.at<short>(cLoc) = 1;

		// am I at the goal?
		if(pointCompare(cLoc, gLoc) ){
			// I am, construct the path 
			length = gScore.at<float>(cLoc);
			path.push_back(gLoc);
			while( cLoc.x != sLoc.x || cLoc.y != sLoc.y ){ // work backwards to start
				cv::Point tLoc(cameFromX.at<short>(cLoc), cameFromY.at<short>(cLoc));
				path.push_back(tLoc); // append path
				cLoc.x = tLoc.x;
				cLoc.y = tLoc.y;
			}
			reverse(path.begin(),path.end());
			return true;
		}

		// not at the goal, get new nbrs
		for(int ni = 0; ni<8; ni++){
			// potential nbr
			cv::Point nbr(cLoc.x + nx[ni], cLoc.y + ny[ni] );
			// viable nbr?
			if(pointOnMat(nbr, cells)){
				
				if(cSet.at<short>(nbr) == 1){ // has it already been eval? in cSet
					continue;
				}

				// calc temporary gscore, estimate of total cost
				double occ_pen = this->get_occ_penalty(nbr);
				double ngScore = gScore.at<float>(cLoc) + (1 + occ_pen) * neighbor_distance[ni]; 
				if(oSet.at<short>(nbr) == 0){
					oSet.at<short>(nbr) = 1;  // add nbr to open set
					oVec.push_back(nbr);
				}
				else if(ngScore >= gScore.at<float>(nbr) ){ // is temp gscore worse than stored g score of nbr
					continue;
				}
				cameFromX.at<short>(nbr) = cLoc.x;
				cameFromY.at<short>(nbr) = cLoc.y;

				gScore.at<float>(nbr) = ngScore;
				if(cells.at<short>(nbr) < 102){
					fScore.at<float>(nbr) = gScore.at<float>(nbr) + this->a_star_heuristic * this->get_cells_euclidian_distance(gLoc,nbr);
				}
				else{
					fScore.at<float>(nbr)= INFINITY;
				}
			}
		}
	}
	return false;
}

double Costmap_Utils::get_occ_penalty(const cv::Point &p){
	if(this->cells.at<short>(p) == this->obsFree){
		return this->obsFree_cost;
	}
	else if(this->cells.at<short>(p) == this->infFree){
		return this->infFree_cost;
	}
	else if(this->cells.at<short>(p) == this->infOccupied){
		return this->infOcc_cost;
	}
	else if(this->cells.at<short>(p) == this->infOccupied){
		return this->obsOcc_cost;
	}
	else{
		// something went wrong, probably shouldn't go there...
		return double(INFINITY);
	}
}

void Costmap_Utils::display_costmap(){ // show nice display plot and number it
	cv::namedWindow("Cells Mat", cv::WINDOW_NORMAL);
	cv::imshow("Cells Mat", this->displayPlot);
	cv::waitKey(1);
}

void Costmap_Utils::build_cells_plot(){
	this->displayPlot= cv::Mat::zeros(cells.size(), CV_8UC3);
	for(int i=0; i<this->cells.cols; i++){
		for(int j=0; j<this->cells.rows; j++){
			cv::Point a(i,j);
			if(this->cells.at<short>(a) == this->obsFree){
				this->displayPlot.at<cv::Vec3b>(a) = this->cObsFree;
			}
			else if(this->cells.at<short>(a) == this->infFree){
				this->displayPlot.at<cv::Vec3b>(a) = this->cInfFree;
			}
			else if(this->cells.at<short>(a) == this->obsOccupied){
				this->displayPlot.at<cv::Vec3b>(a) = this->cObsOccupied;
			}
			else if(this->cells.at<short>(a) == this->infOccupied){
				this->displayPlot.at<cv::Vec3b>(a) = this->cInfOccupied;
			}
		}
	}
}

void Costmap_Utils::add_agent_to_costmap_plot(const cv::Scalar &color, const std::vector<cv::Point> &path, const cv::Point &cLoc){
	circle(this->displayPlot, cLoc, 20, color, -1);
	for(size_t i=1; i<path.size(); i++){
		cv::Point a = path[i];
		//cv::Point b = path[i-1];
		//line(this->displayPlot, a, b, color, 10);
		circle(this->displayPlot, a, 5, color, -1);
	}
}

bool pointCompare(const cv::Point &a, const cv::Point &b){
	if(a.x == b.x && a.y == b.y){
		return true;
	}
	else{
		return false;
	}
}

bool pointOnMat(const cv::Point &a, const cv::Mat &b){
	if(a.x >= 0 && a.x < b.cols && a.y >= 0 && a.y < b.rows){
		return true;
	}
	else{
		return false;
	}
}

std::vector<cv::Point> get_image_points_at_intensity(const cv::Mat &image, const int &intensity){
	std::vector<cv::Point> temp;
	for(int i=0; i<image.cols; i++){
		for(int j=0; j<image.rows; j++){
			if(image.at<uchar>(i,j,0) == intensity){
				cv::Point t(i,j);
				temp.push_back(t);
			}
		}
	}
	return temp;
}

double lin_interp(const double &p_min, const double &p_max, const double &p){
	return (p-p_min)/(p_max-p_min);
}

double Costmap_Utils::get_global_distance(const cv::Point2d &g1, const cv::Point2d &g2){
	double R = 6378136.6; // radius of the earth in meters

	double lat1 = this->to_radians(g1.y);
	double lon1 = this->to_radians(g1.x);
	double lat2 = this->to_radians(g2.y);
	double lon2 = this->to_radians(g2.x);

	double dlon = lon2 - lon1;
	double dlat = lat2 - lat1;

	double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	double distance = R * c; // in meters
	return distance;
}

double Costmap_Utils::get_global_heading(const cv::Point2d &g1, const cv::Point2d &g2){
	double lat1 = this->to_radians(g1.y);
	double lat2 = this->to_radians(g2.y);

	double dLong = this->to_radians(g2.x - g1.x);

	double x = sin(dLong) * cos(lat2);
	double y = cos(lat1) * sin(lat2) - (sin(lat1)*cos(lat2)*cos(dLong));

	double heading = atan2(x, y);

	return heading;
}

double Costmap_Utils::to_radians(const double &deg){
	return deg*3.141592653589 / 180.0;
}

bool Costmap_Utils::point_in_cells(const cv::Point &p){
	if(p.x >= 0 && p.y >= 0){
		if(p.x < this->map_size_cells.x && p.y < this->map_size_cells.y){
			return true;
		}
	}
	return false;
}

/*
Point2d Costmap_Utils::local_to_global(const Point2d &local){
	Point2d global;

	double C_EARTH = 6378136.6;
	double dlati = local.x / C_EARTH;
	double lati = dlati + this->origin_lati;
	dlongti = y / ( C_EARTH * math.cos(lati / 2.0 + origin_lati / 2.0) )
	longti = dlongti + origin_longti

	return [lati, longti]


	return global;
}

Point2d Costmap_Utils::global_to_local(const Point2d &global){
	Point2d local;
	double d = distance_from_a_to_b( home_lati, origin_longti, lati, longti )
	double b = heading_from_a_to_b( home_lati, origin_longti, lati, longti )

	local.x = -d*math.sin(b)
	local.y = d*math.cos(b)

	return local;
}
*/

