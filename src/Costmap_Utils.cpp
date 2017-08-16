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
	this->a_star_heuristic = 1.0; // 1->inf get greedier

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

bool Costmap_Utils::initialize_costmap(char* param_file){
	
    cv::FileStorage fs;
    fs.open(param_file, cv::FileStorage::READ);
    if (!fs.isOpened()){
        ROS_ERROR("Costmap_Utils::initialize_costmap::Failed to open %s", param_file);
        return false;
    }

    int toll = (int) fs["pay_obstacle_costs"];
    if(toll == 1){
    	this->pay_obstacle_costs = true;
    }
    else{
    	this->pay_obstacle_costs = false;
    }
    
    this->map_size_cells.x = (int) fs["cells_width"];
	this->map_size_cells.y = (int) fs["cells_height"];
	this->NW_Corner.x = (double) fs["nw_longitude"];
	this->NW_Corner.y = (double) fs["nw_latitude"];
	this->SE_Corner.x = (double) fs["se_longitude"];
	this->SE_Corner.y = (double) fs["se_latitude"];
	cv::String img_name = (cv::String) fs["img_name"];
	// seed into cells satelite information
	this->seed_img(img_name);

	fs.release();

	// set map width / height in meters
	double d = this->get_global_distance(this->NW_Corner, this->SE_Corner);
	double b = this->get_global_heading(this->NW_Corner, this->SE_Corner);
	this->map_size_meters.x = abs(d*sin(b));
	this->map_size_meters.y = abs(d*cos(b));
	
	// set cells per meter
	this->cells_per_meter.x = this->map_size_meters.x / double(this->map_size_cells.x);
	this->cells_per_meter.y = this->map_size_meters.y / double(this->map_size_cells.y);

	// set meters per cell
	this->meters_per_cell.x = double(this->map_size_cells.x) / this->map_size_meters.x;
	this->meters_per_cell.y = double(this->map_size_cells.y) / this->map_size_meters.y;

	// initialize cells
	cv::Mat a = cv::Mat::ones( this->map_size_cells.x, this->map_size_cells.y, CV_16S)*this->infFree;
	this->cells = a.clone();
	// seed euclid distance, makes everything faster
	a = cv::Mat::ones( this->cells.size(), CV_32FC1)*-1;
	this->euclidDist = a.clone();

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

void Costmap_Utils::seed_img(const cv::String &img_name){
	cv::Mat seed;
	seed = cv::imread(img_name, 0);

	if(!seed.data){
		ROS_ERROR("Costmap::seed_img::Could NOT load img");
		return;
	}

	cv::Point image_size_pixels(seed.cols, seed.rows);

	cv::Point2f cells_per_pixel;
	cells_per_pixel.x = double(this->map_size_cells.x) / double(seed.cols);
	cells_per_pixel.y = double(this->map_size_cells.y) / double(seed.rows);

	// go through every pixel of the image and occupancy map
	for(int i=0; i<seed.cols; i++){
		for(int j=0; j<seed.rows; j++){
			cv::Point p(i,j);
			cv::Point c(round(double(i)*cells_per_pixel.x), round(double(j)*cells_per_pixel.y));
			if(seed.at<short>(p) >= 127 ){
				this->cells.at<short>(c) = this->infOccupied;
			}
			else{
				this->cells.at<short>(c) = this->infFree;
			}
		}
	}
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

void Costmap_Utils::cells_to_local_path(const std::vector<cv::Point> &cells_path, std::vector<cv::Point2f> &local_path){
	local_path.clear();

	for(size_t i=0; i<cells_path.size(); i++){
		cv::Point2f l;
		this->cells_to_local(cells_path[i], l);
		local_path.push_back(l);
	}
}

void Costmap_Utils::cells_to_local(const cv::Point &cell, cv::Point2f &loc){
	// convert from cell to local
	loc.x = this->offset_in_meters.x + double(cell.x) * this->meters_per_cell.x;
	loc.y = this->offset_in_meters.y + double(cell.y) * this->meters_per_cell.y;
}


void Costmap_Utils::local_to_cells(const cv::Point2f &loc, cv::Point &cell){
	// move from local x/y meters to costmap cell
	cell.x = round(this->cells_per_meter.x * (loc.x + this->offset_in_meters.x));
	cell.y = round(this->cells_per_meter.y * (loc.y + this->offset_in_meters.y));
}

double Costmap_Utils::get_local_heading(const cv::Point2f &l1, const cv::Point2f &l2){
	double heading = atan2(l2.x-l1.x,l2.y-l1.y);
	return heading;
}

double Costmap_Utils::get_local_euclidian_distance(const cv::Point2f &a, const cv::Point2f &b){
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

bool Costmap_Utils::a_star_distance(const cv::Point &sLoc, const cv::Point &gLoc, double &length){

	if(this->need_initialization){
		return false;
	}

	length = 0.0;
	if(sLoc == gLoc){
		return true;
	}

	cv::Mat cSet = cv::Mat::zeros(cells.size(), CV_16SC1); // 1 means in closed set, 0 means not
	cv::Mat oSet = cv::Mat::zeros(cells.size(), CV_16SC1); // 1 means in open set, 0 means not

	cv::Mat gScore = cv::Mat::ones(cells.size(), CV_32FC1)*INFINITY; // known cost from initial node to n
	cv::Mat fScore = cv::Mat::ones(cells.size(), CV_32FC1)*INFINITY; // known cost from initial node to n

	std::vector<cv::Point> oVec;
	oVec.push_back(sLoc);
	oSet.at<short>(sLoc) = 1; // starting node has score 0
	gScore.at<float>(sLoc)  = 0; // starting node in open set
	fScore.at<float>(sLoc) = this->a_star_heuristic * this->get_cells_euclidian_distance(sLoc, gLoc);
	fScore.at<float>(gLoc) = 1;

	// for nbrs
	int nx[8] = {-1,-1,-1,0, 0,1,1, 1};
	int ny[8] = { 1, 0,-1,1,-1,1,0,-1};
	double neighbor_distance[8] = {1.414214, 1, 1.414214, 1, 1, 1.414214, 1, 1.414214};

	while(oVec.size() > 0){
		/////////////////// this finds node with lowest fScore and makes current
		double min = INFINITY;
		int mindex = -1;

		for(size_t i=0; i<oVec.size(); i++){
			if(fScore.at<float>(oVec[i]) < min){
				min = fScore.at<float>(oVec[i]);
				mindex = i;
			}
		}

		if(mindex < 0){
			return false;
		}

		cv::Point cLoc = oVec[mindex];
		oVec.erase(oVec.begin() + mindex);
		oSet.at<short>(cLoc) = 0;
		cSet.at<short>(cLoc) = 1;

		/////////////////////// end finding current node
		if(pointCompare(cLoc, gLoc)){ // if the current node equals goal, construct path
			//cerr << "Costmap_Utils::a_star_distance::out clean" << endl;
			length = gScore.at<float>(cLoc);
			return true;
		} ///////////////////////////////// end construct path

		for(int ni = 0; ni<8; ni++){
			cv::Point nbr(cLoc.x + nx[ni], cLoc.y + ny[ni]);
			if(pointOnMat(nbr, cells) ){
				if(cSet.at<short>(nbr) == 1){ // has it already been eval? in cSet
					continue;
				}
				double occ_pen = this->get_occ_penalty(nbr);
				double ngScore = gScore.at<float>(cLoc) + (1 + occ_pen) * neighbor_distance[ni]; 
				if(oSet.at<short>(nbr) == 0){
					oSet.at<short>(nbr) = 1;  // add nbr to open set
					oVec.push_back(nbr);
				}
				else if(ngScore >= gScore.at<float>(nbr) ){ // is temp gscore worse than stored g score of nbr
					continue;
				}

				gScore.at<float>(nbr) = ngScore;
				if(cells.at<short>(nbr) < obsOccupied){
					fScore.at<float>(nbr) = gScore.at<float>(nbr) + this->a_star_heuristic * this->get_cells_euclidian_distance(gLoc,nbr);
				}
				else{
					fScore.at<float>(nbr)= INFINITY;
				}
			}
		}
	}
	return INFINITY;
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
	circle(this->displayPlot, cLoc, 2, color, -1);
	for(size_t i=1; i<path.size(); i++){
		cv::Point a = path[i];
		cv::Point b = path[i-1];
		line(this->displayPlot, a, b, color, 1);
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

double Costmap_Utils::get_global_distance(const cv::Point2f &g1, const cv::Point2f &g2){
	double R = 6378136.6; // radius of the earth in meters

	double lat1 = this->to_radians(g1.x);
	double lon1 = this->to_radians(g1.y);
	double lat2 = this->to_radians(g2.x);
	double lon2 = this->to_radians(g2.y);

	double dlon = lon2 - lon1;
	double dlat = lat2 - lat1;

	double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	double distance = R * c; // in meters
	return distance;
}

double Costmap_Utils::get_global_heading(const cv::Point2f &g1, const cv::Point2f &g2){
	double lat1 = this->to_radians(g1.x);
	double lat2 = this->to_radians(g2.x);

	double dLong = this->to_radians(g2.y - g2.x);

	double x = sin(dLong) * cos(lat2);
	double y = cos(lat1) * sin(lat2) - (sin(lat1)*cos(lat2)*cos(dLong));

	double heading = atan2(x, y);

	return heading;
}

double Costmap_Utils::to_radians(const double &deg){
	return deg*3.141592653589 / 360.0;
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
Point2f Costmap_Utils::local_to_global(const Point2f &local){
	Point2f global;

	double C_EARTH = 6378136.6;
	double dlati = local.x / C_EARTH;
	double lati = dlati + this->origin_lati;
	dlongti = y / ( C_EARTH * math.cos(lati / 2.0 + origin_lati / 2.0) )
	longti = dlongti + origin_longti

	return [lati, longti]


	return global;
}

Point2f Costmap_Utils::global_to_local(const Point2f &global){
	Point2f local;
	double d = distance_from_a_to_b( home_lati, origin_longti, lati, longti )
	double b = heading_from_a_to_b( home_lati, origin_longti, lati, longti )

	local.x = -d*math.sin(b)
	local.y = d*math.cos(b)

	return local;
}


*/