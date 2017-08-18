/*
 * Costmap.h
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */

#ifndef COSTMAP_UTILS_H_
#define COSTMAP_UTILs_H_

#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

//using namespace std;
//using namespace cv;

class Costmap_Utils {
public:

	// useful stuff
	cv::Mat cells; // holds occupancy
	cv::Mat displayPlot; // for plotting cells
	cv::Mat euclidDist; // array of distances
	bool pay_obstacle_costs; // am I affected by obstacles?

	cv::Point2d NW_Corner, SE_Corner; // map boundaries

	// values 
	int obsFree, infFree, obsOccupied, infOccupied;
	int ros_occupied, ros_wall, ros_free, ros_unknown;
	cv::Vec3b cObsFree, cInfFree, cObsOccupied, cInfOccupied;
	double obsFree_cost, infFree_cost, obsOcc_cost, infOcc_cost;
	bool need_initialization;
	cv::Point map_size_cells;
	cv::Point2d map_size_meters;
	cv::Point2d cells_per_meter, meters_per_cell;

	// 1 = free space // 2 = inferred free space // 3 = domFree
	// 101 = unknown
	// 201 = wall // 202 = inferred wall // 203 = inflated wall

	// functions
	Costmap_Utils();
	virtual ~Costmap_Utils();

	// set map size
    bool initialize_costmap();
    // use satelite info to seed the exploration
	void seed_img();
    // update cells with observation
	void update_cells(const std::vector<int8_t> &occupancy_grid_array, std::vector<cv::Point> &u_pts, std::vector<int> &u_types);
	// used to share updates with team
	void team_map_update( const std::vector<int> &xs, const std::vector<int> &ys, const std::vector<int> &ts);

	// used to get from occ_grid array to cells
	cv::Point get_cell_index(const int &l);

	void cells_to_local_path(const std::vector<cv::Point> &cells_path, std::vector<cv::Point2d> &local_path);
	void cells_to_local(const cv::Point &cell, cv::Point2d &loc);
	void cells_path_to_local_path(const std::vector<cv::Point> &cp, std::vector<cv::Point2d> &lp);
	void local_to_cells(const cv::Point2d &loc, cv::Point &cell);
	void local_to_global(const cv::Point2d &local, cv::Point2d &global);

	// for putting the path together
	double get_local_heading(const cv::Point2d &l1, const cv::Point2d &l2);
	double get_local_euclidian_distance(const cv::Point2d &l1, const cv::Point2d &l2);

	// distances and planning
	double get_cells_euclidian_distance(const cv::Point &a, const cv::Point &b);
	bool a_star_path(const cv::Point &sLoc, const cv::Point &gLoc, std::vector<cv::Point> &path, double &length);
	double get_occ_penalty(const cv::Point &p); // get the occupancy at p
	double a_star_heuristic;

	// display plot
	void build_cells_plot(); // build nice display plot
	void display_costmap(); // show nice display plot and number it
	void add_agent_to_costmap_plot(const cv::Scalar &color, const std::vector<cv::Point> &myPath, const cv::Point &cLoc);

	// true util functions
	double get_global_distance(const cv::Point2d &g1, const cv::Point2d &g2);
	double get_global_heading(const cv::Point2d &g1, const cv::Point2d &g2);
	double to_radians(const double &deg);
	bool point_in_cells(const cv::Point &p);
};

#endif /* COSTMAP_UTILS_H_ */
