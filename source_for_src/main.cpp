#define _CRT_SECURE_NO_DEPRECATE

#include "World.h"

#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main() {

	// builds the complete world, maps, node, agents, tasks
	int params = -1;
	bool display_map = true;

	std::vector<cv::String> task_selection_methods;

	task_selection_methods.push_back("impact_completion_value");
	task_selection_methods.push_back("impact_completion_value");
	task_selection_methods.push_back("impact_completion_reward");
	task_selection_methods.push_back("value_completion");
	task_selection_methods.push_back("greedy_completion_reward");
	task_selection_methods.push_back("greedy_completion_time");
	task_selection_methods.push_back("greedy_current_reward");
	task_selection_methods.push_back("random_task");

	cv::String img_name;// = "test_env_obstacles";
	World world = World(params, display_map, task_selection_methods, img_name);

	return 1;
}
