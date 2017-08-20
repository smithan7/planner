#pragma once

#include <vector>

class Probability_Node
{
public:
	// constructor
	Probability_Node(int i);
	Probability_Node(std::vector<double>& p_in, std::vector<double>& t_in, int ti_in);

	// destructor
	~Probability_Node();

	// insert a new possible arrival time and update following arrival times
	void add_stop_to_my_path(double time, double prob);

	// insert a new possible arrival time and update following arrival times
	void add_stop_to_shared_path(double time, double prob);

	// update probability, a + b - a*b
	double probability_update_inclusive(double a, double b);

	// update probability for exclusive events, a + b
	double probability_update_exclusive(double a, double b);

	// return the probability at time specified
	double get_probability_at_time(double time);

	// clear all stops except 0, 0.0 and inf, 1.0
	void reset();

	// print out the probability table
	void print_out();

	// return vector containing all probabilities of arrival
	std::vector<double>& get_probability_of_completion() { return this->probability_of_completion; };

	// return vector containing all arrival times
	std::vector<double>& get_arrival_time() { return this->completion_time; }

	// return single int giving task index correspoinding to this node
	int get_task_index() { return this->task_index; }

	// have i been claimed? do I need to be reset or searched over?
	bool claimed();

	// if there are claims after the query time, add to list and return true, exclude inf
	bool get_claims_after(double query_time, std::vector<double> &probs, std::vector<double> &times);

private:
	std::vector<double> completion_time;
	std::vector<double> probability_of_completion;
	int task_index;

	double getCDF(double x);
	double getPDF(double x);
	double mean, stan_dev;
	double pi, sqrt_pi;


};

