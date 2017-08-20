#include "Proabability_Node.h"

#include <algorithm>


Probability_Node::Probability_Node(int i) {
	this->task_index = i;

	// at t=0, prob of completing = 0
	this->probability_of_completion.push_back(0.0);
	this->completion_time.push_back(0.0);

	// at t=inf, prob of completing = 1.0
	this->probability_of_completion.push_back(1.0);
	this->completion_time.push_back(double(INFINITY));

	this->pi = 3.14159265358979323846264338327950288419716939937510582;
	this->sqrt_pi = sqrt(pi);
}

Probability_Node::Probability_Node(std::vector<double> &p_in, std::vector<double> &t_in, int ti_in) {
	this->completion_time = t_in;
	this->probability_of_completion = p_in;
	this->task_index = ti_in;
}

bool Probability_Node::claimed() {
	if (this->completion_time.size() > 2 || this->probability_of_completion.size() > 2) {
		return true;
	}
	else {
		return false;
	}
}

bool Probability_Node::get_claims_after(double query_time, std::vector<double>& probs, std::vector<double>& times){
	// if there are claims after the query time, add to list and return true, exclude inf
	if (this->probability_of_completion.size() == 2) {
		return false;
	}
	else {
		probs.clear();
		times.clear();
		for (int i = 1; i < this->probability_of_completion.size() - 1; i++) {
			if (this->completion_time[i] > query_time) {
				probs.push_back(this->probability_of_completion[i] - this->probability_of_completion[i-1]);
				times.push_back(this->completion_time[i]);
			}
		}
		if (probs.size() > 0) {
			return true;
		}
		else {
			return false;
		}

	}
}

Probability_Node::~Probability_Node() {}


void Probability_Node::add_stop_to_my_path(double time, double prob) {

	if (prob <= 0) {
		return;
	}
	else {
		prob = std::min(prob, 1.0);
	}


	//UE_LOG(LogTemp, Error, TEXT("Probability_Node::add_stop_to_my_path: completion_time.size(): %i"), this->completion_time.size());
	int time_index = -1;
	for (size_t i = 1; i < this->completion_time.size(); i++) { // find place to insert the new point so that path is in temporal order
															 //UE_LOG(LogTemp, Error, TEXT("%0.2f > %0.2f && %0.2f <= %0.2f = %i"), time, this->completion_time[i - 1], time, this->completion_time[i], time > this->completion_time[i - 1] && time <= this->completion_time[i]);
		if (time > this->completion_time[i - 1] && time <= this->completion_time[i]) { // needs to be lower than the next one not larger
																				 //UE_LOG(LogTemp, Error, TEXT("Probability_Node::add_stop_to_my_path: %i"), i);
			time_index = int(i);
			break;
		}
	}
	if (time_index >= 0) { // was a point found?
						   // yes, insert at the found point
		if (this->completion_time[time_index] == time) { // is it at the same time?
													  // yes, merge the two probabilities
			this->probability_of_completion[time_index] = this->probability_update_exclusive(prob, this->probability_of_completion[time_index]);
		}
		else {
			// no, insert the new time
			double updated_prob = this->probability_update_exclusive(this->probability_of_completion[time_index - 1], prob);
			this->probability_of_completion.insert(this->probability_of_completion.begin() + time_index, updated_prob);
			this->completion_time.insert(this->completion_time.begin() + time_index, time);

		}
		// update all following times
		for (size_t i = size_t(time_index) + 1; i < this->probability_of_completion.size(); i++) {
			this->probability_of_completion[i] = this->probability_update_exclusive(this->probability_of_completion[i], prob);
		}
	}
}

void Probability_Node::add_stop_to_shared_path(double time, double prob) {

	//UE_LOG(LogTemp, Error, TEXT("Probability_Node::add_stop_to_my_path: completion_time.size(): %i"), this->completion_time.size());
	int time_index = -1;
	for (size_t i = 1; i < this->completion_time.size(); i++) { // find place to insert the new point so that path is in temporal order
															 //UE_LOG(LogTemp, Error, TEXT("%0.2f > %0.2f && %0.2f <= %0.2f = %i"), time, this->completion_time[i - 1], time, this->completion_time[i], time > this->completion_time[i - 1] && time <= this->completion_time[i]);
		if (time > this->completion_time[i - 1] && time <= this->completion_time[i]) { // needs to be lower than the next one not larger
																				 //UE_LOG(LogTemp, Error, TEXT("Probability_Node::add_stop_to_my_path: %i"), i);
			time_index = int(i);
			break;
		}
	}
	if (time_index >= 0) { // was a point found?
						   // yes, insert at the found point
		if (this->completion_time[time_index] == time) { // is it at the same time?
													  // yes, merge the two probabilities
			this->probability_of_completion[time_index] = this->probability_update_inclusive(prob, this->probability_of_completion[time_index]);
		}
		else {
			// no, insert the new time
			double updated_prob = this->probability_update_inclusive(this->probability_of_completion[time_index], prob);
			this->probability_of_completion.insert(this->probability_of_completion.begin() + time_index, updated_prob);
			this->completion_time.insert(this->completion_time.begin() + time_index, time);

		}
		// update all following times
		for (size_t i = size_t(time_index) + 1; i < this->probability_of_completion.size(); i++) {
			this->probability_of_completion[i] = this->probability_update_inclusive(this->probability_of_completion[i], prob);
		}
	}
}

double Probability_Node::get_probability_at_time(double time) {
	if (this->completion_time.size() == this->probability_of_completion.size()) { // ensure they are the same size
		for (size_t i = 1; i < this->completion_time.size(); i++) {
			if (time < this->completion_time[i] && time >= this->completion_time[i - 1]) {
				return this->probability_of_completion[i - 1]; // view as a series of step functions, which step am I on?
			}
		}
	}
	return 0.0f;
}

double Probability_Node::probability_update_inclusive(double a, double b) {
	return a + b - a*b;
}

double Probability_Node::probability_update_exclusive(double a, double b) {
	if (1.0f > a + b) {
		return a + b;
	}
	else {
		return 1.0f;
	}
}

double Probability_Node::getPDF(double x) {
	double pdf = 1 / sqrt(2 * pow(this->stan_dev, 2)*pi)*exp(-pow(x - this->mean, 2) / (2 * pow(this->stan_dev, 2)));
	return pdf;
}

double Probability_Node::getCDF(double x) {
	// A Sigmoid Approximation of the Standard Normal Integral
	// Gary R. Waissi and Donald F. Rossin

	double z = (x - this->mean) / this->stan_dev;
	double cdf = 1 / (1 + exp(-sqrt_pi*(-0.0004406*pow(z, 5) + 0.0418198*pow(z, 3) + 0.9*z)));
	return cdf;
}

void Probability_Node::reset() {
	this->probability_of_completion.clear();
	this->completion_time.clear();

	this->probability_of_completion.push_back(0.0);
	this->completion_time.push_back(0.0);

	this->probability_of_completion.push_back(1.0);
	this->completion_time.push_back(double(INFINITY));
}

void Probability_Node::print_out() {
	for (size_t i = 0; i < this->completion_time.size(); i++) {
		printf("Probability_Node::print_out: P( task[%i] = complete |%0.2f sec) = %0.2f", this->task_index, this->completion_time[i], this->probability_of_completion[i]);
	}
}
