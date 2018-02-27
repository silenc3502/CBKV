/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "helper_functions.h"
#include "map.h"
#include "particle_filter.h"

std::default_random_engine gen;
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	int i;
	num_particles = 103;

	/* pow() function makes CPU Performance very low! */

	/* It comes from Udacity Lesson 15 lecture 5. */
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];

	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> gauss_x(x, std_x);

	// TODO: Create normal distributions for y and theta
	normal_distribution<double> gauss_y(y, std_y);
	normal_distribution<double> gauss_theta(theta, std_theta);

	for(i = 0; i < num_particles; i++)
	{
		Particle p;
		p.id = i;
		p.x = gauss_x(gen);
		p.y = gauss_y(gen);
		p.theta = gauss_theta(gen);
		p.weight = 1.0;

		// Noise
		p.x += gauss_x(gen);
		p.y += gauss_y(gen);
		p.theta += gauss_theta(gen);

		particles.push_back(p);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	/* It comes from Udacity Lesson 15 lecture 7. */
	int i;

	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	// Create Guassian Noise
	normal_distribution<double> gauss_x(0, std_x);
	normal_distribution<double> gauss_y(0, std_y);
	normal_distribution<double> gauss_theta(0, std_theta);

	for(i = 0; i < num_particles; i++)
	{
		double theta = particles[i].theta;

		if(fabs(yaw_rate) < 0.00001)
		{
			particles[i].x += velocity * delta_t * cos(theta);
			particles[i].y += velocity * delta_t * sin(theta);
		}
		else
		{
			particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
			particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}

		// Noise
		particles[i].x += gauss_x(gen);
		particles[i].y += gauss_y(gen);
		particles[i].theta += gauss_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	/* It comes from Udacity Lesson 15 lecture 9. */
	uint i, j;

	double min_dist = numeric_limits<double>::max();

	uint observe_num = observations.size();
	uint predict_num = predicted.size();

	for(i = 0; i < observe_num; i++)
	{
		for(j = 0; j < predict_num; j++)
		{
#if 0
			double x_dist = observations[i].x - predicted[j].x;
			double y_dist = observations[i].y - predicted[j].y;
			double dist = pow(x_dist, 2.0) + pow(y_dist, 2.0);
#endif
			double d = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);

			if(d < min_dist)
			{
				min_dist = d;
				observations[i].id = predicted[j].id;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	/* It comes from Udacity Lesson 15 lecture 11, 12, 13, 14, 15 and 19. */
	/* Below #if part is for test about performance.
	   However there are no gain of performance.
	   I think it adjust the memory allocation amount for fit Kernel's Page Size.
	   But, I think it doesn't do it. */
#if 0
	int i;
	uint j;
#endif

	double landmark_range = std_landmark[0];
	double landmark_bearing = std_landmark[1];

#if 0	/* This can makes performance bad too. */
	double x, y, theta, sq_sensor_range = sensor_range * sensor_range;
#endif

	for(int i = 0; i < num_particles; i++)
	{
#if 1
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;
#endif
		x = particles[i].x;
		y = particles[i].y;
		theta = particles[i].theta;

		/* Find Landmarks. */
		//double sq_sensor_range = pow(sensor_range, 2.0);
		double sq_sensor_range = sensor_range * sensor_range;
		vector<LandmarkObs> range_landmarks;

		for(uint j = 0; j < map_landmarks.landmark_list.size(); j++)
		{
			float landmark_x = map_landmarks.landmark_list[j].x_f;
			float landmark_y = map_landmarks.landmark_list[j].y_f;
			int id = map_landmarks.landmark_list[j].id_i;
			double dx = x - landmark_x;
			double dy = y - landmark_y;

			/* pow() function is so bad at this case - performance so bad
			   This makes 33% improvement when doesn't use pow() func. */
			//if(pow(dx, 2.0) + pow(dy, 2.0) <= sq_sensor_range)
			if((dx * dx + dy * dy) <= sq_sensor_range)
				range_landmarks.push_back(LandmarkObs{id, landmark_x, landmark_y});
		}

		/* Observation Coordinates Transform */
		vector<LandmarkObs> mapped_obs;

		for(uint j = 0; j < observations.size(); j++)
		{
			double xx = cos(theta) * observations[j].x - sin(theta) * observations[j].y + x;
			double yy = sin(theta) * observations[j].x + cos(theta) * observations[j].y + y;
			mapped_obs.push_back(LandmarkObs{observations[j].id, xx, yy});
		}

		/* Observation Association */
		dataAssociation(range_landmarks, mapped_obs);

		/* Reset weight */
		particles[i].weight = 1.0;

		/* Now Calculate Weights */
		for(uint j = 0; j < mapped_obs.size(); j++)
		{
			double observe_x = mapped_obs[j].x;
			double observe_y = mapped_obs[j].y;

			int landmark_id = mapped_obs[j].id;

			double landmark_x, landmark_y;
			unsigned int k = 0;
			unsigned int landmarks_num = range_landmarks.size();
			bool found = false;

			while(!found && k < landmarks_num)
			{
				if(range_landmarks[k].id == landmark_id)
				{
					found = true;
					landmark_x = range_landmarks[k].x;
					landmark_y = range_landmarks[k].y;
				}
				k++;
			}

			double dx = observe_x - landmark_x;
			double dy = observe_y - landmark_y;

			/* This is Weights.
			   Very strange situation.
			   When I didn't use pow() then it decrease performance 11%. */
			double weight = (1 / (2 * M_PI * landmark_range * landmark_bearing)) * exp(-(pow(dx, 2.0) / (2 * pow(landmark_range, 2.0)) + (pow(dy, 2.0) / (2 * pow(landmark_bearing, 2.0)))));
			//double weight = (1 / (2 * M_PI * landmark_range * landmark_bearing)) * exp(-(dx * dx / (2 * landmark_range * landmark_range) + (dy * dy / (2 * landmark_bearing * landmark_bearing))));

			if (weight == 0)
				particles[i].weight *= 0.00001;
			else
				particles[i].weight *= weight;
		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	int i;
	vector<double> weights;
	double weight_max = numeric_limits<double>::min();

	/* Comparing & Set max */
	for(i = 0; i < num_particles; i++)
	{
		weights.push_back(particles[i].weight);

		if(particles[i].weight > weight_max)
			weight_max = particles[i].weight;
	}

	/* Create Non-Gaussian Distribution - It's Uniform Distribution */
	uniform_real_distribution<double> non_gauss_double(0.0, weight_max);
	uniform_int_distribution<int> non_gauss_int(0, num_particles - 1);

	/* Index Generation */
	int idx = non_gauss_int(gen);

	double beta = 0.0;

	vector<Particle> resample_particles;

	for(i = 0; i < num_particles; i++)
	{
		beta += non_gauss_double(gen) * 2.0;

		while(beta > weights[idx])
		{
			beta -= weights[idx];
			idx = (idx + 1) % num_particles;
		}
		resample_particles.push_back(particles[idx]);
	}

	particles = resample_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
		const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length()-1);  // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length()-1);  // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length()-1);  // get rid of the trailing space
	return s;
}
