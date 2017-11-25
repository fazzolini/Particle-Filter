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

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles [DONE]
  // TODO: Initialize all particles to first position [DONE]
  // (based on estimates of x, y, theta and their uncertainties from GPS)
  // TODO: Initialize all weights to 1 [DONE]
	// TODO: Add random Gaussian noise to each particle [DONE]
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  // 1) Set the number of particles
  num_particles = 1000;
  
  // 2) Initialize particles to initial positions x and y
  for (int i = 0; i < num_particles; ++i) {
    // Initialize new particle
    Particle p;

    // Set id, position and weight to default values (no noise)
    p.id = i;
    p.x = x;
    p.y = y;
    p.theta = theta;
    p.weight = 1.0;

    // Add to filter
    particles.push_back(p);
  }

  // 3) Add noise
  // Create random number generator
  default_random_engine rand_gen;

  // Define distribution objects
  normal_distribution<double> x_noise_gen(0, std[0]);
  normal_distribution<double> y_noise_gen(0, std[1]);
  normal_distribution<double> theta_noise_gen(0, std[2]);

  // Add noise from normal distributions to particles
  for (int j = 0; j < num_particles; ++j) {
    particles[j].x += x_noise_gen(rand_gen);
    particles[j].y += y_noise_gen(rand_gen);
    particles[j].theta += theta_noise_gen(rand_gen);
  }

  // Set initialized flag
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise [DONE]
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  // Create random number generator
  default_random_engine rand_gen;

  // Define distribution objects
  normal_distribution<double> x_noise_gen(0, std[0]);
  normal_distribution<double> y_noise_gen(0, std[1]);
  normal_distribution<double> theta_noise_gen(0, std[2]);


  // Iterate through particles
  for (int i = 0; i < num_particles; ++i) {
    // Predict particle's next position according to the motion model

    // For easier later use
    double th0 = particles[i].theta;

    // When theta_dot (yaw_rate) is 0 (or close), use special solution (avoid division by zero)
    if (fabs(yaw_rate) < 0.001) {
      particles[i].x += velocity * delta_t * cos(th0);
      particles[i].y += velocity * delta_t * sin(th0);
      // Don't update theta, because it is close to zero, so stays constant
    } else {
      // Use standard bicycle motion model
      particles[i].x += velocity / yaw_rate * (sin(th0 + yaw_rate * delta_t) - sin(th0));
      particles[i].y += velocity / yaw_rate * (cos(th0) - cos(th0 + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    // Add noise to updates
    particles[i].x += x_noise_gen(rand_gen);
    particles[i].y += y_noise_gen(rand_gen);
    particles[i].theta += theta_noise_gen(rand_gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement
  // TODO: assign the observed measurement to this particular landmark
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

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
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    // me
//    return particle;
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
