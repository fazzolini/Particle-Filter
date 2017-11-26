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
#include "helper_functions.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO: Set the number of particles [DONE][OK]
  // TODO: Initialize all particles to first position [DONE][OK]
  // (based on estimates of x, y, theta and their uncertainties from GPS)
  // TODO: Initialize all weights to 1 [DONE][OK]
  // TODO: Add random Gaussian noise to each particle [DONE][OK]
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  // 1) Set the number of particles
  num_particles = 100;

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
  // TODO: Add measurements to each particle and add random Gaussian noise [DONE][OK]
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  // Create random number generator
  default_random_engine rand_gen;

  // Define distribution objects
  normal_distribution<double> x_noise_gen(0, std_pos[0]);
  normal_distribution<double> y_noise_gen(0, std_pos[1]);
  normal_distribution<double> theta_noise_gen(0, std_pos[2]);


  // Iterate through particles
  for (int i = 0; i < num_particles; ++i) {
    // Predict particle's next position according to the motion model

    // For easier later use
    double th0 = particles[i].theta;

    // When theta_dot (yaw_rate) is 0 (or close), use special solution (avoid division by zero)
    if (fabs(yaw_rate) < 0.0001) {
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

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
  // TODO: Find the predicted measurement that is closest to each observed measurement [DONE][FIXED]
  // TODO: assign the observed measurement to this particular landmark [DONE][OK]
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  //   implement this method and use it as a helper during the updateWeights phase.

  /**
   * Approach:
   * 1) for each observation
   * 2) go through all predictions and find closest one's id
   * 3) set this observation's id to the found id
   * Note: this is a double loop with O(m*n) complexity
   */

  // Loop through observations
  for (int i = 0; i < observations.size(); ++i) {
    // Store current observation
    LandmarkObs obs = observations[i];

    // Keep track of distance and id (initialize to unrealistic default values)
    double smallest_distance = numeric_limits<double>::max(); // set to unrealistically large value
    int closest_id = -1; // set to impossible default value

    // Loop through predictions
    for (int j = 0; j < predicted.size(); ++j) {
      // Store current prediction
      LandmarkObs pred = predicted[j];

      // Calculate distance
      double distance = dist(obs.x, obs.y, pred.x, pred.y);

      // Update if distance is smaller than current smallest_distance
      if (distance < smallest_distance) {
        smallest_distance = distance;
        closest_id = pred.id;
      }

      // Store current observation's id to closest prediction's id
      observations[i].id = closest_id;
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  // TODO: Update the weights of each particle using a multi-variate Gaussian distribution
  // You can read more about this distribution here:
  //    https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //    according to the MAP'S coordinate system. You will need to transform between the two systems.
  //    Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  //    The following is a good resource for the theory:
  //    https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //    and the following is a good resource for the actual equation to implement (look at equation
  //    3.33
  //    http://planning.cs.uiuc.edu/node99.html

  /**
   * Observations: vehicle's frame
   * Particles: map's frame
   *
   * Clarifying note to oneself:
   *    - we have a car whose position we don't know
   *    - observations are relative to the car's frame
   *    - for each particle, we assume the particle is the car
   *    - then we need to translate observation from particle frame to map frame
   *      (use homogeneous transform provided in lesson)
   *    - now we have observations with map coordinates
   *    - find closest landmark for each such transformed observation (each observation has multiple points)
   *    - assume the landmark is true position
   *    - use gaussian to get probability that observation point belongs to this landmark
   *    - return product of all observation points probabilities as weight of this particle
   *
   * This method seems like a lot of things to do.
   */

  // Goal: iterate through all particles and calculate each particle's weight
  for (int i = 0; i < particles.size(); ++i) {
    // Select current particle
    Particle p = particles[i];
    double p_x = p.x;
    double p_y = p.y;
    double p_th = p.theta;

    // This will store landmarks that are within sensor_range of the particle
    vector<LandmarkObs> landmarks;

    // This will store transformed observations
    vector<LandmarkObs> observations_transformed;

    // Goal: only select landmarks within sensor_range from current particle
    for (int k = 0; k < map_landmarks.landmark_list.size(); ++k) {
      // Current landmark
      Map::single_landmark_s l = map_landmarks.landmark_list[k];

      // Check distance from current particle
      double distance = dist(p_x, p_y, l.x_f, l.y_f);

      // Select the landmark
      if (distance <= sensor_range) {
        landmarks.push_back(LandmarkObs{l.id_i, l.x_f, l.y_f});
      }
    }

    // Goal: iterate through all observations and transform them into map coordinates
    for (int j = 0; j < observations.size(); ++j) {
      // Use homogeneous transform to get map coordinates of observation
      double x_map = cos(p_th) * observations[j].x - sin(p_th) * observations[j].y + p_x;
      double y_map = sin(p_th) * observations[j].x + cos(p_th) * observations[j].y + p_y;
      observations_transformed.push_back(LandmarkObs{observations[j].id, x_map, y_map});
    }

    // Goal: associate landmark IDs with transformed observation's point
    dataAssociation(landmarks, observations_transformed);

    // We will multiply weights, so start at 1 (initial value)
    particles[i].weight = 1.0;

    // Iterate through all transformed observations and calculate weights
    for (int m = 0; m < observations_transformed.size(); ++m) {
      // For easier access
      double obs_x = observations_transformed[m].x;
      double obs_y = observations_transformed[m].y;
      int obs_id = observations_transformed[m].id;

      // To store landmark coordinates
      double l_x;
      double l_y;

      // Find coordinates of landmark for this observation
      for (int j = 0; j < landmarks.size(); ++j) {
        if (landmarks[j].id == obs_id) {
          l_x = landmarks[j].x;
          l_y = landmarks[j].y;
        }
      }

      // Goal: use gaussian to calculate probability (weight)
      double sd_x = std_landmark[0];
      double sd_y = std_landmark[1];
      double normalizer = 1.0 / (2.0 * M_PI * sd_x * sd_y);
      double power_term = -(pow(l_x - obs_x, 2) / (2 * sd_x * sd_x)) - (pow(l_y - obs_y, 2) / (2 * sd_y * sd_y));
      double proba = normalizer * exp(power_term);

      // Multiply particle's weight by weight calculated in preceding step
      particles[i].weight *= proba;

    }
  }
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  /**
   * Use resampling wheel as taught in class by Sebastian
   */

  // Store resampled particles
  vector<Particle> resampled_particles;

  // Make a list of weights for the wheel
  vector<double> weights;
  for (int i = 0; i < particles.size(); ++i) {
    weights.push_back(particles[i].weight);
  }

  // Determine largest weight
  // max_element returns pointer so need to dereference
  double max_weight = *max_element(weights.begin(), weights.end());

  // Create random number generator
  default_random_engine rand_gen;

  // Distributions for wheel position and beta
  uniform_int_distribution<int> dist_wheel_pos(0, particles.size() - 1);
  uniform_real_distribution<float> dist_beta(0, max_weight);

  // Starting wheel position and beta
  int wheel_position = dist_wheel_pos(rand_gen);
  double beta = 0.0;

  // Resample using the wheel
  for (int j = 0; j < particles.size(); ++j) {
    // Random beta up to twice the largest weight
    beta += dist_beta(rand_gen) * 2.0;
    while (beta > weights[wheel_position]) {
      // Reduce beta by current weight size
      beta -= weights[wheel_position];
      // Advance position (use modulo to make it a 'wheel')
      wheel_position = static_cast<int>((wheel_position + 1) % particles.size());
    }
    // Sample particle at current wheel position
    resampled_particles.push_back(particles[wheel_position]);
  }

  // Update particles with resampled particles
  particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle &particle, const std::vector<int> &associations,
                                         const std::vector<double> &sense_x, const std::vector<double> &sense_y) {
  // particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
