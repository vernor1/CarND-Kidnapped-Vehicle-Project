/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 *  Changed on: May 7, 2017
 *      Author: Yury Melnikov
 */

#include "particle_filter.h"
#include <map>
#include <numeric>
#include <random>

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// 2*Pi, needed for computing multivariate Gaussian distribution
const auto k2pi = 2. * M_PI;

// Number of particles
enum { kNumParticles = 50 };

// Local Helper-Functions
// -----------------------------------------------------------------------------

double GetDistance(double x0, double y0, double x1, double y1) {
  auto dx = x1 - x0;
  auto dy = y1 - y0;
  return std::sqrt(dx * dx + dy * dy);
}

} // namespace

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  weights.clear();
  // Set the number of particles
  num_particles = kNumParticles;

  // Create normal (Gaussian) distributions for x, y, theta
  std::random_device rd;
  std::default_random_engine gen(rd());
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  for (auto id = 0; id < num_particles; ++id) {
    Particle p;
    p.id = id;
    // Initialize all particles to first position (based on estimates of  x, y,
    // theta and their uncertainties from GPS). Add random Gaussian noise to
    // each particle.
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    // Initialize all weights to 1. 
    p.weight = 1;
    particles.push_back(p);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t,
                                double std_pos[],
                                double velocity,
                                double yaw_rate) {
  const auto velocity_by_yaw_rate = velocity / yaw_rate;
  const auto yaw_rate_delta_t = yaw_rate * delta_t;

  std::random_device rd;
  std::default_random_engine gen(rd());

  for (auto& p : particles) {
    // Equations for updating x, y and the yaw angle when the yaw rate is not
    // equal to zero:
    //   p_x += v / theta_dot * (sin(p_theta + theta_dot * dt) - sin(p_theta))
    //   p_y += v / theta_dot * (cos(p_theta) - cos(p_theta + theta_dot * dt))
    //   p_theta += theta_dot * dt
    p.x += velocity_by_yaw_rate
         * (std::sin(p.theta + yaw_rate_delta_t) - std::sin(p.theta));
    p.y += velocity_by_yaw_rate
         * (std::cos(p.theta) - std::cos(p.theta + yaw_rate_delta_t));
    p.theta += yaw_rate_delta_t;

    // Create normal (Gaussian) distributions for x, y, theta
    std::normal_distribution<double> dist_x(p.x, std_pos[0]);
    std::normal_distribution<double> dist_y(p.y, std_pos[1]);
    std::normal_distribution<double> dist_theta(p.theta, std_pos[2]);

    // Add random Gaussian noise
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
                                     std::vector<LandmarkObs>& observations) {
  // Empty.
}

void ParticleFilter::updateWeights(double sensor_range,
                                   double std_landmark[], 
                                   std::vector<LandmarkObs> observations,
                                   Map map_landmarks) {
  weights.clear();
  const auto one_by_2_pi_sigma_xy = 1.
               / (k2pi * std_landmark[0] * std_landmark[1]);
  const auto double_sigma_x_sq = 2. * std_landmark[0] * std_landmark[0];
  const auto double_sigma_y_sq = 2. * std_landmark[1] * std_landmark[1];

  for (auto& p : particles) {
    p.weight = 1;
    struct Coordinates {
      double x;
      double y;
      Coordinates() : x(0), y(0) { }
      Coordinates(double in_x, double in_y) : x(in_x), y(in_y) { }
    };
    std::vector<Coordinates> transformed_observations;

    for (const auto& obs : observations) {
      // Rotate and translate every observation to map coordinates w.r.t. to the
      // particle position and heading:
      //   x = obs_x * cos(p_theta) - obs_y * sin(p_theta) + p_x
      //   y = obs_x * sin(p_theta) + obs_y * cos(p_theta) + p_y
      transformed_observations.push_back(Coordinates(
        obs.x * std::cos(p.theta) - obs.y * std::sin(p.theta) + p.x,
        obs.x * std::sin(p.theta) + obs.y * std::cos(p.theta) + p.y));
    }

    // Associate the observation with a landmark
    bool is_any_observation_associated = false;
    for (const auto& obs: transformed_observations) {
      // Check that transformed observation is within range
      if (GetDistance(p.x, p.y, obs.x, obs.y) < sensor_range) {
        // Create an ordered map of distances and corresponding landmarks
        std::map<double, Map::single_landmark_s> landmark_distances;
        for (const auto& landmark : map_landmarks.landmark_list) {
          auto distance = GetDistance(obs.x, obs.y, landmark.x_f, landmark.y_f);
          // Check that landmark is within range
          if (distance < sensor_range) {
            landmark_distances[distance] = landmark;
          }
        }
        if (!landmark_distances.empty()) {
	  // Update the weights of each particle using a multivariate Gaussian
          // distribution
          const auto& nearest_landmark = landmark_distances.begin()->second;
          const auto dx = obs.x - nearest_landmark.x_f;
          const auto dy = obs.y - nearest_landmark.y_f;
          p.weight *= one_by_2_pi_sigma_xy
                    * std::exp(-(dx * dx / double_sigma_x_sq
                               + dy * dy / double_sigma_y_sq));
          is_any_observation_associated = true;
        }
      }
    }
    // Handle the corner case when a poorly chosen particle is too far from all
    // landmarks
    if (!is_any_observation_associated) {
      p.weight = 0;
    }
    weights.push_back(p.weight);
  }
}

void ParticleFilter::resample() {
  // Resample particles with replacement with probability proportional to their
  // weight
  std::random_device rd;
  std::default_random_engine gen(rd());
  std::discrete_distribution<> distribution(weights.begin(), weights.end());
  std::vector<Particle> new_particles;
  for (auto id = 0; id < num_particles; ++id) {
    auto& p = particles.at(distribution(gen));
    p.id = id;
    new_particles.push_back(p);
  }
  particles = new_particles;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
