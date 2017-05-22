#include "particle_filter.h"
#include <numeric>

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// A value below this absolute constant is considered zero
const auto kEpsilon = 1e-6;

// 2*Pi, needed for computing multivariate Gaussian distribution
const auto k2pi = 2. * M_PI;

// Local Helper-Functions
// -----------------------------------------------------------------------------

double GetDistance(double x0, double y0, double x1, double y1) {
  auto dx = x1 - x0;
  auto dy = y1 - y0;
  return std::sqrt(dx * dx + dy * dy);
}

} // namespace

ParticleFilter::ParticleFilter(unsigned int num_particles,
                               const DirectedPosition& init_pos,
                               const DirectedPosition& std_pos)
  : rng_(random_device_())
{
  // Create normal (Gaussian) distributions for x, y, yaw
  std::normal_distribution<double> dist_x(init_pos.x, std_pos.x);
  std::normal_distribution<double> dist_y(init_pos.y, std_pos.y);
  std::normal_distribution<double> dist_yaw(init_pos.yaw, std_pos.yaw);

  for (auto id = 0; id < num_particles; ++id) {
    Particle p;
    p.id = id;
    // Initialize all particles to first position (based on estimates of  x, y,
    // yaw and their uncertainties from GPS). Add random Gaussian noise to
    // each particle.
    p.x = dist_x(rng_);
    p.y = dist_y(rng_);
    p.yaw = dist_yaw(rng_);
    // Initialize all weights to 1
    p.weight = 1;
    particles_.push_back(p);
  }
}

void ParticleFilter::Predict(double delta_t,
                             const DirectedPosition& std_pos,
                             double velocity,
                             double yaw_rate) {
  double v_dt = 0;
  double v_by_theta_dot = 0;
  double theta_dot_dt = 0;

  bool is_zero_yaw_rate = std::fabs(yaw_rate) < kEpsilon;
  if (is_zero_yaw_rate) {
    v_dt = velocity * delta_t;
  } else {
    v_by_theta_dot = velocity / yaw_rate;
    theta_dot_dt = yaw_rate * delta_t;
  }

  // Create normal (Gaussian) distributions for x, y, yaw
  std::normal_distribution<double> dist_x(0, std_pos.x);
  std::normal_distribution<double> dist_y(0, std_pos.x);
  std::normal_distribution<double> dist_yaw(0, std_pos.yaw);

  for (auto& p : particles_) {
    if (is_zero_yaw_rate) {
      // Equations for updating x, y and the yaw angle when the yaw rate is
      // equal to zero:
      //   p_x += v * dt * cos(p_yaw)
      //   p_y += v * dt * sin(p_yaw)
      //   p_yaw = p_yaw
      p.x += v_dt * std::cos(p.yaw);
      p.y += v_dt * std::sin(p.yaw);
    } else {
      // Equations for updating x, y and the yaw angle when the yaw rate is not
      // equal to zero:
      //   p_x += v / theta_dot * (sin(p_yaw + theta_dot * dt) - sin(p_yaw))
      //   p_y += v / theta_dot * (cos(p_yaw) - cos(p_yaw + theta_dot * dt))
      //   p_yaw += theta_dot * dt
      p.x += v_by_theta_dot
           * (std::sin(p.yaw + theta_dot_dt) - std::sin(p.yaw));
      p.y += v_by_theta_dot
           * (std::cos(p.yaw) - std::cos(p.yaw + theta_dot_dt));
      p.yaw += theta_dot_dt;
    }

    // Add random Gaussian noise
    p.x += dist_x(rng_);
    p.y += dist_y(rng_);
    p.yaw += dist_yaw(rng_);
  }
}

ParticleFilter::Particle ParticleFilter::Update(
  double sensor_range,
  const Position& std_landmark,
  const PositionSequence& observations,
  const PositionSequence& landmarks) {

  // Local constants for optimization of expensive computations
  const auto one_by_2_pi_sigma_xy = 1.
               / (k2pi * std_landmark.x * std_landmark.y);
  const auto double_sigma_x_sq = 2. * std_landmark.x * std_landmark.x;
  const auto double_sigma_y_sq = 2. * std_landmark.y * std_landmark.y;

  // Auxilliary container of weights for further resampling
  std::vector<double> weights;

  for (auto& p : particles_) {
    p.weight = 1;
    std::vector<Position> transformed_observations;

    for (const auto& obs : observations) {
      // Rotate and translate every observation to map coordinates w.r.t. to the
      // particle position and heading:
      //   x = obs_x * cos(p_yaw) - obs_y * sin(p_yaw) + p_x
      //   y = obs_x * sin(p_yaw) + obs_y * cos(p_yaw) + p_y
      transformed_observations.push_back(Position(
        obs.x * std::cos(p.yaw) - obs.y * std::sin(p.yaw) + p.x,
        obs.x * std::sin(p.yaw) + obs.y * std::cos(p.yaw) + p.y));
    }

    // Associate the observation with a landmark
    bool is_any_observation_associated = false;
    for (const auto& obs: transformed_observations) {
      // Check that transformed observation is within range
      if (GetDistance(p.x, p.y, obs.x, obs.y) < sensor_range) {
        bool is_landmark_found = false;
        auto nearest_distance = std::numeric_limits<double>::max();
        Position nearest_landmark;
        for (const auto& landmark : landmarks) {
          auto distance = GetDistance(obs.x, obs.y, landmark.x, landmark.y);
          // Check that landmark is within range
          if (distance < sensor_range && distance < nearest_distance) {
              is_landmark_found = true;
              nearest_distance = distance;
              nearest_landmark.x = landmark.x;
              nearest_landmark.y = landmark.y;
          }
        }
        if (is_landmark_found) {
          // Update the weights of each particle using a multivariate Gaussian
          // distribution
          const auto dx = obs.x - nearest_landmark.x;
          const auto dy = obs.y - nearest_landmark.y;
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

  // Save the best particle
  Particle best_particle;
  double highest_weight = -1;
  for (const auto& p : particles_) {
    if (p.weight > highest_weight) {
      best_particle = p;
      highest_weight = p.weight;
    }
  }

  // Resample particles by replacing with probability proportional to their
  // weights
  std::discrete_distribution<> distribution(weights.begin(), weights.end());
  std::vector<Particle> new_particles;
  for (auto id = 0; id < particles_.size(); ++id) {
    auto& p = particles_.at(distribution(rng_));
    p.id = id;
    new_particles.push_back(p);
  }
  particles_ = new_particles;

  return best_particle;
}
