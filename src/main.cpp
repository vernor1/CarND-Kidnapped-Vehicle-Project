#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include "particle_filter.h"

// Local Types
// -----------------------------------------------------------------------------

typedef std::normal_distribution<double> Distribution;

struct Control {
  // Velocity [m/s]
  double velocity;
  // Yaw rate [rad/s]
  double yaw_rate;
};
typedef std::vector<Control> ControlSequence;

typedef std::vector<ParticleFilter::DirectedPosition> DirectedPositionSequence;

// Local Constants
// -----------------------------------------------------------------------------

// File names
const char kLandmarksFileName[] = "data/map_data.txt";
const char kControlDataFileName[] = "data/control_data.txt";
const char kGroundTruthDataFileName[] = "data/gt_data.txt";
const char kObservationFileNamePrefix[] = "data/observation/observations_";

// Number of time steps before accuracy is checked by grader
enum { kTimeStepsBeforeLockRequired = 100 };

// Max allowed completion time to pass [sec]
const double kMaxCompletionTime = 45;

// Max allowed translation error to pass [m]
const double kMaxTranslationError = 1;

// Max allowed yaw error [rad]
const double kMaxYawError = 0.05;

// Time elapsed between measurements [sec]
const double kDeltaT = 0.1;

// Sensor range [m]
const double kSensorRange = 50;

// Sigmas - just an estimate, usually comes from uncertainty of sensor, but if
// you used fused data from multiple sensors, it's difficult to find these
// uncertainties directly:
// GPS measurement uncertainty x [m], y [m], yaw [rad]
const ParticleFilter::DirectedPosition kSigmaPos(0.3, 0.3, 0.01);
// Observation measurement uncertainty x [m], y [m]
const ParticleFilter::Position kSigmaObs(0.3, 0.3);

// Number of particles
enum { kNumParticles = 50 };

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Reads the map of lanmarks from a file.
// @param[in] file_name  Name of the file containing the map data
// @return  The sequence of lanmark positions
ParticleFilter::PositionSequence GetLandmarks(const std::string& file_name) {
  std::ifstream ifs(file_name, std::ifstream::in);
  if (!ifs) {
    throw std::invalid_argument("Could not open landmark file");
  }
  ParticleFilter::PositionSequence landmarks;
  // Run over each single line
  std::string line;
  while (std::getline(ifs, line)) {
    ParticleFilter::Position landmark;
    // Landmark Id is not used in this project
    int id;
    // Read data from current line
    std::istringstream iss(line);
    iss >> landmark.x >> landmark.y >> id;
    // Add to landmark sequence
    landmarks.push_back(landmark);
  }
  return landmarks;
}

// Reads control measurements from a file.
// @param[in] file_name  Name of the file containing the control measurements
// @return  The sequence of control measurements
ControlSequence GetControlMeasurements(const std::string& file_name) {
  std::ifstream ifs(file_name, std::ifstream::in);
  if (!ifs) {
    throw std::invalid_argument("Could not open control data file");
  }
  ControlSequence control_measurements;
  // Run over each single line
  std::string line;
  while (std::getline(ifs, line)) {
    Control control;
    std::istringstream iss(line);
    iss >> control.velocity >> control.yaw_rate;
    // Add to sequence of control measurements
    control_measurements.push_back(control);
  }
  return control_measurements;
}

// Reads ground truth data from a file.
// @param[in] file_name  Name of the file containing the ground truth data
// @return  The sequence of ground truth positions
DirectedPositionSequence GetGroundTruthData(const std::string& file_name) {

  std::ifstream ifs(file_name, std::ifstream::in);
  if (!ifs) {
    throw std::invalid_argument("Could not open ground truth file");
  }

  DirectedPositionSequence positions;
  // Run over each single line
  std::string line;
  while (std::getline(ifs, line)) {
    ParticleFilter::DirectedPosition position;
    std::istringstream iss(line);
    iss >> position.x >> position.y >> position.yaw;
    // Add to sequence of ground truth data
    positions.push_back(position);
  }
  return positions;
}

// Reads observation data from a file and adds Gausian noise to it.
// @param[in] rng  Random number generator
// @param[in] dist_x  Distribution of coordinate x
// @param[in] dist_y  Distribution of coordinate y
// @param[in] observation_id  Identifier of the observation
// @return  The sequence of observation positions
template<typename G>
ParticleFilter::PositionSequence GetNoisyObservations(
  G& rng,
  Distribution& dist_x,
  Distribution& dist_y,
  unsigned int observation_id) {

  // Read in landmark observations for current time step
  std::ostringstream file_name;
  file_name << kObservationFileNamePrefix << std::setfill('0')
            << std::setw(6) << observation_id + 1 << ".txt";
  // Open file of landmark measurements
  std::ifstream ifs(file_name.str(), std::ifstream::in);
  if (!ifs) {
    throw std::invalid_argument("Could not open observation file");
  }

  ParticleFilter::PositionSequence observations;
  // Run over each single line
  std::string line;
  while (std::getline(ifs, line)) {
    std::istringstream iss(line);
    ParticleFilter::Position observation;
    // Read the observation
    iss >> observation.x >> observation.y;
    // Simulate the addition of noise to noiseless observation data
    observation.x += dist_x(rng);
    observation.y += dist_y(rng);
    observations.push_back(observation);
  }
  return observations;
}

// Calcualtes the deviation error between a position and its ground truth.
// @param[in] ground_truth  The ground truth position
// @param[in] position  The position
// @return  The position error
ParticleFilter::DirectedPosition GetError(
  const ParticleFilter::DirectedPosition& ground_truth,
  const ParticleFilter::DirectedPosition& position) {

  ParticleFilter::DirectedPosition error(
    std::fabs(position.x - ground_truth.x),
    std::fabs(position.y - ground_truth.y),
    std::fabs(position.yaw - ground_truth.yaw));
  error.yaw = std::fmod(error.yaw, 2.0 * M_PI);
  if (error.yaw > M_PI) {
    error.yaw = 2.0 * M_PI - error.yaw;
  }
  return error;
}

// main
// -----------------------------------------------------------------------------

int main() {
  // Start timer
  auto start_time = std::clock();
  // Noise generation
  std::random_device random_device;
  std::default_random_engine rng(random_device());
  Distribution dist_obs_x(0, kSigmaObs.x);
  Distribution dist_obs_y(0, kSigmaObs.y);

  // Read landmarks, control data, ground truth data
  ParticleFilter::PositionSequence landmarks;
  ControlSequence control_measurements;
  DirectedPositionSequence ground_truth_data;
  try {
    landmarks = GetLandmarks(kLandmarksFileName);
    control_measurements = GetControlMeasurements(kControlDataFileName);
    ground_truth_data = GetGroundTruthData(kGroundTruthDataFileName);
  }
  catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  // Run particle filter
  std::unique_ptr<ParticleFilter> particle_filter;
  ParticleFilter::DirectedPosition total_error;
  ParticleFilter::DirectedPosition mean_error;

  for (auto step = 0; step < control_measurements.size(); ++step) {
    std::cout << "Time step: " << step << std::endl;
    // Initialize particle filter if this is the first time step.
    if (particle_filter) {
      // Predict the vehicle's next state
      particle_filter->Predict(kDeltaT,
                               kSigmaPos,
                               control_measurements[step - 1].velocity,
                               control_measurements[step - 1].yaw_rate);
    }
    else {
      Distribution dist_init_x(ground_truth_data[step].x, kSigmaPos.x);
      Distribution dist_init_y(ground_truth_data[step].y, kSigmaPos.y);
      Distribution dist_init_yaw(ground_truth_data[step].yaw, kSigmaPos.yaw);
      particle_filter.reset(new ParticleFilter(
        kNumParticles,
        ParticleFilter::DirectedPosition(dist_init_x(rng),
                                         dist_init_y(rng),
                                         dist_init_yaw(rng)),
        kSigmaPos));
    }

    ParticleFilter::Particle best_particle;
    try {
      // Update the weights and resample
      best_particle = particle_filter->Update(
        kSensorRange,
        kSigmaObs,
        GetNoisyObservations(rng, dist_obs_x, dist_obs_y, step),
        landmarks);
    }
    catch (const std::exception& e) {
      std::cout << e.what() << std::endl;
      return -1;
    }

    // Calculate and output the average weighted error of the particle filter
    // over all time steps so far
    auto position_error = GetError(ground_truth_data[step], best_particle);
    total_error.x += position_error.x;
    total_error.y += position_error.y;
    total_error.yaw += position_error.yaw;
    mean_error.x = total_error.x / (step + 1);
    mean_error.y = total_error.y / (step + 1);
    mean_error.yaw = total_error.yaw / (step + 1);

    // Print the cumulative weighted error
    std::cout << "Cumulative mean weighted error: x " << mean_error.x
              << " y " << mean_error.y
              << " yaw " << mean_error.yaw << std::endl;

    // If the error is too high, say so and then exit
    if (step >= kTimeStepsBeforeLockRequired) {
      if (mean_error.x > kMaxTranslationError) {
        std::cout << "x error, " << mean_error.x
                  << " is larger than the maximum allowed error, "
                  << kMaxTranslationError << std::endl;
        return -1;
      }
      if (mean_error.y > kMaxTranslationError) {
        std::cout << "y error, " << mean_error.y
                  << " is larger than the maximum allowed error, "
                  << kMaxTranslationError << std::endl;
        return -1;
      }
      if (mean_error.yaw > kMaxYawError) {
        std::cout << "yaw error, " << mean_error.yaw
                  << " is larger than the maximum allowed error, "
                  << kMaxYawError << std::endl;
        return -1;
      }
    }
  }

  // Output the completion time for the filter
  auto stop_time = std::clock();
  double completion_time = (stop_time - start_time)
                           / static_cast<double>(CLOCKS_PER_SEC);
  std::cout << "Completion time (sec): " << completion_time << std::endl;

  // Print success if accuracy and completion time are sufficient
  if (completion_time < kMaxCompletionTime) {
    std::cout << "Success! The particle filter passed!" << std::endl;
  }
  else {
    std::cout << "Completion time " << completion_time
              << " is larger than the maximum allowed completion time, "
              << kMaxCompletionTime << std::endl;
    return -1;
  }

  return 0;
}
