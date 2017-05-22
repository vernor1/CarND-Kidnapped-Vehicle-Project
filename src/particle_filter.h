#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <random>
#include <vector>

class ParticleFilter {

public:
  struct Position {
    double x;
    double y;
    Position() : x(0), y(0) { }
    Position(double in_x, double in_y) : x(in_x), y(in_y) { }
    virtual ~Position() { }
  };
  typedef std::vector<Position> PositionSequence;

  struct DirectedPosition : public Position {
    double yaw;
    DirectedPosition() : Position(0, 0), yaw(0) { }
    DirectedPosition(double in_x, double in_y, double in_yaw)
      : Position(in_x, in_y), yaw(in_yaw) { }
  };

  struct Particle : public DirectedPosition {
    unsigned int id;
    double weight;
    Particle() : DirectedPosition(0, 0, 0), id(0), weight(0) { }
  };

  // Constructor
  // @param M Number of particles
  ParticleFilter(unsigned int num_particles,
                 const DirectedPosition& init_pos,
                 const DirectedPosition& std_pos);

  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
   *   standard deviation of yaw [rad]]
   */
//  void init(double x, double y, double theta, const DirectedPosition& std_pos);

  /**
   * prediction Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
   *   standard deviation of yaw [rad]]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */
  void Predict(double delta_t,
               const DirectedPosition& std_pos,
               double velocity,
               double yaw_rate);

  /**
   * updateWeights Updates the weights for each particle based on the likelihood of the
   *   observed measurements.
   * @param sensor_range Range [m] of sensor
   * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
   *   standard deviation of bearing [rad]]
   * @param observations Vector of landmark observations
   * @param map Map class containing map landmarks
   */
  Particle Update(double sensor_range,
                  const Position& std_landmark,
                  const PositionSequence& observations,
                  const PositionSequence& landmarks);

  /**
   * resample Resamples from the updated set of particles to form
   *   the new set of particles.
   */
//  void resample();

private:
  std::random_device random_device_;
  std::default_random_engine rng_;

  // Set of current particles
  std::vector<Particle> particles_;
};

#endif /* PARTICLE_FILTER_H_ */
