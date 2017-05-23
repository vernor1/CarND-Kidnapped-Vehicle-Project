#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <random>
#include <vector>

class ParticleFilter {

public:
  // Contains the position data: x and y coordinates
  struct Position {
    double x;
    double y;
    Position() : x(0), y(0) { }
    Position(double in_x, double in_y) : x(in_x), y(in_y) { }
    virtual ~Position() { }
  };
  typedef std::vector<Position> PositionSequence;

  // Contains the directed position data: Yaw, x and y coordinates
  struct DirectedPosition : public Position {
    double yaw;
    DirectedPosition() : Position(0, 0), yaw(0) { }
    DirectedPosition(double in_x, double in_y, double in_yaw)
      : Position(in_x, in_y), yaw(in_yaw) { }
  };

  // Contains the particle data: Id, weight, yaw, x and y coordinates
  struct Particle : public DirectedPosition {
    unsigned int id;
    double weight;
    Particle() : DirectedPosition(0, 0, 0), id(0), weight(0) { }
  };

  // Ctor
  // @param[in] num_particles  Number of particles
  // @param[in] init_pos  Initial noisy position
  // @param[in] std_pos  Standard deviation of the initial position 
  ParticleFilter(unsigned int num_particles,
                 const DirectedPosition& init_pos,
                 const DirectedPosition& std_pos);

  // Dtor
  virtual ~ParticleFilter() { }

  // Predicts the state for the next time step using the process model.
  // @param[in] delta_t  Time between time step t and t+1 in measurements [s]
  // @param[in] std_pos  Standard deviation of x [m], y [m] and yaw [rad]
  // @param velocity  Velocity of car from t to t+1 [m/s]
  // @param yaw_rate  Yaw rate of car from t to t+1 [rad/s]
  void Predict(double delta_t,
               const DirectedPosition& std_pos,
               double velocity,
               double yaw_rate);

   // Updates the weights for each particle based on the likelihood of the
   // observed measurements and resamples the updated set of particles to form
   // the new set of particles.
   // @param sensor_range  Range [m] of sensor
   // @param std_landmark  Standard deviation of x [m] and y [m] of observations
   // @param observations  Sequence of landmark observations
   // @param landmarks  Sequence of landmark positions
   // @return  The best particle
  Particle Update(double sensor_range,
                  const Position& std_landmark,
                  const PositionSequence& observations,
                  const PositionSequence& landmarks);

private:
  std::random_device random_device_;
  std::default_random_engine rng_;

  // Set of current particles
  std::vector<Particle> particles_;
};

#endif /* PARTICLE_FILTER_H_ */
