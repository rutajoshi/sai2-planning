// 2D Path Planner for Mobile Base Robots

#include "../include/MobilePlanner.hpp"

namespace Sai2Planning
{

MobilePlanner::MobilePlanner(Eigen::VectorXd& statespace_lo, Eigen::VectorXd& statespace_hi, Eigen::VectorXd& initial_position, Eigen::VectorXd& goal_position, Eigen::VectorXd& goal_velocity, OccupancyGrid& occupancy)
  : _statespace_lo(statespace_lo), _statespace_hi(statespace_hi), _initial_position(initial_position), _goal_position(goal_position), _goal_velocity(goal_velocity), _occupancy(&occupancy)
{}

MobilePlanner::~MobilePlanner()
{
  // Implement here
}

void MobilePlanner::setMaxVelocity(const Eigen::VectorXd max_velocity)
{
  _max_velocity = max_velocity;
}

void MobilePlanner::setMaxVelocity(const double max_velocity)
{
  setMaxVelocity(max_velocity * Eigen::VectorXd::Ones(3));
}

void MobilePlanner::setMaxAcceleration(const Eigen::VectorXd max_acceleration)
{
  _max_acceleration = max_acceleration;
}

void MobilePlanner::setMaxAcceleration(const double max_acceleration)
{
  setMaxAcceleration(max_acceleration * Eigen::VectorXd::Ones(3));
}

void MobilePlanner::setMaxJerk(const Eigen::VectorXd max_jerk)
{
  _max_jerk = max_jerk;
}

void MobilePlanner::setMaxJerk(const double max_jerk)
{
  setMaxJerk(max_jerk * Eigen::VectorXd::Ones(3));
}

void MobilePlanner::setGoalPositionAndVelocity(const Eigen::VectorXd goal_position, const Eigen::VectorXd goal_velocity)
{
  _goal_position = goal_position;
  _goal_velocity = goal_velocity;
}

bool MobilePlanner::isFreeMotion(const Eigen::VectorXd position_a, const Eigen::VectorXd position_b)
{
  // Use obstacle representation to identify whether the path from a to b requires going through an obstacle
  // Use occupancy grids for this high level trajectory - only for the later part do you have to worry about moving obstacles
  float distance = (position_a - position_b).norm();
  uint32_t num_points = (uint32_t) (distance / _occupancy->_resolution);
  float x_step = abs(position_b[0]-position_a[0]) / num_points;
  float slope = (position_b[1]-position_a[1]) / (position_b[0]-position_a[0]);
  float intercept = position_a[1] - slope*position_a[0];
  for (int i = 0; i < num_points; i++) {
    float waypoint_x = position_a[0] + x_step;
    float waypoint_y = slope*waypoint_x + intercept;
    if (!_occupancy->isFree(waypoint_x, waypoint_y)) {
      return false;
    }
  }
  return true;
}

uint32_t MobilePlanner::findNearest(Eigen::MatrixXd& candidateStates, const Eigen::VectorXd queryState, uint32_t numStates)
{
  // Iterate through the list of candidate states and return the index of the one that has the least
  // Euclidean distance to the queryState
  double min_dist = 1000000000000.0;
  uint32_t min_index = 0;
  // uint32_t i = 0;
  for (uint32_t i = 0; i < numStates; i++) {
    Eigen::VectorXd candidate = candidateStates.row(i);
    double dist = (candidate - queryState).norm();
    if (dist < min_dist) {
      min_dist = dist;
      min_index = i;
    }
    // i += 1;
  }
  return min_index;
}

Eigen::VectorXd MobilePlanner::steerTowards(const Eigen::VectorXd x1, const Eigen::VectorXd x2, double eps)
{
  Eigen::VectorXd motion = x2 - x1;
  double dist = motion.norm();
  if (dist < eps) {
    return x2;
  }
  return x1 + motion/dist * eps;
}

Eigen::MatrixXd MobilePlanner::generateTrajectory(double eps, uint32_t max_iters, double goal_bias)
{
  // Implement RRT trajectory generation here
  // Given initial and goal positions, and obstacles, find a path using RRT or RRT*
  // Check for collisions using isFreeMotion
  const uint32_t SPATIAL_DIM = 2;
  Eigen::MatrixXd V(max_iters + 1, SPATIAL_DIM);
  uint32_t j;
  for (j = 0; j < SPATIAL_DIM; j++) {
    V(0, j) = _initial_position(j);
  }
  uint32_t n = 1;

  Eigen::VectorXd P(max_iters+1);
  bool success = false;
  Eigen::VectorXd x_rand(SPATIAL_DIM);
  Eigen::VectorXd x_near(SPATIAL_DIM);
  Eigen::VectorXd x_new(SPATIAL_DIM);
  uint32_t x_near_idx = 0;

  for (uint32_t k = 0; k < max_iters; k++) {
    double r = ((double) rand() / (RAND_MAX));
    if (r < goal_bias) {
      x_rand = _goal_position;
    } else {
      double rangeSize = 0;
      for (j = 0; j < SPATIAL_DIM; j++) {
        rangeSize = (_statespace_hi(j) - _statespace_lo(j));
        x_rand(j) = (double) (rand()*rangeSize + _statespace_lo(j)) / rangeSize;
      }
    }

    x_near_idx = findNearest(V, x_rand, n);
    x_near = V.row(x_near_idx);
    x_new = steerTowards(x_near, x_rand, eps);

    if (isFreeMotion(x_near, x_new)) {
      for (j = 0; j < SPATIAL_DIM; j++) {
        V(n, j) = x_new(j);
      }
      P(n) = x_near_idx;
      n += 1;

      if ((x_new - _goal_position).norm() < 0.05) { // REPLACE WITH CONSTANT
        success = true;
        break;
      }
    }
  }

  Eigen::MatrixXd path(n, SPATIAL_DIM);
  if (success == true) {
    // Store and return the path
    uint32_t curr_idx = n - 1;
    for (uint32_t k = 0; k < n; k++) {
      path(n - k - 1) = V(curr_idx);
      curr_idx = P(curr_idx);
    }
    return path;
  }

  return path; // empty if success == false

  // for k in range(max_iters):
  //     if random.random() < goal_bias:
  //         x_rand = self.x_goal
  //     else:
  //         x_rand = np.random.uniform(self.statespace_lo, self.statespace_hi)
  //     x_near_idx = self.find_nearest(V[:n], x_rand)
  //     x_near = V[x_near_idx]
  //     x_new = self.steer_towards(x_near, x_rand, eps)
  //     if self.is_free_motion(self.obstacles, x_near, x_new):
  //         V[n] = x_new
  //         P[n] = x_near_idx
  //         n += 1
  //         if np.allclose(x_new, self.x_goal):
  //             success = True
  //             break
  // if success:
  //     self.path = []
  //     curr_idx = n - 1
  //     while curr_idx != 0:
  //         self.path.append(V[curr_idx])
  //         curr_idx = P[curr_idx]
  //     self.path.append(self.x_init)
  //     self.path.reverse()



}

bool MobilePlanner::goalReached()
{
  return _goal_reached;
}


}
