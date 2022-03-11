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
  uint32_t num_points = (uint32_t) (distance / (_occupancy->_resolution / 10.0));
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
      // With probability goal_bias, set x_rand to goal position
      x_rand = _goal_position;
    } else {
      // Otherwise, sample randomly from state space
      double rangeSize = 0;
      for (j = 0; j < SPATIAL_DIM; j++) {
        rangeSize = (_statespace_hi(j) - _statespace_lo(j));
        x_rand(j) = (double) (((double) rand() / (RAND_MAX)) * rangeSize + _statespace_lo(j));
      }
    }

    x_near_idx = findNearest(V, x_rand, n);
    x_near = V.row(x_near_idx);
    x_new = steerTowards(x_near, x_rand, eps);

    if (isFreeMotion(x_near, x_new)) {
      std::cout << x_new(0) << "," << x_new(1) << "\n";
      for (j = 0; j < SPATIAL_DIM; j++) {
        V(n, j) = x_new(j);
      }
      P(n) = x_near_idx;
      n += 1;

      if ((x_new - _goal_position).norm() < 0.05) { // REPLACE WITH CONSTANT
        _goal_reached = true;
        success = true;
        break;
      }
    }
  }

  Eigen::MatrixXd path(n, SPATIAL_DIM); // TODO(Ruta): the final path is less than length n
  if (success == true) {
    // Store and return the path
    std::cout << "n = " << n << "\n";
    uint32_t curr_idx = n - 1;
    for (uint32_t k = 0; k < n - 1; k++) {
      for (uint32_t j = 0; j < SPATIAL_DIM; j++) {
        path(n - k - 1, j) = V(curr_idx, j);
      }
      // path.row(n - k - 1) << V(curr_idx);
      curr_idx = P(curr_idx);
    }
    for (uint32_t m = 0; m < SPATIAL_DIM; m++) {
      path(0, m) = _initial_position(m);
    }
    return path;
  }

  return path; // empty if success == false
}

Eigen::MatrixXd MobilePlanner::computeSmoothedTrajectory(Eigen::MatrixXd path, float V_des, float alpha, float dt)
{
    const uint32_t SPATIAL_DIM = 2;
    // Smooth the path returned by generateTrajectory here

    std::cout << "Printing the path before smoothing\n";
    for (int i = 0; i < path.rows(); i++) {
      std::cout << path.row(i) << "\n";
    }

    // 1) get the estimated times to reach each point given
    std::vector<double> times;
    for (int i = 0; i < path.rows(); i++) {
      if (i == 0) {
        times.push_back(0.0);
        // std::cout << 0.0 << "\n";
        continue;
      }
      float delta_x = (path.row(i) - path.row(i-1)).norm();
      float delta_t = delta_x / V_des;
      // std::cout << times[i-1] + delta_t << "\n";
      times.push_back(times[i-1] + delta_t);
    }

    // 2) get a new set of times, in equal increments of dt
    int timesteps = (int) (times[path.rows()-1] / dt);
    std::vector<double> t_smoothed;
    for (int i = 0; i < timesteps; i++) {
      t_smoothed.push_back(i*dt);
    }

    // 3) Get the lists of x and y values from the path
    std::vector<double> x;
    std::vector<double> y;
    for (int i = 0; i < path.rows(); i++) {
      x.push_back(path.row(i)(0));
      y.push_back(path.row(i)(1));
    }

    // 4) Get the equations for each dimension by interpolation
    tk::spline x_spline(times, x);
    tk::spline y_spline(times, y);

    // 5) Evaluate the equations to get a smooth path
    Eigen::MatrixXd smoothed(timesteps, SPATIAL_DIM);
    for (int i = 0; i < timesteps; i++) {
      Eigen::VectorXd row_smooth(SPATIAL_DIM);
      double t = t_smoothed[i];
      row_smooth << x_spline(t), y_spline(t);
      smoothed(i, 0) = row_smooth(0);
      smoothed(i, 1) = row_smooth(1);
    }

    // 6) return the smoothed path
    return smoothed;
}

bool MobilePlanner::goalReached()
{
  return _goal_reached;
}


}
