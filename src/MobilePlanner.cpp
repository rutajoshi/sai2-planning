// 2D Path Planner for Mobile Base Robots

#include "../include/MobilePlanner.hpp"
#include <stdexcept>

namespace Sai2Planning
{

MobilePlanner::MobilePlanner(const Eigen::VectorXd& initial_position, const Eigen::VectorXd& goal_position)
{
  // Implement here
}

MobilePlanner::MobilePlanner()
{
  // Implement here
}

void MobilePlanner::setMaxVelocity(const Eigen::VectorXd max_velocity)
{
  // Implement here
}

void MobilePlanner::setMaxVelocity(const double max_velocity)
{
  // Implement here
}

void MobilePlanner::setMaxAcceleration(const Eigen::VectorXd max_acceleration)
{
  // Implement here
}

void MobilePlanner::setMaxAcceleration(const double max_acceleration)
{
  // Implement here
}

void MobilePlanner::setMaxJerk(const Eigen::VectorXd max_jerk)
{
  // Implement here
}

void MobilePlanner::setMaxJerk(const double max_jerk)
{
  // Implement here
}

void MobilePlanner::setGoalPositionAndVelocity(const Eigen::VectorXd goal_position, const Eigen::VectorXd goal_velocity)
{
  // Implement here
}

void MobilePlanner::computeNextState(Eigen::VectorXd& next_position, Eigen::VectorXd& next_velocity, Eigen::VectorXd& next_acceleration)
{
  // Implement here
}

bool MobilePlanner::goalReached()
{
  return _goal_reached;
}


}
