// 2D Path Planner for Mobile Base Robots

#include "../include/MobilePlanner.hpp"
#include <stdexcept>

namespace Sai2Planning
{

MobilePlanner::MobilePlanner(const Eigen::VectorXd& initial_position, const Eigen::VectorXd& goal_position)
{
  _initial_position = initial_position;
  _goal_position = goal_position;
}

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
}

void MobilePlanner::generateTrajectory()
{
  // Implement RRT trajectory generation here
  // Given initial and goal positions, and obstacles, find a path using RRT or RRT*
  // Check for collisions using isFreeMotion
}

bool MobilePlanner::goalReached()
{
  return _goal_reached;
}


}
