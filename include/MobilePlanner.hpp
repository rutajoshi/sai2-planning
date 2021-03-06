// 2D Path Planner for Mobile Base Robots

#include <iostream>
#include <stdexcept>
#include <math.h>
#include <Eigen/Dense>
#include "../include/OccupancyGrid.hpp"
#include "../include/spline.h"

#ifndef SAI2_PLANNING_MOBILEPLANNER_H
#define SAI2_PLANNING_MOBILEPLANNER_H

namespace Sai2Planning
{

class MobilePlanner{
public:

  /**
	 * @brief      constructor
	 *
	 * @param[in]	 statespace_lo			Vector2d lower bound on state space
	 * @param[in]	 statespace_hi			Vector2d upper bound on state space
	 * @param[in]  initial_position   Initial position of the robot in 2d
   * @param[in]  goal_position      Goal position of the robot in 2d
	 * @param[in]  goal_velocity      Goal velocity of the robot in 2d
   * @param[in]  occupancy          OccupancyGrid for this planner
	 */
	MobilePlanner(Eigen::VectorXd& statespace_lo, Eigen::VectorXd& statespace_hi, Eigen::VectorXd& initial_position, Eigen::VectorXd& goal_position, Eigen::VectorXd& goal_velocity, OccupancyGrid& occupancy);

  /**
	 * @brief      destructor
	 */
	~MobilePlanner();

	/**
	 * @brief      Sets the maximum velocity in each direction.
	 *
	 * @param[in]  max_velocity  Vector of the maximum velocity per direction
	 */
	void setMaxVelocity(const Eigen::VectorXd max_velocity);

	/**
	 * @brief      Sets the maximum velocity.
	 *
	 * @param[in]  max_velocity  Scalar of the maximum velocity in all directions
	 */
	void setMaxVelocity(const double max_velocity);

	/**
	 * @brief      Sets the maximum acceleration in each direction.
	 *
	 * @param[in]  max_acceleration  Vector of the maximum acceleration
	 */
	void setMaxAcceleration(const Eigen::VectorXd max_acceleration);

	/**
	 * @brief      Sets the maximum acceleration.
	 *
	 * @param[in]  max_acceleration  Scalar of the maximum acceleration in all directions
	 */
	void setMaxAcceleration(const double max_acceleration);

	/**
	 * @brief      Sets the maximum jerk in each direction.
	 *
	 * @param[in]  max_jerk  Vector of the maximum jerk
	 */
	void setMaxJerk(const Eigen::VectorXd max_jerk);

	/**
	 * @brief      Sets the maximum jerk.
	 *
	 * @param[in]  max_jerk  Scalar of the maximum jerk in all directions
	 */
	void setMaxJerk(const double max_jerk);

	/**
	 * @brief      Sets the goal position and velocity
	 *
	 * @param[in]  goal_position  The goal position
	 * @param[in]  goal_velocity  The goal velocity
	 */
	void setGoalPositionAndVelocity(const Eigen::VectorXd goal_position, const Eigen::VectorXd goal_velocity);

  /**
	 * @brief      Function to know if path from positionA to positionB is collision free
   *
   * @param[in]  position_a   Starting position
   * @param[in]  position_b   Ending position
	 *
	 * @return     true if the path is collision free, false otherwise
	 */
  bool isFreeMotion(const Eigen::VectorXd position_a, const Eigen::VectorXd position_b);

	/**
	 * @brief			Function to find the index of the candidate state, from a given list, such that
	 * 						the steering distance from x to that state is minimized
	 *
	 * @param[in]	candidateStates		List of candidate states
	 * @param[in]	queryState				query state
	 * @param[in] numStates					The number of states in candidateStates to check
	 *
	 * @return integer index of the candidate state that has least steering distance to x
	 */
	uint32_t findNearest(Eigen::MatrixXd& candidateStates, const Eigen::VectorXd queryState, uint32_t numStates);

	/**
	 * @brief			Steers from x1 towards x2 along the shortest path (subject to robot
   *      			dynamics). Returns x2 if the length of this shortest path is less than
   *      			eps, otherwise returns the point at distance eps along the path from
   *      			x1 to x2.
	 *
	 * @param[in]	x1	start state
	 * @param[in] x2 	target state
	 * @param[in] eps maximum steering distance
	 *
	 * @return the position (state) that is eps distance from x1 toward x2, or x2 if distance < eps
	 */
	 Eigen::VectorXd steerTowards(const Eigen::VectorXd x1, const Eigen::VectorXd x2, double eps);

	/**
	 * @brief      Uses RRT to calculate a trajectory
	 *
	 * @param[in]	eps 				maximum steering distance
	 * @param[in] max_iters		maximum number of RRT iterations (early termination
   *            						is possible when a feasible solution is found)
	 * @param[in] goal_bias		probability during each iteration of setting
   *            						x_rand = this._goal_position (instead of uniformly randly sampling
   *            						from the state space)
	 *
	 * @return								path to goal
	 */
	Eigen::MatrixXd generateTrajectory(double eps, uint32_t max_iters, double goal_bias);

	/**
	 * @brief			Smoothes the trajectory using B-splines
	 *
	 * @param[in] path			Eigen::MatrixXd path (N, SPATIAL_DIM)
	 * @param[in] V_des 		Desired nominal velocity (use as heuristic to assign nominal times to points in initial path)
	 * @param[in] alpha			Smoothing parameter for interpolation
	 * @param[in] dt				Timestep for final smoothed path
	 */
	 Eigen::MatrixXd computeSmoothedTrajectory(Eigen::MatrixXd path, float V_des, float alpha, float dt);

	/**
	 * @brief      Function to know if the goal position and velocity is reached
	 *
	 * @return     true if the goal state is reached, false otherwise
	 */
	bool goalReached();

  /**
   * Member variables
   */
  bool _goal_reached = false;
	Eigen::VectorXd& _statespace_lo;
	Eigen::VectorXd& _statespace_hi;
  Eigen::VectorXd& _initial_position; //Eigen::VectorXd::Zero(2)
  Eigen::VectorXd& _goal_position;
  Eigen::VectorXd& _goal_velocity;

  Eigen::VectorXd _max_velocity;
  Eigen::VectorXd _max_acceleration;
  Eigen::VectorXd _max_jerk;

  Eigen::VectorXd _current_position;
  Eigen::VectorXd _current_velocity;
  Eigen::VectorXd _current_acceleration;
  Eigen::VectorXd _current_jerk;

  OccupancyGrid* _occupancy = NULL;

};

} /* namespace Sai2Planning */

#endif //SAI2_PLANNING_MOBILEPLANNER_H
