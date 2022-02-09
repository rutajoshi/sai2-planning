// 2D Path Planner for Mobile Base Robots

#include <iostream>

namespace Sai2Planning
{

class MobilePlanner{
public:

  /**
	 * @brief      constructor
	 *
	 * @param[in]  initial_position   Initial position of the robot in 2d
   * @param[in]  goal_position      Goal position of the robot in 2d
	 */
	MobilePlanner(const Eigen::VectorXd& initial_position, const Eigen::VectorXd& goal_position);

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
	 * @brief      Calculates the next desired position and velocity for the next step
	 *
	 * @param      next_position  The desired position in the next step
	 * @param      next_velocity  The desired velocity in the next step
	 */
	void computeNextState(Eigen::VectorXd& next_position, Eigen::VectorXd& next_velocity, Eigen::VectorXd& next_acceleration);

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

};

}
