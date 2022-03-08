// Obstacle

#ifndef SAI2_PLANNING_OBSTACLE_H
#define SAI2_PLANNING_OBSTACLE_H

#include <iostream>

namespace Sai2Planning
{

class Obstacle{
public:
  /**
	 * @brief      constructor
	 *
	 * @param[in]  topLeftX      x position of top left corner
   * @param[in]  topLeftY      y position of top left corner
   * @param[in]  bottomRightX  x position of bottom right corner
   * @param[in]  bottomRightY  y position of bottom right corner
	 */
	// Obstacle(double topLeftX, double topLeftY, double bottomRightX, double bottomRightY);

	/**
	 * @brief      constructor
	 *
	 * @param[in]  centerX      x position of center
   * @param[in]  centerY      y position of center
   * @param[in]  radius  			radius
	 */
	Obstacle(double centerX, double centerY, double radius);

  /**
	 * @brief      destructor
	 */
	~Obstacle();

  /**
   * Member variables
   */
  // double _topLeftX;
  // double _topLeftY;
  // double _bottomRightX;
  // double _bottomRightY;
	double _centerX;
	double _centerY;
	double _radius;
};

} /* namespace Sai2Planning */

#endif //SAI2_PLANNING_OBSTACLE_H
