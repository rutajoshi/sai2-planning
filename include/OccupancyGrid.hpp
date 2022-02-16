// OccupancyGrid

#ifndef SAI2_PLANNING_OCCGRID_H
#define SAI2_PLANNING_OCCGRID_H

#include <iostream>
#include "../include/Obstacle.hpp"
#include <list>

namespace Sai2Planning
{

class OccupancyGrid{
public:
  /**
	 * @brief      constructor
	 *
	 * @param[in]  width      Width of occupancy grid (x dimension)
   * @param[in]  height     Height of occupancy grid (y dimension)
   * @param[in]  obstacles  List of obstacles for this robot to keep track of
	 */
	OccupancyGrid(const uint32_t width, const uint32_t height, std::list<Obstacle>& obstacles, const float resolution);

  /**
	 * @brief      destructor
	 */
	~OccupancyGrid();

  /**
   * @brief      Returns whether (x,y) is free (not in collision with an obstacle)
   *
   * @param[in] x   width position
   * @param[in] y   height position
   *
   * @return     true if position is free, false otherwise
   */
  bool isFree(const double x, const double y);

  /**
   * Member variables
   */
  uint32_t _width;
  uint32_t _height;
  std::list<Obstacle>* _obstacles;
  float _resolution;

};

} /* namespace Sai2Planning */

#endif //SAI2_PLANNING_OCCGRID_H
