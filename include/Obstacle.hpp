// Obstacle

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
	Obstacle(const double topLeftX, const double topLeftY, const double bottomRightX, const double bottomRightY);

  /**
	 * @brief      destructor
	 */
	~Obstacle();

  /**
   * Member variables
   */
  double _topLeftX = 0.0;
  double _topLeftY = 0.0;
  double _bottomRightX = 0.0;
  double _bottomRightY = 0.0;
}
