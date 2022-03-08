// Obstacle

#include "../include/Obstacle.hpp"
#include <stdexcept>

namespace Sai2Planning
{

Obstacle::Obstacle(double centerX, double centerY, double radius)
{
  _centerX = centerX;
  _centerY = centerY;
  _radius = radius;
}

// Obstacle::Obstacle(double topLeftX, double topLeftY, double bottomRightX, double bottomRightY)
// {
//   _topLeftX = topLeftX;
//   _topLeftY = topLeftY;
//   _bottomRightX = bottomRightX;
//   _bottomRightY = bottomRightY;
// }

Obstacle::~Obstacle()
{
  // destructor
}

}
