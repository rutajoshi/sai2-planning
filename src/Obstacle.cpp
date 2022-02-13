// Obstacle

#include "../include/Obstacle.hpp"
#include <stdexcept>

namespace Sai2Planning
{
  Obstacle::Obstacle(const double topLeftX, const double topLeftY, const double bottomRightX, const double bottomRightY)
  {
    _topLeftX = topLeftX;
    _topLeftY = topLeftY;
    _bottomRightX = bottomRightX;
    _bottomRightY = bottomRightY;
  }

  Obstacle::~Obstacle(){}
}
