// OccupancyGrid

#include "../include/OccupancyGrid.hpp"

namespace Sai2Planning
{

OccupancyGrid::OccupancyGrid(const uint32_t width, const uint32_t height, std::list<Obstacle>& obstacles, const float resolution)
{
  _width = width;
  _height = height;
  _obstacles = &obstacles;
  _resolution = resolution;
}

OccupancyGrid::~OccupancyGrid()
{
  // destructor
}

bool OccupancyGrid::isFree(const double x, const double y)
{
  // For each obstacle, check that the point (x,y) is not contained inside
  for (auto const& obs : *_obstacles) {
    // if (obs._topLeftX <= x && obs._bottomRightX >= x && obs._topLeftY <= y && obs._bottomRightY >= y) {
    //   return false;
    // }
    double distToObs = sqrt(pow(x - obs._centerX, 2) + pow(y - obs._centerY, 2));
    if (distToObs <= obs._radius) {
      return false;
    }
  }
  return true;
}

}
