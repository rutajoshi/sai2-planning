// OccupancyGrid

#include "../include/OccupancyGrid.hpp"
#include <stdexcept>

namespace Sai2Planning
{
  OccupancyGrid::OccupancyGrid(const uint32_t width, const uint32_t height, list<Obstacle>& obstacles, const float resolution)
  {
    _width = width;
    _height = height;
    _obstacles = obstacles;
    _resolution = resolution;
  }

  OccupancyGrid::~OccupancyGrid() {}

  bool OccupancyGrid::isFree(const double x, const double y)
  {
    // For each obstacle, check that the point (x,y) is not contained inside
    for (auto const& obs : _obstacles) {
      if (obs._topLeftX < x && obs._bottomRightX > x && obs._topLeftY > y && obs._bottomRightY < y) {
        return false;
      }
    }
    return true;
  }
}
