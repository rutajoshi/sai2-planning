#include <iostream>
#include <fstream>
#include <string>
#include "../include/MobilePlanner.hpp"
#include "../include/OccupancyGrid.hpp"

using namespace Sai2Planning;
using namespace std;

const string obs_file = "../../pytest/obstacles.txt";
const string grid_file = "../../pytest/grid.txt";
const string path_file = "../../pytest/rrt_path.txt";

int main() {

  // 1. Make a list of obstacles and write to file
  std::list<Obstacle> obstacles;
  Obstacle obsA = Obstacle(2.5, 10.0, 2.5); //Obstacle(0, 7, 5, 13);
  Obstacle obsB = Obstacle(5.0, 3.5, 3.0); //Obstacle(3, 2, 7, 5);
  Obstacle obsC = Obstacle(12.0, 4.5, 4.5); //Obstacle(10, 0, 14, 9);
  obstacles.push_back(obsA);
  obstacles.push_back(obsB);
  obstacles.push_back(obsC);

  ofstream obsfile(obs_file);
  if(obsfile.is_open()) {
    for (const Obstacle &obstacle : obstacles) {
      obsfile << obstacle._centerX << "," << obstacle._centerY << "," << obstacle._radius << "\n";
    }
    obsfile.close();
  } else {
    std::cerr<<"Unable to open file";
  }

  // 2. Initialize an occupancy grid and write to file
  OccupancyGrid grid = OccupancyGrid(14, 14, obstacles, 1.0);

  ofstream gridfile(grid_file);
  if(gridfile.is_open()) {
    gridfile << grid._width << "," << grid._height << "," << grid._resolution << "\n";
    gridfile.close();
  } else {
    std::cerr<<"Unable to open file";
  }

  // 3. Initialize a mobile planner
  Eigen::VectorXd statespace_lo(2);
  statespace_lo << 0, 0;
	Eigen::VectorXd statespace_hi(2);
  statespace_hi << 14, 14;
  Eigen::VectorXd initial_position(2);
  initial_position << 1, 1;
  Eigen::VectorXd goal_position(2);
  goal_position << 12, 12;
  Eigen::VectorXd goal_velocity(2);
  goal_velocity << 0.5, 0.5;
  MobilePlanner planner = MobilePlanner(statespace_lo, statespace_hi, initial_position, goal_position, goal_velocity, grid);

  // 4. Call generateTrajectory
  Eigen::MatrixXd path = planner.generateTrajectory(1, 10000, 0.7);
  float V_des = 0.4;
  float alpha = 0.5;
  float dt = 0.1;
  Eigen::MatrixXd smoothed = planner.computeSmoothedTrajectory(path, V_des, alpha, dt);

  // 5. Go through rows and save positions to a txt file
  ofstream myfile("../../pytest/rrt_path.txt");
  if(myfile.is_open()) {
    for (int i = 0; i < smoothed.rows(); i++) {
      std::cout << smoothed.row(i)(0) << "," << smoothed.row(i)(1) << "\n";
      myfile << smoothed.row(i)(0) << "," << smoothed.row(i)(1) << "\n";
    }
    myfile.close();
  } else {
    std::cerr<<"Unable to open file";
  }

  // 6. Plot the map and the path in python

  cout << "Tests Completed" << endl;
}
