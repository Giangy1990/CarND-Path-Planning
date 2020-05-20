#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <limits>
#include <math.h>
#include <iostream>
#include "spline.h"
#include "helpers.h"

using std::vector;

class Planner{
  public:
  Planner(double max_speed_, double max_acc_, int lane_);
  void computeTrajectory(const vector<double>& previous_path_x, const vector<double>& previous_path_y, const vector<double> map_waypoints_x,
  const vector<double> map_waypoints_y, const vector<double> map_waypoints_s, vector<double>& next_x_vals, vector<double>& next_y_vals);
  
  void findObstacles(const vector<vector<double> >& sensor_fusion, const vector<double> map_waypoints_x, const vector<double> map_waypoints_y);
  double getSpeed();
  void setEgoState(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);
  
  private:
  double max_speed;
  double max_acc;
  typedef struct{
    int lane = -1;
    double d = -1;
    double s = std::numeric_limits<double>::max();
    double vx = 0;
    double vy = 0;
  } obstacle_t;
  vector<obstacle_t> obstacles;
  typedef struct{
    int lane;
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    double ref_speed;
  } ego_t;
  ego_t ego;
  int prev_size;
  float time_step;
  
  void resetObjects();
  vector<double> predictObstacle(const vector<double>& obs, const vector<double> &maps_x, const vector<double> &maps_y);
};
#endif // PLANNER_H