#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <limits>
#include <math.h>
#include <iostream>
#include "spline.h"
#include "helpers.h"

#define PATH_FOLLOW 1U
#define LANE_CHANGE 2U

#define FRONT_LEFT_OBSTACLE 0U
#define FRONT_OBSTACLE 1U
#define FRONT_RIGHT_OBSTACLE 2U
#define REAR_LEFT_OBSTACLE 3U
#define REAR_RIGHT_OBSTACLE 4U

#define EVALUATE_RANGE 30

using std::vector;

class Planner{
  public:
  Planner(double max_speed_, double max_acc_, int lane_, float dt);
  void setState(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, const vector<double>& previous_path_x, const vector<double>& previous_path_y, const vector<double> &maps_x, const vector<double> &maps_y);
  void findObstacles(const vector<vector<double> >& sensor_fusion);
  void chooseManeuvre();
  void computeTrajectory(const vector<double>& previous_path_x, const vector<double>& previous_path_y, const vector<double> map_waypoints_x,
  const vector<double> map_waypoints_y, const vector<double> map_waypoints_s, vector<double>& next_x_vals, vector<double>& next_y_vals);
  
  
  private:
  int curr_state;
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
    int target_lane;
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
  
  int whichLane(double car_d);
  double getSpeed(int target_obs);
  void resetObjects();
  double predictObstacle(const vector<double>& obs);
  double laneChangeSpeed(obstacle_t& front, obstacle_t& rear);
  bool laneChangeSpace(obstacle_t& front, obstacle_t& rear);
};
#endif // PLANNER_H