#include "planner.h"

Planner::Planner(double max_speed_, double max_acc_, int lane_, float dt){
  max_speed = max_speed_;
  max_acc = max_acc_;
  ego.lane = lane_;
  time_step = dt;
  obstacle_t obs;
  for (int count = 0; count < 5; ++count){
    obstacles.push_back(obs);
  }
}

void Planner::resetObjects(){
  obstacle_t obs;
  for (int idx = 0; idx < obstacles.size(); ++idx){
    obstacles[idx] = obs;
  }
}

void Planner::findObstacles(const vector<vector<double> >& sensor_fusion, const vector<double> map_waypoints_x, const vector<double> map_waypoints_y){
  // initialize obstacles
  resetObjects();
  
  for ( int sensor_idx = 0; sensor_idx < sensor_fusion.size(); ++sensor_idx ) {
    // predixt = [x, y, s, d]
    vector<double> predict = predictObstacle(sensor_fusion[sensor_idx], map_waypoints_x, map_waypoints_y);
    double car_d = predict[3];
    int car_lane = -1;
    
    // check in which lane the car is
    if ( car_d > 0 && car_d < 4 ) {
      car_lane = 0;
    }
    else if ( car_d > 4 && car_d < 8 ) {
      car_lane = 1;
    }
    else if ( car_d > 8 && car_d < 12 ) {
      car_lane = 2;
    }
    // check the validity of the car_lane
    if (car_lane < 0) {
      continue;
    }
    
    // check that the obstacle is in a distance range when we reach last prev traj point
    double check_car_s = predict[2];
    double validity_range = 30;
    if (fabs(ego.s - check_car_s) < validity_range){
      continue;
    }
    
    // save obstacles
    if ( car_lane == ego.lane ) {
      // Car in our lane.
      if (check_car_s > ego.s && (obstacles[1].lane == -1 || check_car_s < obstacles[1].s)){
        obstacles[1].lane = car_lane;
        obstacles[1].s = check_car_s;
        obstacles[1].d = car_d;
        obstacles[1].vx = sensor_fusion[sensor_idx][3];
        obstacles[1].vy = sensor_fusion[sensor_idx][4];
      }
    }
    else if ( car_lane - ego.lane == -1 ) {
      // Car left
      if (check_car_s > ego.s && (obstacles[0].lane == -1 || check_car_s < obstacles[0].s)){
        obstacles[0].lane = car_lane;
        obstacles[0].s = check_car_s;
        obstacles[0].d = car_d;
        obstacles[0].vx = sensor_fusion[sensor_idx][3];
        obstacles[0].vy = sensor_fusion[sensor_idx][4];
      }
      else if (check_car_s < ego.s && (obstacles[3].lane == -1 || check_car_s > obstacles[3].s)){
        obstacles[3].lane = car_lane;
        obstacles[3].s = check_car_s;
        obstacles[3].d = car_d;
        obstacles[3].vx = sensor_fusion[sensor_idx][3];
        obstacles[3].vy = sensor_fusion[sensor_idx][4];
      }
    } else if ( car_lane - ego.lane == 1 ) {
      // Car right
      if (check_car_s > ego.s && (obstacles[2].lane == -1 || check_car_s < obstacles[2].s)){
        obstacles[2].lane = car_lane;
        obstacles[2].s = check_car_s;
        obstacles[2].d = car_d;
        obstacles[2].vx = sensor_fusion[sensor_idx][3];
        obstacles[2].vy = sensor_fusion[sensor_idx][4];
      }
      else if (check_car_s < ego.s && (obstacles[4].lane == -1 || check_car_s > obstacles[4].s)){
        obstacles[4].lane = car_lane;
        obstacles[4].s = check_car_s;
        obstacles[4].d = car_d;
        obstacles[4].vx = sensor_fusion[sensor_idx][3];
        obstacles[4].vy = sensor_fusion[sensor_idx][4];
      }
    }
  }
}

double Planner::getSpeed(){
  if (obstacles[1].lane == -1){
    return max_speed;
  }
  else{
    double vx = obstacles[1].vx;
    double vy = obstacles[1].vy;
    double speed = sqrt(vx*vx + vy*vy);
    return speed;
  }
}

void Planner::setState(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, int size){
  ego.x = car_x;
  ego.y = car_y;
  ego.s = car_s;
  ego.d = car_d;
  ego.yaw = car_yaw;
  ego.speed = car_speed;
  prev_size = size;
}

void Planner::computeTrajectory(const vector<double>& previous_path_x, const vector<double>& previous_path_y, const vector<double> map_waypoints_x,
  const vector<double> map_waypoints_y, const vector<double> map_waypoints_s, vector<double>& next_x_vals, vector<double>& next_y_vals){
  vector<double> pnt_x, pnt_y;
  int prev_size = previous_path_x.size();
  double ref_x, ref_y, ref_yaw, ref_vel, prev_x, prev_y;
  if (prev_size < 2){
    // compute ref e prev points
    prev_x = ego.x - cos(ego.yaw);
    prev_y = ego.y - sin(ego.yaw);
    ref_x = ego.x;
    ref_y = ego.y;
    ref_yaw = ego.yaw;

    // setinitial speed
    ref_vel = ego.speed;
  }
  else{
    // compute ref e prev points
    prev_x = previous_path_x[prev_size - 2];
    prev_y = previous_path_y[prev_size - 2];
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    ref_yaw = atan2(ref_y - prev_y, ref_x - prev_x);

    // setinitial speed
    double dx = sqrt(pow(ref_x - prev_x, 2) + pow(ref_y - prev_y, 2));
    ref_vel = dx*2.24/time_step;
  }
  
  // set initial points fo spline
  pnt_x.push_back(prev_x);
  pnt_y.push_back(prev_y);
  pnt_x.push_back(ref_x);
  pnt_y.push_back(ref_y);
  
  // create waypoints for the spline
  int lookahead = 90;
  int step_s = 30;
  for (float delta_s = step_s; delta_s <= lookahead; delta_s += step_s){
    vector<double> next_waypoint = getXY(ego.s + delta_s, 2 + 4*ego.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    pnt_x.push_back(next_waypoint[0]);
    pnt_y.push_back(next_waypoint[1]);
  }


  // change reference frame
  for (int pnt_idx = 0; pnt_idx < pnt_x.size(); ++pnt_idx){
    double dx = pnt_x[pnt_idx] - ref_x;
    double dy = pnt_y[pnt_idx] - ref_y;
    pnt_x[pnt_idx] = dx*cos(-ref_yaw) - dy*sin(-ref_yaw);
    pnt_y[pnt_idx] = dx*sin(-ref_yaw) + dy*cos(-ref_yaw);
  }

  // create the spline
  tk::spline ref_s;
  ref_s.set_points(pnt_x, pnt_y);

  // recover the point in the prev path
  for (int pnt_idx = 0; pnt_idx < previous_path_x.size(); ++pnt_idx){
    next_x_vals.push_back(previous_path_x[pnt_idx]);
    next_y_vals.push_back(previous_path_y[pnt_idx]);
  }

  // compute the spline variation that guarantees the desired velocity
  double spline_x = 50;
  double spline_y = ref_s(spline_x);
  double spline_dist = sqrt(spline_x*spline_x + spline_y*spline_y);

  double x_increment = 0;
  
  // get reference speed
  ego.ref_speed = getSpeed();
  
  for( int i = 1; i < 50 - prev_size; i++ ) {
    // regulate the speed
    if (ref_vel < ego.ref_speed){
      // increase speed till max speed
      ref_vel = fmin(ref_vel + max_acc, ego.ref_speed);
    }
    else{
      // decrease speed till max speed
      ref_vel = fmax(ref_vel - max_acc, ego.ref_speed);
    }

    // compute trajectory samples
    double N = spline_dist/(time_step*ref_vel/2.24);
    double x_point = x_increment + spline_x/N;
    double y_point = ref_s(x_point);

    x_increment = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  } 
}

vector<double> predictObstacle(const vector<double>& obs, const vector<double> &maps_x, const vector<double> &maps_y){
  double curr_x = obs[1];
  double curr_y = obs[2];
  double curr_vx = obs[3];
  double curr_vy = obs[4];
  double T = prev_size*time_step;
  
  double new_x = curr_x + T*curr_vx;
  double new_y = curr_y + T*curr_vy;
  double theta = atan2(new_y - curr_y, new_x - curr_x);
  vector<double> frenet = getFrenet(new_x, new_y, theta, maps_x, maps_y);
  
  return {new_x, new_y, frenet[0], frenet[1]};
}