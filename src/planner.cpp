#include "planner.h"

Planner::Planner(double max_speed_, double max_acc_, int lane_, float dt){
  max_speed = max_speed_;
  max_acc = max_acc_;
  ego.lane = lane_;
  time_step = dt;
  curr_state = PATH_FOLLOW;
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

double Planner::getSpeed(int target_obs){
  if (obstacles[target_obs].lane == -1){
    return max_speed;
  }
  else{
    double vx = obstacles[target_obs].vx;
    double vy = obstacles[target_obs].vy;
    double speed = sqrt(vx*vx + vy*vy);
    return speed;
  }
}

int Planner::whichLane(double car_d){
  int car_lane = -1;
  // check in which lane the car is
  if ( car_d > 0 && car_d <= 4 ) {
    car_lane = 0;
  }
  else if ( car_d > 4 && car_d <= 8 ) {
    car_lane = 1;
  }
  else if ( car_d > 8 && car_d < 12 ) {
    car_lane = 2;
  }

  return car_lane;
}

void Planner::setState(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, const vector<double>& previous_path_x, const vector<double>& previous_path_y, const vector<double> &maps_x, const vector<double> &maps_y){
  prev_size = previous_path_x.size();

  if (prev_size < 2){
    // set current state
    ego.x = car_x;
    ego.y = car_y;
    ego.yaw = car_yaw;
    ego.s = car_s;
    ego.d = car_d;
    ego.speed = car_speed;
    ego.lane = whichLane(car_d);
  }
  else{
    // compute current state
    double prev_x = previous_path_x[prev_size - 2];
    double prev_y = previous_path_y[prev_size - 2];

    ego.x = previous_path_x[prev_size - 1];
    ego.y = previous_path_y[prev_size - 1];
    ego.yaw = atan2(ego.y - prev_y, ego.x - prev_x);

    vector<double> frenet = getFrenet(ego.x, ego.y, ego.yaw, maps_x, maps_y);
    ego.s = frenet[0];
    ego.d = frenet[1];

    double dx = sqrt(pow(ego.x - prev_x, 2) + pow(ego.y - prev_y, 2));
    ego.speed = dx*2.24/time_step;
    ego.lane = whichLane(ego.d);
  }

}

double Planner::predictObstacle(const vector<double>& obs){
  double curr_vx = obs[3];
  double curr_vy = obs[4];
  double curr_s = obs[5];
  double T = prev_size*time_step;

  double speed = sqrt(curr_vx*curr_vx + curr_vy*curr_vy);

  return (curr_s + speed*T);
}

double Planner::laneChangeSpeed(obstacle_t& front, obstacle_t& rear){
  // check rear conditions
  if (rear.getSpeed() <= ego.speed){
    // there is no vehicle behind us in the target lane or it is slower or at
    // least it travels at my speed

    // check the front speed
    if (front.lane == -1){
      // no vehicle in the target lane
      return max_speed;
    }
    else{
      // only front vehicle is present
      return front.getSpeed();
    }
  }

  return 0.0;
}

bool Planner::laneChangeSpace(obstacle_t& front, obstacle_t& rear){
  bool rear_space = ((ego.s - rear.s) >= REAR_SAFETY_RANGE) || (rear.lane == -1);
  bool front_space = ((front.s - ego.s) >= FRONT_SAFETY_RANGE) || (front.lane == -1);
  return rear_space && front_space;
}

void Planner::findObstacles(const vector<vector<double> >& sensor_fusion){
  // initialize obstacles
  resetObjects();

  for ( int sensor_idx = 0; sensor_idx < sensor_fusion.size(); ++sensor_idx ) {
    double car_d = sensor_fusion[sensor_idx][6];

    // check in which lane the car is
    int car_lane = whichLane(car_d);

    // check the validity of the car_lane
    if (car_lane < 0) {
      continue;
    }

    // predict the obstacle abscissa when we reach last prev traj point
    double check_car_s = predictObstacle(sensor_fusion[sensor_idx]);

    // save obstacles
    if ( car_lane == ego.lane ) {
      // Car in our lane.
      if (check_car_s > ego.s && (obstacles[FRONT_OBSTACLE].lane == -1 || check_car_s < obstacles[FRONT_OBSTACLE].s)){
        obstacles[FRONT_OBSTACLE].lane = car_lane;
        obstacles[FRONT_OBSTACLE].s = check_car_s;
        obstacles[FRONT_OBSTACLE].d = car_d;
        obstacles[FRONT_OBSTACLE].vx = sensor_fusion[sensor_idx][3];
        obstacles[FRONT_OBSTACLE].vy = sensor_fusion[sensor_idx][4];
      }
    }
    else if ( car_lane - ego.lane == -1 ) {
      // Car left
      if (check_car_s > ego.s && (obstacles[FRONT_LEFT_OBSTACLE].lane == -1 || check_car_s < obstacles[FRONT_LEFT_OBSTACLE].s)){
        obstacles[FRONT_LEFT_OBSTACLE].lane = car_lane;
        obstacles[FRONT_LEFT_OBSTACLE].s = check_car_s;
        obstacles[FRONT_LEFT_OBSTACLE].d = car_d;
        obstacles[FRONT_LEFT_OBSTACLE].vx = sensor_fusion[sensor_idx][3];
        obstacles[FRONT_LEFT_OBSTACLE].vy = sensor_fusion[sensor_idx][4];
      }
      else if (check_car_s < ego.s && (obstacles[REAR_LEFT_OBSTACLE].lane == -1 || check_car_s > obstacles[REAR_LEFT_OBSTACLE].s)){
        obstacles[REAR_LEFT_OBSTACLE].lane = car_lane;
        obstacles[REAR_LEFT_OBSTACLE].s = check_car_s;
        obstacles[REAR_LEFT_OBSTACLE].d = car_d;
        obstacles[REAR_LEFT_OBSTACLE].vx = sensor_fusion[sensor_idx][3];
        obstacles[REAR_LEFT_OBSTACLE].vy = sensor_fusion[sensor_idx][4];
      }
    } else if ( car_lane - ego.lane == 1 ) {
      // Car right
      if (check_car_s > ego.s && (obstacles[FRONT_RIGHT_OBSTACLE].lane == -1 || check_car_s < obstacles[FRONT_RIGHT_OBSTACLE].s)){
        obstacles[FRONT_RIGHT_OBSTACLE].lane = car_lane;
        obstacles[FRONT_RIGHT_OBSTACLE].s = check_car_s;
        obstacles[FRONT_RIGHT_OBSTACLE].d = car_d;
        obstacles[FRONT_RIGHT_OBSTACLE].vx = sensor_fusion[sensor_idx][3];
        obstacles[FRONT_RIGHT_OBSTACLE].vy = sensor_fusion[sensor_idx][4];
      }
      else if (check_car_s < ego.s && (obstacles[REAR_RIGHT_OBSTACLE].lane == -1 || check_car_s > obstacles[REAR_RIGHT_OBSTACLE].s)){
        obstacles[REAR_RIGHT_OBSTACLE].lane = car_lane;
        obstacles[REAR_RIGHT_OBSTACLE].s = check_car_s;
        obstacles[REAR_RIGHT_OBSTACLE].d = car_d;
        obstacles[REAR_RIGHT_OBSTACLE].vx = sensor_fusion[sensor_idx][3];
        obstacles[REAR_RIGHT_OBSTACLE].vy = sensor_fusion[sensor_idx][4];
      }
    }
  }
}

void Planner::chooseManeuvre(){
  switch(curr_state){
    case PATH_FOLLOW:
      // look for the best lane to follow
      bestLane();

      if (ego.target_lane != ego.lane){
        // switch to lane change state
        curr_state = LANE_CHANGE;
      }
      break;

    case LANE_CHANGE:
      if (ego.target_lane == ego.lane){
        // switch to lane change state
        ego.target_speed = getSpeed(FRONT_OBSTACLE);
      }
      else if (ego.target_lane < ego.lane){
        // target lane on the left, check the max allowed speed
        ego.target_speed = getSpeed(FRONT_LEFT_OBSTACLE);
      }
      else{
        // target lane on the right, check the max allowed speed
        ego.target_speed = getSpeed(FRONT_RIGHT_OBSTACLE);
      }

      if (fabs(2 + 4*ego.target_lane - ego.d) <= .5){
        // switch to lane change state
        curr_state = PATH_FOLLOW;
      }
      break;
  }
}

void Planner::bestLane(){
  double front_speed, left_speed, right_speed;
  bool left_space, left_change, right_space, right_change;

  switch(ego.lane){
    case LEFT_LANE:
      // check if the front obstacle is in the chancge lane evaluation range
      if ((obstacles[FRONT_OBSTACLE].s - ego.s) < EVALUATE_RANGE){
        // the obstacles ahead of us is too close
        front_speed = getSpeed(FRONT_OBSTACLE);
      }
      else{
        front_speed = max_speed;
      }

      // right conditions
      right_speed = laneChangeSpeed(obstacles[FRONT_RIGHT_OBSTACLE], obstacles[REAR_RIGHT_OBSTACLE]);
      right_space = laneChangeSpace(obstacles[FRONT_RIGHT_OBSTACLE], obstacles[REAR_RIGHT_OBSTACLE]);
      right_change = right_space && (right_speed >= front_speed);

      if (right_change){
        ego.target_lane = CENTER_LANE;
        ego.target_speed = right_speed;
        break;
      }

      ego.target_speed = front_speed;
      ego.target_lane = LEFT_LANE;
      break;

    case CENTER_LANE:
      // check if the front obstacle is in the chancge lane evaluation range
      if ((obstacles[FRONT_OBSTACLE].s - ego.s) < EVALUATE_RANGE){
        // the obstacles ahead of us is too close
        front_speed = getSpeed(FRONT_OBSTACLE);
      }
      else{
        ego.target_speed = max_speed;
        ego.target_lane = CENTER_LANE;
        break;
      }

      // right conditions
      right_speed = laneChangeSpeed(obstacles[FRONT_RIGHT_OBSTACLE], obstacles[REAR_RIGHT_OBSTACLE]);
      right_space = laneChangeSpace(obstacles[FRONT_RIGHT_OBSTACLE], obstacles[REAR_RIGHT_OBSTACLE]);
      right_change = right_space && (right_speed > front_speed);

      // left conditions
      left_speed = laneChangeSpeed(obstacles[FRONT_LEFT_OBSTACLE], obstacles[REAR_LEFT_OBSTACLE]);
      left_space = laneChangeSpace(obstacles[FRONT_LEFT_OBSTACLE], obstacles[REAR_LEFT_OBSTACLE]);
      left_change = left_space && (left_speed > front_speed);

      if (left_change && right_change){
        if (left_speed > right_speed){
          ego.target_lane = LEFT_LANE;
          ego.target_speed = left_speed;
          break;
        }
        else{
          ego.target_lane = RIGHT_LANE;
          ego.target_speed = right_speed;
          break;
        }
      }
      else if (left_space && !right_space){
        ego.target_lane = LEFT_LANE;
        ego.target_speed = left_speed;
        break;
      }
      else if (!left_space && right_space){
        ego.target_lane = RIGHT_LANE;
        ego.target_speed = right_speed;
        break;
      }

      ego.target_speed = front_speed;
      ego.target_lane = CENTER_LANE;
      break;

    case RIGHT_LANE:
      // check if the front obstacle is in the chancge lane evaluation range
      if ((obstacles[FRONT_OBSTACLE].s - ego.s) < EVALUATE_RANGE){
        // the obstacles ahead of us is too close
        front_speed = getSpeed(FRONT_OBSTACLE);
      }
      else{
        front_speed = max_speed;
      }

      // left conditions
      left_speed = laneChangeSpeed(obstacles[FRONT_LEFT_OBSTACLE], obstacles[REAR_LEFT_OBSTACLE]);
      left_space = laneChangeSpace(obstacles[FRONT_LEFT_OBSTACLE], obstacles[REAR_LEFT_OBSTACLE]);
      left_change = left_space && (left_speed >= front_speed);

      if (left_change){
        ego.target_lane = CENTER_LANE;
        ego.target_speed = left_speed;
        break;
      }

      ego.target_speed = front_speed;
      ego.target_lane = RIGHT_LANE;
      break;
  }
}

void Planner::computeTrajectory(const vector<double>& previous_path_x, const vector<double>& previous_path_y, const vector<double> map_waypoints_x,
  const vector<double> map_waypoints_y, const vector<double> map_waypoints_s, vector<double>& next_x_vals, vector<double>& next_y_vals){

  vector<double> pnt_x, pnt_y;
  double ref_x, ref_y, ref_yaw, ref_speed, prev_x, prev_y;
  if (prev_size < 2){
    // compute ref e prev points
    prev_x = ego.x - cos(ego.yaw);
    prev_y = ego.y - sin(ego.yaw);
    ref_x = ego.x;
    ref_y = ego.y;
  }
  else{
    // compute ref e prev points
    prev_x = previous_path_x[prev_size - 2];
    prev_y = previous_path_y[prev_size - 2];
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
  }

  // set initial speed
  ref_speed = ego.speed;

  // set initial yaw
  ref_yaw = ego.yaw;

  // set initial points fo spline
  pnt_x.push_back(prev_x);
  pnt_y.push_back(prev_y);
  pnt_x.push_back(ref_x);
  pnt_y.push_back(ref_y);

  // create waypoints for the spline
  vector<double> next_waypoint_0 = getXY(ego.s + 30, 2 + 4*ego.target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  pnt_x.push_back(next_waypoint_0[0]);
  pnt_y.push_back(next_waypoint_0[1]);

  vector<double> next_waypoint_1 = getXY(ego.s + 60, 2 + 4*ego.target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  pnt_x.push_back(next_waypoint_1[0]);
  pnt_y.push_back(next_waypoint_1[1]);

  vector<double> next_waypoint_2 = getXY(ego.s + 90, 2 + 4*ego.target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  pnt_x.push_back(next_waypoint_2[0]);
  pnt_y.push_back(next_waypoint_2[1]);

  // change reference frame
  for (int pnt_idx = 0; pnt_idx < pnt_x.size(); ++pnt_idx){
    double dx = pnt_x[pnt_idx] - ref_x;
    double dy = pnt_y[pnt_idx] - ref_y;
    pnt_x[pnt_idx] = dx*cos(0-ref_yaw) - dy*sin(0-ref_yaw);
    pnt_y[pnt_idx] = dx*sin(0-ref_yaw) + dy*cos(0-ref_yaw);
  }

  // regulate the speed
  if (ref_speed < ego.target_speed){
    // increase speed till ref speed
    ref_speed = fmin(ref_speed + max_acc, ego.target_speed);
  }
  else{
    // decrease speed till ref speed
    ref_speed = fmax(ref_speed - max_acc, ego.target_speed);
  }

  std::cout << "current_lane = " << ego.lane << std::endl;
  std::cout << "target_lane = " << ego.target_lane << std::endl;
  std::cout << "target_speed = " << ego.target_speed << std::endl;
  std::cout << "ref_speed = " << ref_speed << std::endl;

  // create the spline
  tk::spline ref_s;
  ref_s.set_points(pnt_x, pnt_y);

  // recover the point in the prev path
  for (int pnt_idx = 0; pnt_idx < previous_path_x.size(); ++pnt_idx){
    next_x_vals.push_back(previous_path_x[pnt_idx]);
    next_y_vals.push_back(previous_path_y[pnt_idx]);
  }

  // compute the spline variation that guarantees the desired velocity
  double spline_x = 30.0;
  double spline_y = ref_s(spline_x);
  double spline_dist = sqrt(spline_x*spline_x + spline_y*spline_y);

  double x_increment = 0;

  // get reference speed
  for( int i = 1; i <= NUM_TRAJECTORY_POINTS  - prev_size; i++ ) {

    // compute trajectory samples
    double N = spline_dist/(time_step*ref_speed/2.237);
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
