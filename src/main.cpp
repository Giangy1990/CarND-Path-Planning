#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "planner.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

const double MAX_SPEED = 49.5;
const double MAX_ACC = .224;
int lane = 1;
Planner plan(MAX_SPEED, MAX_ACC, lane);

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          // update ego state into planner class
          plan.setEgoState(car_x, car_y, car_s, car_d, car_yaw, car_speed);
          
          // Prediction : Analysing other cars positions.
          plan.findObstacles(sensor_fusion);
          //double max_speed = plan.getSpeed();
          
          // plan trajectory
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          plan.computeTrajectory(previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y,  map_waypoints_s, next_x_vals, next_y_vals);
          /*
          double speed_diff = MAX_ACC;
          
          vector<double> pnt_x, pnt_y;
          int prev_size = previous_path_x.size();
          double ref_x, ref_y, ref_yaw, ref_vel;
          if (prev_size < 2){
            // compute ref e prev points
            double prev_x = car_x - cos(car_yaw);
            double prev_y = car_y - sin(car_yaw);
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = car_yaw;
            
            // set initial points fo spline
            pnt_x.push_back(prev_x);
            pnt_y.push_back(prev_y);
            pnt_x.push_back(car_x);
            pnt_y.push_back(car_y);
            
            // setinitial speed
          	ref_vel = car_speed;
          }
          else{
            // compute ref e prev points
            double prev_x = previous_path_x[prev_size - 2];
            double prev_y = previous_path_y[prev_size - 2];
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            ref_yaw = atan2(ref_y - prev_y, ref_x - prev_x);
            
            // set initial points fo spline
            pnt_x.push_back(prev_x);
            pnt_y.push_back(prev_y);
            pnt_x.push_back(ref_x);
            pnt_y.push_back(ref_y);
            
            // setinitial speed
            double dx = sqrt(pow(ref_x - prev_x, 2) + pow(ref_y - prev_y, 2));
          	ref_vel = dx*2.24/0.02;
          }          
          // create waypoints for the spline
          int lookahead = 90;
          int step_s = 30;
          for (float delta_s = step_s; delta_s <= lookahead; delta_s += step_s){
            vector<double> next_waypoint = getXY(car_s + delta_s, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
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

          for( int i = 1; i < 50 - prev_size; i++ ) {
            // regulate the speed
            if (ref_vel < max_speed){
              // increase speed till max speed
              ref_vel = fmin(ref_vel + speed_diff, max_speed);
            }
            else{
              // decrease speed till max speed
              ref_vel = fmax(ref_vel - speed_diff, max_speed);
            }
            
            // compute trajectory samples
            double N = spline_dist/(0.02*ref_vel/2.24);
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
          */
          
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}