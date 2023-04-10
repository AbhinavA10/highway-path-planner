#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Start in lane 1, which is the center lane (0 is left and 2 is right)
  int lane = 1; // switch this as needed to change lanes
  // Start at zero velocity and gradually accelerate
  double ref_vel = 0.0; // mph

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // Read in map
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x; // Normal component to waypoint
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

  // Websocket message from simulator
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ref_vel,&lane]
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
          
          // Main car's localization Data, from simulator
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

          json msgJson;

          /**
           * The given code defines a path made up of (x,y) points that the car
           *   will visit sequentially every .02 seconds
           */

          int prev_size = previous_path_x.size();
          
          if (prev_size > 0) {
            car_s = end_path_s; // last path car was following before recalculating trajectory
          }
        
          // ------- COLLISION AVOIDANCE WITH FORWARD VEHICLE -------
          bool too_close = false; // True if too close to a car in front
          int lane_to_move_into = lane;
          vector<double> lane_costs = {0,0,0}; // Cost of moving into each lane. left, middle, right
          
          // Penalty for switching lanes
          for (int i = 0; i<lane_costs.size(); i++){
            if (i==lane){ // current lane, differential
              continue; // No penalty for staying in current lane
            }
            else{
              lane_costs[i] += 1; // Penalty to move to another lane
            }
          }

          // Penalty for extreme lanes
          if (lane==0){ // currently in left lane
            lane_costs[2] = 10000; // Impossible to move into right-most lane
          }
          else if (lane==2){ // currently in right lane
            lane_costs[0] = 10000; // Impossible to move into left-most lane
          }

          // Cost of lanes based on traffic cars
          int left_lane = lane-1;
          int right_lane = lane+1;
          // Find ref_v to use
          for (int i = 0; i < sensor_fusion.size(); i++) {
            // Check if the car [i] is in the same lane as the ego vehicle
            float d = sensor_fusion[i][6];
            if (d < (2+4*lane+2) && d > (2+4*lane-2)){ // other car is in current lane
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];    
              // Calculate the check_car's future location
              check_car_s += (double)prev_size * 0.02 * check_speed;
              if (check_car_s > car_s && (check_car_s - car_s) < 30){ // If the traffic_car is ahead and within 30 meters
                too_close = true; // ego car is too close to car in front
                lane_costs[lane] += 5; // Want to move out of this lane if possible
              } 
            }

            if (left_lane>=0 && (d < (2+4*left_lane+2) && d > (2+4*left_lane-2))){ // traffic_car is in a valid lane, 1 lane left of us
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              // Calculate the check_car's future location
              check_car_s += (double)prev_size * 0.02 * check_speed;

              if ((check_car_s > car_s && (check_car_s - car_s) < 20)){ // If the traffic_car is ahead and within 20 meters
                    lane_costs[left_lane] += 100; // Don't want to move into lane with possible collision
              } 
            }

            if (right_lane<=2 && (d < (2+4*right_lane+2) && d > (2+4*right_lane-2))){ // traffic_car is in a valid lane, 1 lane right of us
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];       
              // Calculate the check_car's future location
              check_car_s += (double)prev_size * 0.02 * check_speed;

              if ((check_car_s > car_s && (check_car_s - car_s) < 20)){ // If the traffic_car is ahead and within 20 meters
                    lane_costs[right_lane] += 100; // Don't want to move into lane with possible collision
              } 
            }
          }
          
          lane_to_move_into = std::min_element(lane_costs.begin(),lane_costs.end()) - lane_costs.begin();
          std::cout << "Left:" << lane_costs[0] << " Middle:" << lane_costs[1] << " Right:" << lane_costs[2] << std::endl;
          std::cout << "Min Cost Lane:" << lane_to_move_into << std::endl << std::endl;

          lane = lane_to_move_into; // Spline interpolation handles smoothly changing to desired lane

          // ------- SMOOTH TRAJECTORY GENERATION -------
          // Create a list of evenly spaced waypoints 30m apart
          // Interpolate those waypoints later with spline and fill it in with more points
          vector<double> ptsx;
          vector<double> ptsy;
        
          // Reference x, y, yaw states, either will be the starting point or end point of the previous path
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
        
          // if previous path is almost empty, use the car as starting reference
          if (prev_size < 2) {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - 0.5 * cos(car_yaw); // go slightly backwards in time
            double prev_car_y = car_y - 0.5 * sin(car_yaw); // go slightly backwards in time
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // Use the previous path's end point as starting reference
          else {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev , ref_x - ref_x_prev);
            // Use the two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
        
          // Add 3 points spaced evenly 30m ahead of the starting reference, for use in spline generation
          // A fixed value of d=2+4*lane enables lane following via Frenet coordinates
          vector<double> next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          // ptsx and ptsy now has a list of 5 points, to use for interpolation
        
          for (int i = 0; i < ptsx.size(); i++) {
            // shift car reference angle to 0 degrees (convert to local frame)
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
            ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
          }
        
          // Create a spline
          tk::spline s;
          // Set (x,y) anchor points to the spline (i.e. fits a spline to those points)
          s.set_points(ptsx, ptsy);
          
          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // Calculate how to break up spline points so that we travel at desired velocity
          double target_x = 30.0; // 30.0 m is the distance horizon
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y); // this is the d in the diagram
          double x_add_on = 0.0; // Related to the transformation (starting at zero)
          // Fill up the rest of path planner after filling it with previous points, will always output 50 points
          for (int i = 1; i <= 50-previous_path_x.size(); i++) {
            // Reduce speed if too close, add if no longer close
            if (too_close) {
              ref_vel -= .224; // 5 m/s^2
            } else if (ref_vel < 49.5) {
              ref_vel += .224; // no car infront, or accelerate from cold start
            }
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate x, y back to normal
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // Send path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
