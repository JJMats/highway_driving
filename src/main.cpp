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
  
  // Start in lane 1
  int lane = 1;
  
  // Set the target reference velocity
  double ref_v = 0.0; //was 49.5 MPH

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

          int prev_size = previous_path_x.size();
          
          // Sensor Fusion
          if(prev_size > 0){
            car_s = end_path_s;
          }
          
          bool too_close = false;
          
          // Run through all objects in the sensor_fusion vector and 
          //   find reference velocity to use
          for(int i=0; i < sensor_fusion.size(); ++i){
          	// Determine from the Frenet coordinates if a car is in the current lane
            float d = sensor_fusion[i][6];
            if(d < (2+4*lane+2) && d > (2+4*lane-2)){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              // If utilizing previous points, this can project an s value outwards in future time
              check_car_s += ((double)prev_size * 0.02 * check_speed);
              // Check s values greater than the ego vehicle's and s gap
              if((check_car_s > car_s) && ((check_car_s-car_s) < 30)){
                // Perform some logic to make decisions on whether the vehicle should slow down
                //   or change lanes to prevent collisions.
                //ref_v = 29.5; // MPH
                
                //****** Work on this!!
                
                too_close = true;
                
                // **** Change to better lane changing logic here
                // **** Check in Frenet coordinates if there are vehicles in the lane to possibly change to
                // **** Use cost functions
                // This will utilize the spline library to allow for smooth lane changes
                if(lane < 0){
                  lane = 0;
                }
              }
            }
          }
          
          
          if(too_close){
            ref_vel -= 0.224;
          } else if(rev_v < 49.5){
            ref_vel += 0.224;
          }
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // Create lists of widely spaced waypoints, which can be interpolated via spline fit
          vector<double> ptsx;
          vector<double> ptsy;
          
          // Define reference x, y, and yaw states
          // Either reference the starting point for the car's location or the end point of the previous path
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // If the previous list size is too small, use the car as the starting reference
          if(prev_size) < 2){
            // Select two points that form a tangential path to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // Redefine the reference state as the prior path's end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            // Append two points that form the tangential path with the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // Use Frenet coordinate system
          // Add evenly spaced points in front of the starting reference
          double point_dist = 30.0; // meters
          vector<double> next_wp0 = getXY(car_s+   point_dist , (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+(2*point_dist), (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+(3*point_dist), (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // Perform transformation back to the local car coordinates
          for(int i=0; i < ptsx.size(); ++i){
            // Shift the reference angle to 0 degrees, and change the x,y-coordinates to the origin
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            
            ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));            
          }
          
          // Create the spline and set the x,y points
          tk::spline s;
          s.set_points(ptsx, ptsy);
          
          // Define the actual x,y points used for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // Start by using all of the previous path points
          for(int i=0; i < previous_path.size(); ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate method of breaking up spline points to travel at the desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double x_add_on = 0.0;
          
          // Fill the remainder of the path planner after appending the previous points.
          int output_point_count = 50;
          for(int i=1; i <= output_point_count-previous_path_x.size(); ++i){
            
            // ****** Adding and subtracting from reference velocity here would be more efficient ******
            // ****** It may be better to run through and check to see if any of these points are too far apart,
            // ******  instead of projecting the spline onto a triangle
            double N = (target_dist / (0.02 * ref_v / 2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate coordinate system back into Frenet Coordinate System
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          /*
          // Get car to drive in straight line
          double dist_inc = 0.5;
          for (int i = 0; i < 50; ++i) {
            next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
            next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
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