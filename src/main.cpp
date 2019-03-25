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
  double ref_accel = 0.0; // reference acceleration rate
  const double SPEED_LIMIT = 49.5; // * 0.44704; // meters per second
  //double safe_following_distance = 1.49; // meters per meters/sec   //0.45; // meters per mph
  double safe_following_distance = 30.0; // meters
  const double MAX_ACCEL = 10.0; // m/s^2
  const double MAX_JERK = 10.0; // m/s^3
  const double UPDATE_RATE = 0.02; // 50 Hz
  double max_accel_increment = MAX_JERK * UPDATE_RATE; // m/s^2, limits acceleration change between update events to maximum allowable jerk
    
  h.onMessage([&ref_v,&ref_accel,&SPEED_LIMIT,&safe_following_distance,&MAX_ACCEL,&MAX_JERK,&max_accel_increment,
               &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane]
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
          
          // Convert car_speed to meters per second
          //car_speed *= 0.44704;
          std::cout << std::endl << "Vehicle speed (m/s): " << car_speed << "; x,y: " << car_x << "," << car_y << "; s,d: " << car_s << "," << car_d << "; yaw: " << car_yaw << std::endl;

          // Convert reference velocity back to meters per second
          //ref_v *= 0.44704;
          
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
            std::cout << "Adjusted car_s: " << car_s << std::endl;
          }
          
          //bool too_close = false;
          
          double ref_v_adjustment = 0.0;
          
          // Run through all objects in the sensor_fusion vector and 
          //   find each object's location and reference velocity to
          //   compare to the ego vehicle
          std::cout << "Sensor_fusion.size(): " << sensor_fusion.size() << std::endl;
          
          //double dist_to_next_followed_vehicle = std::numeric_limits<double>::max();
          //int next_followed_vehicle_index = -1;
          //double next_followed_vehicle_speed = 0.0;

          bool object_in_lane = false;
          bool object_to_left = false;
          bool object_to_right = false;
          
          for(int i=0; i < sensor_fusion.size(); ++i){
            //if(object_in_lane && object_to_left && object_to_right){
              // Short circuit if all flags are set
            //  break;
            //}
            
            // Determine from the Frenet d coordinate which lane the object is in
            float d = sensor_fusion[i][6];
            int object_lane_id = -1;

            // Check to see if the object is in a lane
            if(d < 0.0 || d >= 12.0){
              continue;
            }

            if(d >= 0.0 && d < 4.0){
              object_lane_id = 0; // Left lane
            }else if(d >= 4.0 && d < 8.0){
              object_lane_id = 1; // Center lane
            }else{
              object_lane_id = 2; // Right lane
            }

            // Determine from the Frenet d coordinate if a car is in the current lane
            //if(d < (2+4*lane+2) && d > (2+4*lane-2)){
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy); // meters per second
            double check_car_s = sensor_fusion[i][5]; // meters
            double s_diff = check_car_s - car_s;
              
            std::cout << "Vehicle id: " << sensor_fusion[i][0] << "; speed: " << check_speed << "; s,d position: " << check_car_s << "," << d << "; lane id: " << object_lane_id << "; s_diff: " << s_diff << std::endl;
              
            // If utilizing previous points, this can project an s value outwards in future time
            check_car_s += ((double)prev_size * 0.02 * check_speed);
            //double safe_following_distance = 30.0; // meters
            double s_diff_2 = check_car_s - car_s;
            std::cout << "s_diff: " << s_diff << "; s_diff_2:" << s_diff_2 << std::endl;
              
            if(object_lane_id == lane && !object_in_lane){
              // The object is in the same lane as the ego vehicle, verify that it is in the forward direction
              //  and that it is further away than a safe following distance.
              object_in_lane = s_diff_2 > 0 && s_diff_2 < safe_following_distance; // ? true : false;
              std::cout << "Object in lane: " << object_in_lane << std::endl;
            }else if(object_lane_id < lane && !object_to_left){
              // The object is in the lane to the left of the ego vehicle. Determine if it is beyond a safe following distance
              //   fore and aft.
              object_to_left = abs(s_diff_2) < safe_following_distance; // ? true : false;
              std::cout << "Object to left: " << object_to_left << std::endl;
            }else if(object_lane_id > lane && !object_to_right){
              // The object is in the lane to the right of the ego vehicle. Determine if it is beyond a safe following distance
              //   fore and aft.
              object_to_right = abs(s_diff_2) < safe_following_distance; // ? true : false;
              std::cout << "Object to right: " << object_to_right << std::endl;
            }
          }

          double velocity_diff = 0.0;
          // *** Build in cost function here
          if(object_in_lane){
            if(!object_to_left && lane > 0){
              // Change lanes to the left
              lane--;
            }else if(!object_to_right && lane < 2){
              // Change lanes to the right
              lane++;
            }else{
              //velocity_diff -= MAX_ACCEL;
              velocity_diff -= max_accel_increment;
            }
          }else{
            if(lane != 1){
              if((lane == 0 && !object_to_right) || (lane == 2 && !object_to_left)){
                // Return to center lane for more lane change options
                lane = 1;
              }
            }
            
            // Increase speed to meet the speed limit
            if(ref_v < SPEED_LIMIT){
              //velocity_diff += MAX_ACCEL;
              velocity_diff += max_accel_increment;
            }
          }
              

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
          if(prev_size < 2){
            // Select two points that form a tangential path to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // Redefine the reference state as the prior path's end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
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
          for(int i = 0; i < ptsx.size(); ++i){
            // Shift the reference angle to 0 degrees, and change the x,y-coordinates to the origin
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));            
          }
          
          if(ptsx.size() > 0 && ptsy.size() > 0){
            // Create the spline and set the x,y points
            tk::spline s;
            s.set_points(ptsx, ptsy);

            // Define the actual x,y points used for the planner
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // Start by using all of the previous path points
            for(int i = 0; i < prev_size; ++i){
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
            for(int i = 1; i < output_point_count - prev_size; ++i){

              // ****** Adding and subtracting from reference velocity here would be more efficient ******
              // ****** It may be better to run through and check to see if any of these points are too far apart,
              // ******  instead of projecting the spline onto a triangle

              ref_v += velocity_diff;
              if (ref_v >= SPEED_LIMIT){
                ref_v = SPEED_LIMIT;
              }else if(ref_v < max_accel_increment){
                ref_v = max_accel_increment;
              }
              //ref_v = ref_v >= SPEED_LIMIT ? SPEED_LIMIT : ref_v;              

              double N = target_dist / (0.02 * ref_v / 2.23694);
              double x_point = x_add_on + target_x / N;
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
        }
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