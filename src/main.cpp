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

  // Lane index to start in
  int current_lane = 1;
  int lane_change_counter = 0;
  
  // Target velocity, mph
  double target_velocity = 0.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&current_lane,&target_velocity,&lane_change_counter]
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

          if(previous_path_x.size() != previous_path_y.size())
          {
            std::cout << "Incorrect input packet!!!" << std::endl;
            return;
          }

          //std::cout << std::endl;
          //std::cout << "Current car s = " << car_s << std::endl;
          //std::cout << "End path s    = " << end_path_s << std::endl;
          
          int previous_path_size = previous_path_x.size();

          double current_car_s = car_s;

          if(previous_path_size > 0)
          {
            car_s = end_path_s;
          }

          bool too_close = false;
          int target_lane = current_lane;
          double other_car_speed = 0;
          double other_car_s = current_car_s;

          // go over all other cars detected around
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            // check whether the other car's d coordinate falls into the desired lane we want to drive in
            double other_car_d = sensor_fusion[i][6];
            if(other_car_d > (current_lane * 4) && other_car_d < (current_lane * 4 + 4))
            {
              // the other car is in our tagret lane

              double other_car_vx = sensor_fusion[i][3];
              double other_car_vy = sensor_fusion[i][4];
              other_car_speed = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);
              other_car_s = sensor_fusion[i][5];
              //std::cout << "Other car s now   = " << other_car_s << std::endl;
              
              // if the other car is now in front of us
              if(current_car_s < other_car_s)
              {
                // predict other car's next position at the same time as we have in our path
                double other_car_s_predicted = other_car_s + other_car_speed * 0.02 * previous_path_size;
                //std::cout << "Other car s predicted   = " << other_car_s_predicted << std::endl;
                
                // if in the future we can be in front of the other car (we go through - definite collision)
                // OR we can be behind the other car BUT the distance is unsafe (we come too close)
                double safety_distance_front = (car_speed * 1.6) * SAFETY_DISTANCE_FRONT;
                if((car_s >= other_car_s_predicted) ||
                    ((other_car_s_predicted - car_s) < safety_distance_front))
                {
                  std::cout << "Ego car is too close to the car in front. Ego car s: " << other_car_s_predicted << " m. Other car s: " << car_s << " m. Safety distance: " << safety_distance_front << " m" << std::endl;
                  // fire up the warning flag
                  too_close = true;
                  
                  // and check for possible maneuvers

                  // from the leftmost lane there's only one maneuver to check - lane change right
                  if(current_lane == LEFTMOST_LANE)
                  {
                    // check if the lane on the right is not busy
                    if(!lane_is_busy(current_car_s, car_s, car_speed, current_lane + 1, previous_path_size, sensor_fusion))
                    {
                      // change the lane to the right
                      target_lane = current_lane + 1;
                    }
                  }
                  // from the rightmost lane there's only one maneuver to check - lane change left
                  else if(current_lane == RIGHTMOST_LANE)
                  {
                    // check if the lane on the left is not busy
                    if(!lane_is_busy(current_car_s, car_s, car_speed, current_lane - 1, previous_path_size, sensor_fusion))
                    {
                      // change the lane to the left
                      target_lane = current_lane - 1;
                    }
                  }
                  // in the middle lane we can decide - left or right, depending on different criteria, e.g.
                  //   - speed of the next preceding vehicle
                  //   - average speed of all the preceding vehicles
                  //   - other rules, like passing on the left only
                  else
                  {
                    bool left_lane_busy = lane_is_busy(current_car_s, car_s, car_speed, current_lane - 1, previous_path_size, sensor_fusion);
                    bool right_lane_busy = lane_is_busy(current_car_s, car_s, car_speed, current_lane + 1, previous_path_size, sensor_fusion);
                    // if the left lane busy - there's only one chance
                    if(left_lane_busy)
                    {
                      // can we go to the right?
                      if(!right_lane_busy)
                      {
                        // change the lane to the right
                        target_lane = current_lane + 1;
                      }
                    }
                    // if the right lane busy - there's also only one chance
                    else if(right_lane_busy)
                    {
                      // can we go to the left?
                      if(!left_lane_busy)
                      {
                        // change the lane to the left
                        target_lane = current_lane - 1;
                      }
                    }
                    // if it is safe to change to either left or right lanes
                    else
                    {
                      // check how fast they go
                      double left_lane_speed = lane_speed(car_s, current_lane - 1, previous_path_size, sensor_fusion);
                      double right_lane_speed = lane_speed(car_s, current_lane + 1, previous_path_size, sensor_fusion);
                      // only if the right lane goes faster
                      if(right_lane_speed > left_lane_speed)
                      {
                        // change the lane to the right
                        target_lane = current_lane + 1;
                      }
                      // in all other cases - e.g. both left and right lanes go at the same speed or there're no leading cars
                      else
                      {
                        // prefer passing in the left lane
                        target_lane = current_lane - 1;
                      }
                    }
                  }

                  /*
                  // simpler logic to cycle through the lanes, preferring change to the left lane from current

                  // if we are not in the leftmost lane - try lane change to the left
                  if(current_lane > 0)
                  {
                    // check if the lane on the left is not busy
                    if(!lane_is_busy(current_car_s, car_s, car_speed, current_lane - 1, previous_path_size, sensor_fusion))
                    {
                      // change the lane to the left
                      target_lane = current_lane - 1;
                    }
                  }

                  // if we still didn't change the lane (either we're in the left-most lane already OR lane change to the left is not possible)
                  // AND we're not in the rightmost lane
                  if(target_lane == current_lane && current_lane < 2)
                  {
                    // check if the lane on the right is not busy
                    if(!lane_is_busy(current_car_s, car_s, car_speed, current_lane + 1, previous_path_size, sensor_fusion))
                    {
                      // change the lane to the right
                      target_lane = current_lane + 1;
                    }
                  }
                  */
                }
              }
            }
          }

          // if we're too close to the other vehicle in our lane
          if(too_close)
          {
            // slow down
            if(target_velocity > other_car_speed)
            {
              if((target_velocity - other_car_speed) < SPEED_INCREMENT_BRAKING)
              {
                target_velocity = other_car_speed;
              }
              else
              {
                target_velocity -= SPEED_INCREMENT_BRAKING;
              }
            }
          }
          // if the lane ahead is clean
          else
          {
            // consider to speed up to the limit
            if(target_velocity < SPEED_LIMIT)
            {
              if((SPEED_LIMIT - target_velocity) < SPEED_INCREMENT_ACCELERATION)
              {
                target_velocity = SPEED_LIMIT;
              }
              else
              {
                target_velocity += SPEED_INCREMENT_ACCELERATION;
              }
            }
            
            // if we drive fast enough, the lane ahead is clean (therefore no lane change maneuver was considered), try to go to the lane on the right
            if(target_velocity > MIN_CRUISING_SPEED && target_lane == current_lane && current_lane < 2)
            {
              // check if the lane on the right is not busy
              if(!lane_is_busy(current_car_s, car_s, car_speed, current_lane + 1, previous_path_size, sensor_fusion))
              {
                // change the lane to the right
                target_lane = current_lane + 1;
              }
            }
          }

        
          // lane change is only allowed every N sec
          if(lane_change_counter > 0)
          {
            lane_change_counter -= (50 - previous_path_size);
          }

          if(current_lane != target_lane && lane_change_counter <= 0)
          {
            current_lane = target_lane;
            lane_change_counter = 50 * MIN_LANE_CHANGE_INTERVAL;
          }

          // generate additional points to travel into the lane that was decided
          // using smooth spline and maintaining target velocity

          // Start with sparse way-points
          vector<double> sparse_points_x;
          vector<double> sparse_points_y;

          // calculate origin of the car coordinate system at this moment

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // fill in two initial points into the spline input
          if(previous_path_size < 1)
          {
            // if there're no points in the unfinished path:
            // just take the car position and calculate additional point in the past to make the path tangent according to the car's angle

            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            sparse_points_x.push_back(prev_car_x);
            sparse_points_x.push_back(car_x);

            sparse_points_y.push_back(prev_car_y);
            sparse_points_y.push_back(car_y);
          }
          else
          {
            // if there're points in the unfinished path:
            // take the last point as the current reference

            ref_x = previous_path_x[previous_path_size - 1];
            ref_y = previous_path_y[previous_path_size - 1];

            // if there're more than one points in the unfinished path:
            // take the pre-last point into the spline and use it for the angle calculation
            // otherwise just use the current car position
            double ref_x_prev = car_x;
            double ref_y_prev = car_y;
            if(previous_path_size >= 2)
            {
              ref_x_prev = previous_path_x[previous_path_size - 2];
              ref_y_prev = previous_path_y[previous_path_size - 2];
            }

            // calculate yaw angle at the reference car position
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            sparse_points_x.push_back(ref_x_prev);
            sparse_points_x.push_back(ref_x);

            sparse_points_y.push_back(ref_y_prev);
            sparse_points_y.push_back(ref_y);
          }

          // fill in sparse waypoints with several points ahead of the current location with a fixed interval
          for(int i = 1; i <= 3; i++)
          {
            vector<double> next_way_point = getXY(car_s + 30 * i, 2 + current_lane * 4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            sparse_points_x.push_back(next_way_point[0]);
            sparse_points_y.push_back(next_way_point[1]);
          }

          // transform the points to the car coordinate system (first shift, then rotation)
          for(int i = 0; i < sparse_points_x.size(); i++)
          {

            double shifted_x = sparse_points_x[i] - ref_x;
            double shifted_y = sparse_points_y[i] - ref_y;

            sparse_points_x[i] = shifted_x * cos(0-ref_yaw) - shifted_y * sin(0-ref_yaw);
            sparse_points_y[i] = shifted_x * sin(0-ref_yaw) + shifted_y * cos(0-ref_yaw);
          }

          // create a new spline object
          tk::spline s;

          // initialize it with sparse points
          s.set_points(sparse_points_x, sparse_points_y);

          
          // create vectors with resulting points that the car will travel
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // first fill in all the points from the previous path (not travelled yet)
          for(int i = 0; i < previous_path_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // then generate and add missing points up to total 50 points, using spline

          // pick up some reference point ahead (30 m is far enough for a second of travel at the speed no greater than 50 mph)
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_distance = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;
          
          for(int i = 1; i <= 50 - previous_path_size; i++)
          {
            // pick up the next point
            double N = target_distance / (0.02 * target_velocity / 2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            // transform the point back to the global coordinate system (first rotation, then shift)
            double x_shift = x_point * cos(ref_yaw) - y_point * sin(ref_yaw);
            double y_shift = x_point * sin(ref_yaw) + y_point * cos(ref_yaw);
            x_point = x_shift + ref_x;
            y_point = y_shift + ref_y;

            // put the point into the resulting vectors
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // Driving straight line
          /*
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          double dist_inc = 0.5;
          for (int i = 0; i < 50; ++i) {
            next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
            next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          }
          */
          // in Frenet coordinates
          /*
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          double dist_inc = 0.5;
          for (int i = 0; i < 50; ++i) {
            double next_s = car_s + (i + 1) * dist_inc;
            double next_d = 6;
            vector<double> next_xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(next_xy[0]);
            next_y_vals.push_back(next_xy[1]);
          }
          */

          // Driving a circle
          /*
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          double pos_x;
          double pos_y;
          double angle;
          int path_size = previous_path_x.size();

          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          if (path_size == 0) {
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
          } else {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];

            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
          }

          double dist_inc = 0.5;
          for (int i = 0; i < 50-path_size; ++i) {    
            next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
            next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
            pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
            pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
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