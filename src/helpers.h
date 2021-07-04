#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// multiplier used to maintain safety distance (speed in km/h is multiplied by this value to obtain rough distance in meters)
// rule of thumb is kmh/2 which leads to pretty defencive driving style
// lower values result in more aggressive driving
// distance_front is used to calculate how close the ego car will approach other car from behind
#define SAFETY_DISTANCE_FRONT (1.0/3)
// distance_front_overtake is used to calculate how close the ego car will approach other car from behind in case of a lane change
#define SAFETY_DISTANCE_FRONT_OVERTAKE (1.0/4)
// distance_back is used to calculate how much the ego car will leave to the vehicle coming from behind in case of a lane change
#define SAFETY_DISTANCE_BACK_OVERTAKE (1.0/5)

// maximum cruising speed, mph
#define SPEED_LIMIT 49.8

// speed increment used for acceleration, mph
#define SPEED_INCREMENT_ACCELERATION 0.224 * 5
// speed increment used for braking, mph
#define SPEED_INCREMENT_BRAKING 0.224 * 7

// minimum speed considered for the lane change to the right, mph (German "Rechtsfahrgebot")
// to turn off this behaviour - simply set this value to speed limit or above, so that the car never reaches it
//#define MIN_CRUISING_SPEED 40
#define MIN_CRUISING_SPEED 50

// minimal interval between lane changes, seconds (to avoid shaking from lane to lane)
#define MIN_LANE_CHANGE_INTERVAL  3

#define LEFTMOST_LANE 0
#define RIGHTMOST_LANE 2

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// checks if ego vehicle will collide with another vehicle in a target lane, given number of prediction steps and a safety range
// returns true if collision is detected, false otherwise
bool lane_is_busy(double ego_car_s, double ego_car_s_predicted, double ego_car_speed, int target_lane, int prediction_steps, vector<vector<double>> sensor_fusion) {
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    // check whether the pther car's d coordinate falls into the desired lane we want to drive in
    double other_car_d = sensor_fusion[i][6];
    if(other_car_d > (target_lane * 4) && other_car_d < (target_lane * 4 + 4))
    {
      std::cout << std::endl << "Checking car " << sensor_fusion[i][0] << " in the lane " << target_lane << std::endl;

      // the other car is in our tagret lane

      double other_car_vx = sensor_fusion[i][3];
      double other_car_vy = sensor_fusion[i][4];
      double other_car_speed = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);
      double other_car_s = sensor_fusion[i][5];
      std::cout << "Other car s now   = " << other_car_s << std::endl;
      
      // predict other car's next position at the same time as we have in our path
      double other_car_s_predicted = other_car_s + other_car_speed * 0.02 * prediction_steps;
      std::cout << "Other car s predicted   = " << other_car_s_predicted << std::endl;
      if(other_car_s_predicted > ego_car_s_predicted && (other_car_s_predicted - ego_car_s_predicted) < (ego_car_speed * 3.6 * SAFETY_DISTANCE_FRONT_OVERTAKE))
      {
        std::cout << "Other car will be in front but too close" << std::endl;
        return true;
      }
      if(ego_car_s_predicted > other_car_s_predicted  && (ego_car_s_predicted - other_car_s_predicted) < (other_car_speed * 3.6 * SAFETY_DISTANCE_BACK_OVERTAKE))
      {
        std::cout << "Ego car will be in front but too close" << std::endl;
        return true;
      }
      if(other_car_s_predicted == ego_car_s_predicted)
      {
        std::cout << "Ego car will collide with the other car" << std::endl;
        return true;
      }
    }
  }
  std::cout << "Lane change is safe" << std::endl;
  return false;
}

// finds closest vehicle in front of the ego vehicle
// returns speed of the closest vehicle
double lane_speed(double ego_car_s_predicted, int target_lane, int prediction_steps, vector<vector<double>> sensor_fusion) {
  
  double min_distance_to_the_other_car = -1;
  double closest_car_speed = SPEED_LIMIT;
  
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    // check whether the pther car's d coordinate falls into the desired lane we want to drive in
    double other_car_d = sensor_fusion[i][6];
    if(other_car_d > (target_lane * 4) && other_car_d < (target_lane * 4 + 4))
    {
      std::cout << std::endl << "Checking car " << sensor_fusion[i][0] << " in the lane " << target_lane << std::endl;

      // the other car is in our tagret lane

      double other_car_vx = sensor_fusion[i][3];
      double other_car_vy = sensor_fusion[i][4];
      double other_car_speed = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);
      double other_car_s = sensor_fusion[i][5];
      std::cout << "Other car s now   = " << other_car_s << std::endl;
      
      // predict other car's next position at the same time as we have in our path
      double other_car_s_predicted = other_car_s + other_car_speed * 0.02 * prediction_steps;
      std::cout << "Other car s predicted   = " << other_car_s_predicted << std::endl;
      if(other_car_s_predicted > ego_car_s_predicted)
      {
        double distance_to_the_other_car = other_car_s_predicted - ego_car_s_predicted;
        if(min_distance_to_the_other_car < 0 || distance_to_the_other_car < min_distance_to_the_other_car)
        {
          min_distance_to_the_other_car = distance_to_the_other_car;
          closest_car_speed = other_car_speed * 2.24;
        }
      }
    }
  }
  // dont return speed more than the limit, even if some cars are going faster
  if(closest_car_speed > SPEED_LIMIT)
  {
    closest_car_speed = SPEED_LIMIT;
  }

  std::cout << "Speed of the closest car in the lane # " << target_lane << " is " << closest_car_speed << std::endl;
  return closest_car_speed;
}


#endif  // HELPERS_H