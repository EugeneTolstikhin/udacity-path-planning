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
#include "HighwayFSM.h"

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

  constexpr float SIMULATOR_FREQUENCY = 1.0 / 50; //0.02
  constexpr float SPEED_LIMIT = 49.5; //mph
  constexpr float LANE_WIDTH = 4.0; //m
  constexpr size_t AMOUNT_OF_POINTS_FOREHAND = 3;
  constexpr size_t AMOUNT_OF_POINTS_WHOLE_PATH = 50;
  constexpr size_t TOO_CLOSE_POINTS = 30;
  constexpr float SPEED_CHANGE = 0.224; //0.224mph = 10m/s
  constexpr size_t AMOUNT_OF_LANES = 3;

  int lane = 1;
  double ref_vel = SPEED_LIMIT;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          if (previous_path_x.size() != previous_path_y.size())
          {
            throw 1;
          }

          int prev_size = previous_path_x.size();

          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          bool too_close = false;
          bool too_close_left = false;
          bool too_close_right = false;

          bool too_close_left_ahead = false;
          bool too_close_left_behind = false;

          bool too_close_right_ahead = false;
          bool too_close_right_behind = false;

          //find ref_v to use
          for (size_t i = 0; i < sensor_fusion.size(); ++i)
          {
            float d = sensor_fusion.at(i).at(6);

            if (lane > 0)
            {
              //Check the left lane
              if ((LANE_WIDTH / 2 + LANE_WIDTH * (lane - 1) - LANE_WIDTH / 2) < d && d < (LANE_WIDTH / 2 + LANE_WIDTH * (lane - 1) + LANE_WIDTH / 2))
              {
                double vx = sensor_fusion.at(i).at(3);
                double vy = sensor_fusion.at(i).at(4);
                double check_speed = std::sqrt(std::pow(vx, 2) + std::pow(vy, 2));
                double check_car_s = sensor_fusion.at(i).at(5);

                check_car_s += SIMULATOR_FREQUENCY * prev_size * check_speed;

                if (check_car_s > car_s && check_car_s < car_s + TOO_CLOSE_POINTS)
                {
                  too_close_left_ahead = true;
                }

                if (check_car_s < car_s && car_s < check_car_s + TOO_CLOSE_POINTS / 2)
                {
                  too_close_left_behind = true;
                }

                too_close_left = too_close_left_ahead || too_close_left_behind;
              }
            }
            else
            {
              too_close_left = true;
            }

            //Check the current lane
            if ((LANE_WIDTH / 2 + LANE_WIDTH * lane - LANE_WIDTH / 2) < d && d < (LANE_WIDTH / 2 + LANE_WIDTH * lane + LANE_WIDTH / 2))
            {
              double vx = sensor_fusion.at(i).at(3);
              double vy = sensor_fusion.at(i).at(4);
              double check_speed = std::sqrt(std::pow(vx, 2) + std::pow(vy, 2));
              double check_car_s = sensor_fusion.at(i).at(5);

              check_car_s += SIMULATOR_FREQUENCY * prev_size * check_speed;

              if (check_car_s > car_s && check_car_s - car_s < TOO_CLOSE_POINTS)
              {
                too_close = true;
              }
            }

            if (lane < AMOUNT_OF_LANES - 1)
            {
              //Check the right lane
              if ((LANE_WIDTH / 2 + LANE_WIDTH * (lane + 1) - LANE_WIDTH / 2) < d && d < (LANE_WIDTH / 2 + LANE_WIDTH * (lane + 1) + LANE_WIDTH / 2))
              {
                double vx = sensor_fusion.at(i).at(3);
                double vy = sensor_fusion.at(i).at(4);
                double check_speed = std::sqrt(std::pow(vx, 2) + std::pow(vy, 2));
                double check_car_s = sensor_fusion.at(i).at(5);

                check_car_s += SIMULATOR_FREQUENCY * prev_size * check_speed;

                if (check_car_s > car_s && check_car_s < car_s + TOO_CLOSE_POINTS)
                {
                  too_close_right_ahead = true;
                }

                if (check_car_s < car_s && car_s < check_car_s + TOO_CLOSE_POINTS / 2)
                {
                  too_close_right_behind = true;
                }

                too_close_right = too_close_right_ahead || too_close_right_behind;
              }
            }
            else
            {
              too_close_right = true;
            }
          }

          if (too_close)
          {
            std::cout << "Too close ahead" << std::endl;
            if (!too_close_left)
            {
              std::cout << "Can turn left" << std::endl;
              --lane;
            }
            else if (!too_close_right)
            {
              std::cout << "Can turn right" << std::endl;
              ++lane;
            }
            else
            {
              std::cout << "Keep lane and decrease speed. Old speed = " << ref_vel << std::endl;
              ref_vel -= SPEED_CHANGE;
              std::cout << "New speed = " << ref_vel << std::endl;
            }
          }
          else if (ref_vel < SPEED_LIMIT)
          {
            std::cout << "Far away ahead" << std::endl;
            ref_vel += SPEED_CHANGE;
          }

          if (too_close_left_ahead)
          {
            std::cout << "Too close left ahead" << std::endl;
          }

          if (too_close_left_behind)
          {
            std::cout << "Too close left behind" << std::endl;
          }
          
          if (too_close_right_ahead)
          {
            std::cout << "Too close right ahead" << std::endl;
          }

          if (too_close_right_behind)
          {
            std::cout << "Too close right behind" << std::endl;
          }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_size < 2)
          {
            double prev_car_x = car_x - std::cos(car_yaw);
            double prev_car_y = car_y - std::sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x.at(prev_size - 1);
            ref_y = previous_path_y.at(prev_size - 1);

            double ref_x_prev = previous_path_x.at(prev_size - 2);
            double ref_y_prev = previous_path_y.at(prev_size - 2);

            ref_yaw = std::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          //In Frenet add evenly 30m spaces points ahead of the starting reference
          for (int i = 1; i <= AMOUNT_OF_POINTS_FOREHAND; ++i)
          {
            auto next_wp = getXY(car_s + i * TOO_CLOSE_POINTS, (LANE_WIDTH / 2 + LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(next_wp.at(0));
            ptsy.push_back(next_wp.at(1));
          }

          if (ptsx.size() != ptsy.size())
          {
            throw 2;
          }

          for (size_t i = 0; i < ptsx.size(); ++i)
          {
            double shift_x = ptsx.at(i) - ref_x;
            double shift_y = ptsy.at(i) - ref_y;

            ptsx.at(i) = shift_x * std::cos(0 - ref_yaw) - shift_y * std::sin(0 - ref_yaw);
            ptsy.at(i) = shift_x * std::sin(0 - ref_yaw) + shift_y * std::cos(0 - ref_yaw);
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          for (size_t i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x.at(i));
            next_y_vals.push_back(previous_path_y.at(i));
          }

          double target_x = static_cast<double>(TOO_CLOSE_POINTS);
          double target_y = s(target_x);
          double target_dist = std::sqrt(std::pow(target_x, 2) + std::pow(target_y, 2));

          double x_add_on = 0.0;

          for (size_t i = 1; i <= AMOUNT_OF_POINTS_WHOLE_PATH - previous_path_x.size(); ++i)
          {
            double N = target_dist / (SIMULATOR_FREQUENCY * ref_vel / (SPEED_CHANGE * 10));
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //rotate back it to normal after rotating it earlier
            x_point = x_ref * std::cos(ref_yaw) - y_ref * std::sin(ref_yaw);
            y_point = x_ref * std::sin(ref_yaw) + y_ref * std::cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

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