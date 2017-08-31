#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <uWS/uWS.h>

#include "environment.h"
#include "ptg.h"
#include "util.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::Vector3d;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  double ref_vel = 49; // mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            const int car_lane = car_d / 4;

            // Keep lane by default.
            int target_lane = car_lane;

            const auto prev_size = previous_path_x.size();

            Environment env(map_waypoints_s, map_waypoints_x, map_waypoints_y, car_s, car_d, sensor_fusion, prev_size);

            cout << "car_s:" << car_s << ",car_d:" << car_d << ",car_speed:" << car_speed << endl;
            cout << env << endl;

            auto forward_vehicle = env.lane_is_occupied(car_lane);

            const double ideal_vel = 49.5;
            const double ref_vel_inc = 0.224;

            if (forward_vehicle)
            {
                ref_vel -= ref_vel_inc;
            }
            else if (ref_vel < ideal_vel)
            {
                ref_vel += ref_vel_inc;
            }

            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            if (prev_size < 2)
            {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }
            else
            {
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            for (int i = 0; i < previous_path_x.size(); i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            const int c_path_size = 50;
            const double c_mph_to_mps = 2.24;

            if (forward_vehicle)
            {
                if (prev_size < c_path_size)
                {
                    cout << "Forward vehicle detected. id:" << forward_vehicle->id << endl;

                    auto prev_frenet = env.getFrenet(ptsx[0], ptsy[0], ref_yaw);
                    auto frenet = env.getFrenet(ptsx[1], ptsy[1], ref_yaw);

                    cout << "pts[0]:(" << ptsx[0] << ',' << ptsy[0] << "),pts[1]:(" << ptsx[1] << ',' << ptsy[1] << "),frenet_s:" << frenet[0] << ",prev_frenet_s:" << prev_frenet[0] << endl;

                    double speed_estimate = car_speed / c_mph_to_mps;
                    cout << "speed_estimate:" << speed_estimate << endl;

                    Vector3d start_s;
                    start_s << frenet[0],
                               min(speed_estimate, abs((frenet[0] - prev_frenet[0]) / 0.02)),
                               0;

                    Vector3d start_d;
                    start_d << frenet[1],
                               0, // (frenet[1] - prev_frenet[1]) / 0.02,
                               0;

                    cout << "start_s:" << start_s << ", start_d: " << start_d << endl;

                    // Pass forward vehicle.
                    VectorXd delta(6);
                    delta << 0, 0, 0, 0, 0, 0;

                    double T = 2.5;

                    Trajectory best = PTG(start_s, start_d, *forward_vehicle, delta, T, env.get_vehicles());
                    auto s_poly = best.s_poly();
                    auto d_poly = best.d_poly();

                    cout << "best.t:" << best.t << endl;

                    int path_size = best.t / 0.02;

                    for (int i = 1; i <= path_size; i++)
                    {
                        double t = i * 0.02;
                        double s = s_poly.evaluate(t) + start_s[0];
                        double d = d_poly.evaluate(t) + start_d[0];

                        cout << "s_poly:" << s << ",d_poly:" << d << endl;

                        auto xy = env.getXY(s, d);

                        next_x_vals.push_back(xy[0]);
                        next_y_vals.push_back(xy[1]);
                    }
                }
                else
                {
                    cout << "Follow prev path." << endl;
                }
            }
            else
            {
                // Keep lane.
                if (prev_size > 0)
                {
                    car_s = end_path_s;
                }

                auto next_wp0 = env.getXY(car_s + 30, (2 + 4 * target_lane));
                auto next_wp1 = env.getXY(car_s + 60, (2 + 4 * target_lane));
                auto next_wp2 = env.getXY(car_s + 90, (2 + 4 * target_lane));

                ptsx.push_back(next_wp0[0]);
                ptsx.push_back(next_wp1[0]);
                ptsx.push_back(next_wp2[0]);

                ptsy.push_back(next_wp0[1]);
                ptsy.push_back(next_wp1[1]);
                ptsy.push_back(next_wp2[1]);

                // Rotate
                for (int i = 0; i < ptsx.size(); i++)
                {
                    double shift_x = ptsx[i] - ref_x;
                    double shift_y = ptsy[i] - ref_y;

                    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                }

                tk::spline s;
                s.set_points(ptsx, ptsy);

                double target_x = 30.0;
                double target_y = s(target_x);
                double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

                double x_add_on = 0;

                for (int i = 1; i <= c_path_size - previous_path_x.size(); i++)
                {
                    double N = (target_dist / (0.02 * ref_vel / c_mph_to_mps));
                    double x_point = x_add_on + target_x / N;
                    double y_point = s(x_point);

                    x_add_on = x_point;

                    double x_ref = x_point;
                    double y_ref = y_point;

                    // Rotate back to normal after rotating above.
                    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                    x_point += ref_x;
                    y_point += ref_y;

                    next_x_vals.push_back(x_point);
                    next_y_vals.push_back(y_point);
                }
            }

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	json msgJson;

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
















































































