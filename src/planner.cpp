#include "environment.h"
#include "perspective.h"
#include "planner.h"
#include "ptg.h"
#include "spline.h"
#include "util.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::Vector3d;

Planner::Planner(const Map& map)
  : map(map),
    ref_vel(49)
{
}

std::vector<Point> smooth_trajectory(const Trajectory& trajectory,
                                     const Environment& env,
                                     const Perspective& perspective,
                                     const vector<double>& prev_tail_ptsx,
                                     const vector<double>& prev_tail_ptsy)
{
    auto s_poly = trajectory.s_poly();
    auto d_poly = trajectory.d_poly();

    vector<double> downsampled_ptsx(prev_tail_ptsx);
    vector<double> downsampled_ptsy(prev_tail_ptsy);

    static const double DOWNSAMPLE_SECONDS = 1.0;

    int downsample_size = trajectory.t / DOWNSAMPLE_SECONDS;

    for (int i = 0; i <= downsample_size; i++)
    {
        double t = i * DOWNSAMPLE_SECONDS;
        double s = s_poly.evaluate(t);
        double d = d_poly.evaluate(t);
        auto xy = env.getXY(s, d);

        downsampled_ptsx.push_back(xy[0]);
        downsampled_ptsy.push_back(xy[1]);
    }

    // Transform to car perspective.
    perspective.transform_to_car(downsampled_ptsx, downsampled_ptsy);

    for (int i = downsampled_ptsx.size() - 1; i > 0; i--)
    {
        if (downsampled_ptsx[i] < downsampled_ptsx[i - 1])
        {
            downsampled_ptsx.erase(downsampled_ptsx.begin() + i);
            downsampled_ptsy.erase(downsampled_ptsy.begin() + i);
        }
    }

    // Make spline out of downsampled points.
    tk::spline spline;
    spline.set_points(downsampled_ptsx, downsampled_ptsy);

    vector<double> car_ptsx;
    vector<double> car_ptsy;

    static const double PATH_SEGMENT_SECONDS = 0.02;

    int path_size = trajectory.t / PATH_SEGMENT_SECONDS;

    // Sample (s,d) points at expected segments.
    for (int i = 1; i <= path_size; i++)
    {
        double t = i * PATH_SEGMENT_SECONDS;
        double s = s_poly.evaluate(t);
        double d = d_poly.evaluate(t);

        // Convert (s,d) to (x,y)
        auto xy = env.getXY(s, d);

        // Transform (x,y) to car perspective.
        double x = perspective.transform_to_car(xy[0], xy[1]).x;

        // Use spline to smooth x-value.
        double y = spline(x);

        car_ptsx.push_back(x);
        car_ptsy.push_back(y);
    }

    // Transform to global coordinates.
    perspective.transform_to_global(car_ptsx, car_ptsy);

    vector<Point> points;
    for (int i = 0; i < car_ptsx.size(); i++)
    {
        points.emplace_back(Point{car_ptsx[i], car_ptsy[i]});
    }

    return points;
}

Path Planner::plan_path(const Telemetry& tel)
{
    double car_x = tel.car_x;
    double car_y = tel.car_y;
    double car_s = tel.car_s;
    double car_d = tel.car_d;
    double car_yaw = tel.car_yaw;
    double car_speed = tel.car_speed;

    // Previous path data given to the Planner
    const auto& previous_path_x = tel.previous_path_x;
    const auto& previous_path_y = tel.previous_path_y;

    // Previous path's end s and d values
    double end_path_s = tel.end_path_s;
    double end_path_d = tel.end_path_d;

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    const auto& sensor_fusion = tel.sensor_fusion;

    const int car_lane = car_d / 4;

    // Keep lane by default.
    int target_lane = car_lane;

    const auto prev_size = previous_path_x.size();

    Environment env(map.waypoints_s, map.waypoints_x, map.waypoints_y, car_s, car_d, sensor_fusion, prev_size);

    // cout << "car_s:" << car_s << ",car_d:" << car_d << ",car_speed:" << car_speed << endl;
    // cout << env << endl;

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

    Path path;

    for (int i = 0; i < previous_path_x.size(); i++)
    {
        path.next_x_vals.push_back(previous_path_x[i]);
        path.next_y_vals.push_back(previous_path_y[i]);
    }

    const int c_path_size = 50;
    const double c_mph_to_mps = 2.24;

    // cout << "forward_vehicle:" << (forward_vehicle.get() != nullptr) << endl;

    Perspective perspective(ref_x, ref_y, ref_yaw);

    if (forward_vehicle)
    {
        // cout << "prev_size:" << prev_size << endl;
        if (prev_size < c_path_size)
        {
            // cout << "Forward vehicle detected. id:" << forward_vehicle->id << endl;

            auto prev_frenet = env.getFrenet(ptsx[0], ptsy[0], ref_yaw);
            auto frenet = env.getFrenet(ptsx[1], ptsy[1], ref_yaw);

            // cout << "pts[0]:(" << ptsx[0] << ',' << ptsy[0] << "),pts[1]:(" << ptsx[1] << ',' << ptsy[1] << "),frenet_s:" << frenet[0] << ",prev_frenet_s:" << prev_frenet[0] << endl;

            double speed_estimate = car_speed / c_mph_to_mps;
            // cout << "speed_estimate:" << speed_estimate << endl;

            Vector3d start_s;
            start_s << frenet[0],
                       min(speed_estimate, abs((frenet[0] - prev_frenet[0]) / 0.02)),
                       0;

            Vector3d start_d;
            start_d << frenet[1],
                       0, // (frenet[1] - prev_frenet[1]) / 0.02,
                       0;

            // cout << "start_s:" << start_s << ", start_d: " << start_d << endl;

            // Pass forward vehicle.
            VectorXd delta(6);
            delta << 0, 0, 0, 0, 0, 0;

            double T = 2.5;

            Trajectory best = PTG(start_s, start_d, *forward_vehicle, delta, T, env.get_vehicles());

            // cout << "best.t:" << best.t << endl;

            auto points = smooth_trajectory(best, env, perspective, ptsx, ptsy);

            for (const auto& p : points)
            {
                path.next_x_vals.push_back(p.x);
                path.next_y_vals.push_back(p.y);
            }
        }
        else
        {
            // cout << "Follow prev path." << endl;
        }
    }
    else if (prev_size < c_path_size)
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

        perspective.transform_to_car(ptsx, ptsy);

        tk::spline s;
        s.set_points(ptsx, ptsy);

        double target_x = 30.0;
        double target_y = s(target_x);
        double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

        double x_add_on = 0;

        vector<double> new_ptsx;
        vector<double> new_ptsy;

        for (int i = 1; i <= c_path_size - previous_path_x.size(); i++)
        {
            double N = (target_dist / (0.02 * ref_vel / c_mph_to_mps));
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            new_ptsx.push_back(x_point);
            new_ptsy.push_back(y_point);
        }

        perspective.transform_to_global(new_ptsx, new_ptsy);

        for (int i = 0; i < new_ptsx.size(); i++)
        {
            path.next_x_vals.push_back(new_ptsx[i]);
            path.next_y_vals.push_back(new_ptsy[i]);
        }
    }

    return path;
}
