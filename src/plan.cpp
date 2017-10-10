#include "plan.h"
#include <iostream>

#include "perspective.h"
#include "ptg.h"
#include "spline.h"
#include "stopwatch.h"
#include "util.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::Vector3d;

static const double c_mph_to_mps = 2.24;
static const int c_path_size = 50;
static const double PATH_SEGMENT_SECONDS = 0.02;

Plan::Plan(const Map& map, const Telemetry& tel, double ref_vel)
  : map(map),
    tel(tel),
    prev_size(tel.previous_path_x.size()),
    env(map.waypoints_s,
        map.waypoints_x,
        map.waypoints_y,
        tel.car_s,
        tel.car_d,
        tel.sensor_fusion,
        prev_size),
    car_lane(tel.car_d / 4),
    ref_vel(ref_vel)
{
}

Path Plan::make_plan()
{
    forward_vehicle = env.lane_is_occupied(car_lane, 30, 0);

    for (int i = 0; i < tel.previous_path_x.size(); i++)
    {
        path.next_x_vals.push_back(tel.previous_path_x[i]);
        path.next_y_vals.push_back(tel.previous_path_y[i]);
    }

    generate_waypoints();

    return path;
}

void Plan::generate_waypoints()
{
    Waypoints waypoints;
    PrevTail prev_tail = fill_prev_tail(waypoints);

    if (prev_size < c_path_size)
    {
        bool ptg_failed = false;

        if (forward_vehicle)
        {
            Trajectory trajectory = pass_or_follow(prev_tail, waypoints);

            get_trajectory_points(trajectory, waypoints.ptsx, waypoints.ptsy);

            bool success = make_smooth_path(prev_tail, &trajectory, waypoints, path);

            if (success)
            {
                // Convert meters per second to miles per hour.
                double dist = distance(path.next_x_vals[path.next_x_vals.size() - 1],
                                       path.next_y_vals[path.next_y_vals.size() - 1],
                                       path.next_x_vals[path.next_x_vals.size() - 2],
                                       path.next_y_vals[path.next_y_vals.size() - 2]);

                // Update ref_vel for smooth transition to "keep lane with spline".
                ref_vel = dist / PATH_SEGMENT_SECONDS * c_mph_to_mps;
            }
            else
            {
                ptg_failed = true;
            }
        }

        if (!forward_vehicle || ptg_failed)
        {
            keep_lane_with_spline(waypoints);

            make_smooth_path(prev_tail, nullptr, waypoints, path);
        }
    }
}

PrevTail Plan::fill_prev_tail(Waypoints& waypoints)
{
    auto& ptsx = waypoints.ptsx;
    auto& ptsy = waypoints.ptsy;

    double ref_x = tel.car_x;
    double ref_y = tel.car_y;
    double ref_yaw = deg2rad(tel.car_yaw);

    if (prev_size < 2)
    {
        double prev_car_x = ref_x - cos(ref_yaw);
        double prev_car_y = ref_y - sin(ref_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(ref_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(ref_y);
    }
    else
    {
        ref_x = tel.previous_path_x[prev_size - 1];
        ref_y = tel.previous_path_y[prev_size - 1];

        double ref_x_prev = tel.previous_path_x[prev_size - 2];
        double ref_y_prev = tel.previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    return PrevTail{ref_x, ref_y, ref_yaw};
}

Trajectory Plan::pass_or_follow(const PrevTail& prev_tail, const Waypoints& waypoints)
{
    const auto& ptsx = waypoints.ptsx;
    const auto& ptsy = waypoints.ptsy;

    auto prev_frenet = env.getFrenet(ptsx[0], ptsy[0], prev_tail.ref_yaw);
    auto frenet = env.getFrenet(ptsx[1], ptsy[1], prev_tail.ref_yaw);

    double speed_estimate = tel.car_speed / c_mph_to_mps;

    Vector3d start_s;
    start_s << frenet[0],
               min(speed_estimate, abs((frenet[0] - prev_frenet[0]) / PATH_SEGMENT_SECONDS)),
               0;

    // Cap d_vel to lessen oscillating path.
    double d_vel = min(0.25, max(-0.25, (frenet[1] - prev_frenet[1]) / PATH_SEGMENT_SECONDS));

    Vector3d start_d;
    start_d << frenet[1],
               d_vel,
               0;

    vector<PTG_Goal> ptg_goals;

    {
        // Follow behind.
        const double T = 2;
        VectorXd delta(6);
        delta << -3, 0, 0, 0, 0, 0;
        ptg_goals.emplace_back(PTG_Goal{*forward_vehicle, delta, T});
    }

    const double T = 4;
    auto forward_vehicles = env.get_forward_vehicles(45, 10);

    if (forward_vehicles[0] && forward_vehicles[1] && forward_vehicles[2])
    {
    }
    else if (allow_lane_change())
    {
        if (!forward_vehicles[0])
        {
            {
                // Pass left.
                VectorXd delta(6);
                delta << -3, 0, 0, -4, 0, 0;
                ptg_goals.emplace_back(PTG_Goal{*forward_vehicle, delta, T});
            }
        }
        else if (!forward_vehicles[1])
        {
            // Pass in middle lane.
            VectorXd delta(6);
            if (car_lane == 0)
            {
                delta << -3, 0, 0, 4, 0, 0;
            }
            else if (car_lane == 2)
            {
                delta << -3, 0, 0, -4, 0, 0;
            }

            ptg_goals.emplace_back(PTG_Goal{*forward_vehicle, delta, T});
        }
        else if (!forward_vehicles[2])
        {
            {
                // Pass right.
                VectorXd delta(6);
                delta << -3, 0, 0, 4, 0, 0;
                ptg_goals.emplace_back(PTG_Goal{*forward_vehicle, delta, T});
            }
        }
    }

    Trajectory best = PTG(start_s, start_d, ptg_goals, env.get_vehicles());
    return best;
}

void Plan::get_trajectory_points(const Trajectory& trajectory,
                                 vector<double>& ptsx,
                                 vector<double>& ptsy)
{
    auto s_poly = trajectory.s_poly();
    auto d_poly = trajectory.d_poly();

    static const double DOWNSAMPLE_SECONDS = 1.0;

    int downsample_size = trajectory.t / DOWNSAMPLE_SECONDS;

    for (int i = 1; i <= downsample_size; i++)
    {
        double t = i * DOWNSAMPLE_SECONDS;
        double s = s_poly.evaluate(t);
        double d = d_poly.evaluate(t);
        auto xy = env.getXY(s, d);

        ptsx.push_back(xy[0]);
        ptsy.push_back(xy[1]);
    }
}

bool Plan::allow_lane_change()
{
    const double car_s = tel.car_s;

    const double DISALLOW_ZONE = 100;
    return (car_s > DISALLOW_ZONE
            && car_s < (map.max_s - DISALLOW_ZONE));
}

void Plan::keep_lane_with_spline(Waypoints& waypoints)
{
    auto& ptsx = waypoints.ptsx;
    auto& ptsy = waypoints.ptsy;

    const double ideal_vel = 49.5;
    const double ref_vel_inc = 0.224;

    if (forward_vehicle || ref_vel > ideal_vel)
    {
        ref_vel -= ref_vel_inc;
    }
    else if (ref_vel < ideal_vel)
    {
        ref_vel += ref_vel_inc;
    }

    double car_s = tel.car_s;

    // Keep lane.
    if (prev_size > 0)
    {
        car_s = tel.end_path_s;
    }

    const int target_lane = car_lane;

    auto next_wp0 = env.getXY(car_s + 30, (2 + 4 * target_lane));
    auto next_wp1 = env.getXY(car_s + 60, (2 + 4 * target_lane));
    auto next_wp2 = env.getXY(car_s + 90, (2 + 4 * target_lane));

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);
}

bool Plan::make_smooth_path(const PrevTail& prevtail,
                            const Trajectory* p_trajectory,
                            const Waypoints& waypoints,
                            Path& path)
{
    Perspective perspective(prevtail.ref_x, prevtail.ref_y, prevtail.ref_yaw);

    double target_x = 0;
    int path_size = 0;

    if (p_trajectory)
    {
        target_x = p_trajectory->s_poly().evaluate(p_trajectory->t);
        path_size = p_trajectory->t / PATH_SEGMENT_SECONDS;
    }
    else
    {
        target_x = 30.0;
        path_size = c_path_size - tel.previous_path_x.size();
    }

    vector<double> ptsx(waypoints.ptsx);
    vector<double> ptsy(waypoints.ptsy);

    perspective.transform_to_car(ptsx, ptsy);

    bool has_violation = check_spline_has_violation(ptsx, ptsy, __func__);

    if (has_violation)
    {
        return false;
    }

    tk::spline s;
    s.set_points(ptsx, ptsy);

    double target_y = s(target_x);
    double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

    double x_add_on = 0;

    vector<double> new_ptsx;
    vector<double> new_ptsy;

    for (int i = 1; i <= path_size; i++)
    {
        double N = 0;
        if (p_trajectory)
        {
            const Polynomial s_poly = p_trajectory->s_poly();
            double segment_length = (s_poly.evaluate(i * PATH_SEGMENT_SECONDS)
                                     - s_poly.evaluate((i-1) * PATH_SEGMENT_SECONDS));
            N = target_dist / segment_length;
        }
        else
        {
            N = (target_dist / (PATH_SEGMENT_SECONDS * ref_vel / c_mph_to_mps));
        }

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

    return true;
}

double Plan::get_ref_vel() const
{
    return ref_vel;
}

void Plan::print(const vector<double>& ptsx, const vector<double>& ptsy, const char* func)
{
    cout << "print " << func << endl;
    for (int i = 0; i < ptsx.size(); i++)
    {
        cout << '(' << ptsx[i] << ',' << ptsy[i] << ')' << endl;
    }
}

bool Plan::has_spline_violation(const vector<double>& ptsx)
{
    for (int i = 0; i < ptsx.size() - 1; i++)
    {
        if (ptsx[i] > ptsx[i+1])
        {
            return true;
        }
    }

    return false;
}

bool Plan::check_spline_has_violation(const vector<double>& ptsx, const vector<double>& ptsy, const char* func)
{
    if (has_spline_violation(ptsx))
    {
        cout << "spline violation in " << func << endl;

        for (int i = 0; i < ptsx.size(); i++)
        {
            cout << '(' << ptsx[i] << ',' << ptsy[i] << ')';

            if ((i < ptsx.size() - 1) && ptsx[i] > ptsx[i + 1])
            {
                cout << " violation";
            }

            cout << endl;
        }
    }
}
