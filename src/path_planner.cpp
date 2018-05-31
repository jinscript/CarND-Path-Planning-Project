#include "path_planner.hpp"
#include "tools.hpp"
#include "spline.h"

PathPlanner::PathPlanner(vector<double> &map_waypoints_x,
                         vector<double> &map_waypoints_y,
                         vector<double> &map_waypoints_s,
                         vector<double> &map_waypoints_dx,
                         vector<double> &map_waypoints_dy)
{
  map_x = map_waypoints_x;
  map_y = map_waypoints_y;
  map_s = map_waypoints_s;
  map_dx = map_waypoints_dx;
  map_dy = map_waypoints_dy;
  state = KEEP_LANE;
  current_lane = MID;
  target_lane = MID;
  ref_v = 0.0;
  keep_lane_cnt = 0;
}

PathPlanner::~PathPlanner() {}

void PathPlanner::plan(vector<double> &next_x_vals,
                       vector<double> &next_y_vals,
                       double car_x_,
                       double car_y_,
                       double car_s_,
                       double car_d_,
                       double car_yaw_,
                       double car_speed_,
                       vector<double> &previous_path_x_,
                       vector<double> &previous_path_y_,
                       double end_path_s_,
                       double end_path_d_,
                       vector<vector<double>> &sensor_fusion_)
{
  car_x = car_x_;
  car_y = car_y_;
  car_s = car_s_;
  car_d = car_d_;
  car_yaw = car_yaw_;
  car_speed = car_speed_;
  previous_path_x = previous_path_x_;
  previous_path_y = previous_path_y_;
  end_path_s = end_path_s_;
  end_path_d = end_path_d_;
  sensor_fusion = sensor_fusion_;

  stateTransition();
  generateTrajectory(next_x_vals, next_y_vals);
}

bool PathPlanner::hasSlowCarAhead()
{
  int prev_size = previous_path_x.size();
  double car_s_ = prev_size > 0 ? end_path_s : car_s;

  for (int i = 0; i < sensor_fusion.size(); i++)
  {
    // car is on my lane
    double d = sensor_fusion[i][6];
    if (d < current_lane + 2 && d > current_lane - 2)
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += (double)prev_size * 0.02 * check_speed;

      if (check_car_s > car_s_ && (check_car_s - car_s_) < 30)
      {
        return true;
      }
    }
  }
  return false;
}

bool PathPlanner::isLaneClear(Lane purposed_target_lane)
{
  int prev_size = previous_path_x.size();
  double car_s_ = prev_size > 0 ? end_path_s : car_s;

  for (int i = 0; i < sensor_fusion.size(); i++)
  {
    // car is on purposed target lane
    double d = sensor_fusion[i][6];
    if (d < purposed_target_lane + 2 && d > purposed_target_lane - 2)
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += (double)prev_size * 0.02 * check_speed;

      if (check_car_s - car_s_ > -20 && check_car_s - car_s_ < 30)
      {
        return false;
      }
    }
  }
  return true;
}

bool PathPlanner::shouldChangeToLane(Lane purposed_target_lane)
{
  if (abs(purposed_target_lane - current_lane) != 4)
  {
    // can only change to the adjacent lane
    return false;
  }
  else if (!hasSlowCarAhead())
  {
    // do not change lane if there is not slow car ahead
    return false;
  }
  else if (keep_lane_cnt < 50)
  {
    // should at least stay for 1s in current lane
    return false;
  }
  return isLaneClear(purposed_target_lane);
}

bool PathPlanner::isTargetLaneReached()
{
  return abs(car_d - target_lane) < 0.2;
}

void PathPlanner::stateTransition()
{
  switch (state)
  {
  case KEEP_LANE:
    keep_lane_cnt++;
    if (shouldChangeToLane(LEFT))
    {
      target_lane = LEFT;
      state = CHANGE_LANE;
    }
    else if (shouldChangeToLane(MID))
    {
      target_lane = MID;
      state = CHANGE_LANE;
    }
    else if (shouldChangeToLane(RIGHT))
    {
      target_lane = RIGHT;
      state = CHANGE_LANE;
    }
    break;
  case CHANGE_LANE:
    keep_lane_cnt = 0;
    if (isTargetLaneReached())
    {
      state = KEEP_LANE;
      current_lane = target_lane;
    }
    break;
  default:
    break;
  }
}

void PathPlanner::generateTrajectory(vector<double> &next_x_vals, vector<double> &next_y_vals)
{
  int prev_size = previous_path_x.size();

  if (hasSlowCarAhead())
  {
    ref_v -= 0.224;
  }
  else if (ref_v < 49.5)
  {
    ref_v += 0.224;
  }

  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // if previous size is almost empty, use the car as starting reference.
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
    // Redefine reference state as previous path end point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // Use two points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly 50m speed points ahead of the starting reference
  vector<double> next_wp0 = getXY(car_s + 50, target_lane, map_s, map_x, map_y);
  vector<double> next_wp1 = getXY(car_s + 100, target_lane, map_s, map_x, map_y);
  vector<double> next_wp2 = getXY(car_s + 150, target_lane, map_s, map_x, map_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // convert to car coordinates
  for (int i = 0; i < ptsx.size(); i++)
  {
    // shift car angle to 0 degree
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
    ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
  }
  // create a spline
  tk::spline s;

  // set (x, y) points to the spline
  s.set_points(ptsx, ptsy);

  // Start with all of the previous points from last time
  for (int i = 0; i < previous_path_x.size(); i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;

  // Fill up the rest of our path planner after filling it with previous points. here we always outputs 50 points
  for (int i = 0; i <= 50 - previous_path_x.size(); i++)
  {
    double N = target_dist / (0.02 * ref_v / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // convert back to map coordinates
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}
