#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <iostream>

using namespace std;

enum State
{
  KEEP_LANE,
  CHANGE_LANE
};

enum Lane
{
  LEFT = 2,
  MID = 6,
  RIGHT = 10
};

class PathPlanner
{
public:
  /**
    * ctor
    */
  PathPlanner(vector<double> &map_waypoints_x,
              vector<double> &map_waypoints_y,
              vector<double> &map_waypoints_s,
              vector<double> &map_waypoints_dx,
              vector<double> &map_waypoints_dy);
  /**
    * dtor
    */
  virtual ~PathPlanner();
  /**
   *
   */
  void plan(vector<double> &next_x_vals,
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
            vector<vector<double>> &sensor_fusion_);

private:
  vector<double> map_x;
  vector<double> map_y;
  vector<double> map_s;
  vector<double> map_dx;
  vector<double> map_dy;
  // The max s value before wrapping around the track back to 0
  const double max_s = 6945.554;
  State state;
  Lane current_lane;
  Lane target_lane;

  double ref_v;
  // A counter to track number of consecutive frames in KEEP_LANE state
  // to prevent very frequent lane changing
  int keep_lane_cnt;

  // Make these variables as instance variables to avoid passing around.
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;
  vector<vector<double>> sensor_fusion;

  void stateTransition();
  void generateTrajectory(vector<double> &next_x_vals, vector<double> &next_y_vals);
  bool shouldChangeToLane(Lane purposed_target_lane);
  bool hasSlowCarAhead();
  bool isLaneClear(Lane purposed_target_lane);
  bool isTargetLaneReached();
};

#endif /* PATH_PLANNER_H */
