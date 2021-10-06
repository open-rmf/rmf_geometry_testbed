/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


#ifndef RMF_GEOMETRY_TESTBED__REEDSSHEPP_HPP
#define RMF_GEOMETRY_TESTBED__REEDSSHEPP_HPP


#include <Eigen/Dense>
#include <rmf_traffic/Motion.hpp>

enum CurveModes
{
  CURVE_LEFT = 0,
  STRAIGHT_LINE,
  CURVE_RIGHT,

  //Reeds-shepp curves only
  CURVE_LEFT_REVERSED,
  CURVE_RIGHT_REVERSED,
};

struct NonHolonomicMotion
{
  CurveModes modes[3] = { CURVE_RIGHT, STRAIGHT_LINE, CURVE_RIGHT };

  double turning_radius = 1.0;

  Eigen::Vector2d start_pos;
  double start_yaw;

  Eigen::Vector2d end_pos;
  double end_yaw;

  void draw();

  void to_rmf_trajectory(rmf_traffic::Trajectory& traj, double velocity);

private:
  // subroutines

  void line_to_trajectory(Eigen::Vector2d tangent_start, 
    Eigen::Vector2d tangent_end, rmf_traffic::Trajectory& traj, 
    std::chrono::time_point<std::chrono::steady_clock> start_time,
    std::chrono::time_point<std::chrono::steady_clock>& end_time_out,
    double velocity);
  

  void circular_subarc_to_trajectory(Eigen::Vector2d circle_center, 
    double start_arc_radians, double end_arc_radians,
    rmf_traffic::Trajectory& traj, 
    std::chrono::time_point<std::chrono::steady_clock> start_time,
    std::chrono::time_point<std::chrono::steady_clock>& end_time_out,
    double velocity);
  
  void circular_arc_to_trajectory(Eigen::Vector2d circle_center, 
    Eigen::Vector2d start_arc_pt, Eigen::Vector2d end_arc_pt,
    rmf_traffic::Trajectory& traj, 
    std::chrono::time_point<std::chrono::steady_clock> start_time,
    std::chrono::time_point<std::chrono::steady_clock>& end_time_out,
    double velocity, bool anticlockwise);

};

#endif // RMF_GEOMETRY_TESTBED__REEDSSHEPP_HPP
