/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <SFML/Graphics.hpp>
#include <imgui.h>

#include <iostream>
#include <math.h>
#include <Eigen/Dense>

#include <rmf_traffic/Motion.hpp>
#include <rmf_geometry_testbed/IMDraw.hpp>
#include <fcl/math/motion/spline_motion.h>

#include "imgui-SFML.h"

using namespace rmf_geometry_utils;

void draw_fcl_spline(const fcl::SplineMotion<double>& motion, sf::Color color = sf::Color::White)
{
  const uint steps = 100;
  double interp_per_step = 1.0f / (double)steps;
  fcl::Transform3d tf, tf2;

  for (uint i=0; i<steps; ++i)
  {
    double interp = interp_per_step * (double)i;
    double interp_next = interp_per_step * (double)(i + 1);
    
    motion.integrate(interp);
    motion.getCurrentTransform(tf);

    auto translate1 = tf.translation();    
    motion.integrate(interp_next);

    motion.getCurrentTransform(tf2);
    auto translate2 = tf2.translation();
    
    IMDraw::draw_line(sf::Vector2f(translate1.x(), translate1.y()), sf::Vector2f(translate2.x(), translate2.y()), color);
  }
  motion.integrate(0.0); //reset
}

enum CurveModes
{
  CURVE_LEFT = 0,
  STRAIGHT_LINE,
  CURVE_RIGHT,

  //Reeds-shepp curves only
  CURVE_LEFT_REVERSED,
  CURVE_RIGHT_REVERSED,
};

struct DubinsCurve
{
  CurveModes modes[3] = { CURVE_RIGHT, STRAIGHT_LINE, CURVE_RIGHT };

  double turning_radius = 1.0;

  Eigen::Vector2d start_pos;
  double start_yaw;

  Eigen::Vector2d end_pos;
  double end_yaw;

  void draw()
  {
    Eigen::Vector2d start_circle_pos;
    Eigen::Vector2d end_circle_pos;
    Eigen::Vector2d start_dir(cos(start_yaw), sin(start_yaw));
    Eigen::Vector2d end_dir(cos(end_yaw), sin(end_yaw));

    if (modes[0] == CURVE_RIGHT || modes[0] == CURVE_LEFT)
    {
      Eigen::Vector2d start_perp_dir(-start_dir.y(), start_dir.x());
      start_perp_dir *= turning_radius;
      if (modes[0] == CURVE_RIGHT)
        start_perp_dir = -start_perp_dir;
      
      start_circle_pos = start_pos + start_perp_dir;
      IMDraw::draw_circle(start_circle_pos, turning_radius, sf::Color(64,64,64), 32);
    }
    if (modes[2] == CURVE_RIGHT || modes[2] == CURVE_LEFT)
    {
      Eigen::Vector2d end_perp_dir(-end_dir.y(), end_dir.x());
      end_perp_dir *= turning_radius;
      if (modes[2] == CURVE_RIGHT)
        end_perp_dir = -end_perp_dir;
      
      end_circle_pos = end_pos + end_perp_dir;
      IMDraw::draw_circle(end_circle_pos, turning_radius, sf::Color(64,64,64), 32);
    }

    Eigen::Vector2d tangent_start, tangent_end;
    if (modes[1] == STRAIGHT_LINE)
    {
      Eigen::Vector2d start_circle_to_end_circle = end_circle_pos - start_circle_pos;
      start_circle_to_end_circle.normalize();

      Eigen::Vector2d offset = start_circle_to_end_circle * turning_radius;
      Eigen::Vector2d perp = Eigen::Vector2d(-offset.y(), offset.x());
      
      if (modes[0] == CURVE_LEFT)
        tangent_start = start_circle_pos - perp;
      else
        tangent_start = start_circle_pos + perp;

      if (modes[2] == CURVE_LEFT)
        tangent_end = end_circle_pos - perp;
      else
        tangent_end = end_circle_pos + perp;
      
      IMDraw::draw_line(tangent_start, tangent_end, sf::Color(64,64,64));
    }
    else if (modes[1] == CURVE_LEFT || modes[1] == CURVE_RIGHT)
    {
      Eigen::Vector2d start_to_end_circle = end_circle_pos - start_circle_pos;
      double d = start_to_end_circle.norm();

      double theta = acos(d / (4.0 * turning_radius));
      if (modes[1] == CURVE_LEFT)
        theta = -theta;
      
      Eigen::Vector2d n = Eigen::Vector2d(cos(theta), sin(theta));
      n *= turning_radius * 2.0;

      Eigen::Vector2d mid_circle_pos = start_circle_pos + n;
      IMDraw::draw_circle(mid_circle_pos, turning_radius, sf::Color(64,64,64), 32);
    }
  }

  void line_to_trajectory(Eigen::Vector2d tangent_start, 
    Eigen::Vector2d tangent_end, rmf_traffic::Trajectory& traj, 
    std::chrono::time_point<std::chrono::steady_clock> start_time,
    std::chrono::time_point<std::chrono::steady_clock>& end_time_out,
    double velocity)
  {
    Eigen::Vector2d v = tangent_end - tangent_start;
    double distance = v.norm();
    double duration = distance / velocity;

    end_time_out = start_time + rmf_traffic::time::from_seconds(duration);

    Eigen::Vector3d p0 = Eigen::Vector3d(tangent_start.x(), tangent_start.y(), 0.0);
    Eigen::Vector3d p1 = Eigen::Vector3d(tangent_end.x(), tangent_end.y(), 0.0);
    Eigen::Vector3d zero = Eigen::Vector3d(0,0,0);
    
    auto& last_waypoint = traj.back();
    traj.insert(start_time, p0, last_waypoint.velocity());
    traj.insert(end_time_out, p1, zero);
  }

  void circular_subarc_to_trajectory(Eigen::Vector2d circle_center, 
    double start_arc_radians, double end_arc_radians,
    rmf_traffic::Trajectory& traj, 
    std::chrono::time_point<std::chrono::steady_clock> start_time,
    std::chrono::time_point<std::chrono::steady_clock>& end_time_out,
    double velocity)
  {
    // https://mechanicalexpressions.com/explore/geometric-modeling/circle-spline-approximation.pdf
    // form a control polygon with the start,midpoint,endpoint of the arc,
    // and convert it to hermite spline parameters
    Eigen::Vector3d start_arc_vector = Eigen::Vector3d(cos(start_arc_radians), sin(start_arc_radians), 0.0) * turning_radius;
    Eigen::Vector3d end_arc_vector = Eigen::Vector3d(cos(end_arc_radians), sin(end_arc_radians), 0.0) * turning_radius;

    Eigen::Vector3d p0 = Eigen::Vector3d(circle_center.x(), circle_center.y(), 0.0) + start_arc_vector;
    Eigen::Vector3d p3 = Eigen::Vector3d(circle_center.x(), circle_center.y(), 0.0) + end_arc_vector;

    double center_yaw = start_arc_radians + (end_arc_radians - start_arc_radians) * 0.5;
    Eigen::Vector2d midpoint_vector = Eigen::Vector2d(cos(center_yaw), sin(center_yaw)) * turning_radius;
    //IMDraw::draw_circle(start_circle_pos + midpoint_vector, 0.05, sf::Color(0, 127,0));
    Eigen::Vector2d midpoint = circle_center + midpoint_vector;

    Eigen::Vector3d p0_direction = Eigen::Vector3d(start_arc_vector.y(), -start_arc_vector.x(), 0.0);
    Eigen::Vector3d p3_direction = Eigen::Vector3d(-end_arc_vector.y(), end_arc_vector.x(), 0.0);
    p0_direction.normalize();
    p3_direction.normalize();

    double k = (midpoint.x() - 0.5 * p0.x() - 0.5 * p3.x()) / (0.375 * (p0_direction.x() + p3_direction.x()));
    //printf("k: %g\n", k);
    
    Eigen::Vector3d p1 = p0 + p0_direction * k;
    Eigen::Vector3d p2 = p3 + p3_direction * k;

    // estimate duration
    double arclength = std::abs(end_arc_radians - start_arc_radians) * turning_radius;
    double arc_duration = arclength / velocity;
    // printf("start_arc_radians %g end_arc_radians %g arc_duration: %g\n", start_arc_radians, end_arc_radians, arc_duration);
    end_time_out = start_time + rmf_traffic::time::from_seconds(arc_duration);
    double duration_per_slice = arc_duration / 3.0;

    auto convert_bspline_to_hermite = [&](Eigen::Vector3d p0, Eigen::Vector3d p1,
        Eigen::Vector3d p2, Eigen::Vector3d p3, sf::Color c, double offset_timing)
    {
      Eigen::Matrix4d mtx_bspline;
      mtx_bspline << 
        -1.0,  3.0, -3.0,  1.0,
          3.0, -6.0,  3.0,  0.0,
        -3.0,  0.0,  3.0,  0.0,
          1.0,  4.0,  1.0,  0.0;
      mtx_bspline /= 6.0;
      
      Eigen::Matrix4d mtx_hermite;
      mtx_hermite << 
        2.0, -2.0, 1.0, 1.0,
        -3.0, 3.0, -2.0, -1.0,
        0.0, 0.0, 1.0, 0.0,
        1.0, 0.0, 0.0, 0.0;

      Eigen::Matrix4d mtx_hermite_inv = mtx_hermite.inverse();

      Eigen::Matrix<double, 4, 3> input;
      input << 
        p0[0], p0[1], p0[2],
        p1[0], p1[1], p1[2],
        p2[0], p2[1], p2[2],
        p3[0], p3[1], p3[2];

      auto result = mtx_hermite_inv * mtx_bspline * input;
      Eigen::Vector2d hermite_p0(result(0, 0), result(0, 1));
      Eigen::Vector2d hermite_p1(result(1, 0), result(1, 1));
      Eigen::Vector2d hermite_v0(result(2, 0), result(2, 1));
      Eigen::Vector2d hermite_v1(result(3, 0), result(3, 1));

      IMDraw::draw_circle(Eigen::Vector2d(hermite_p0.x(), hermite_p0.y()), 0.05, c);
      IMDraw::draw_circle(Eigen::Vector2d(hermite_p1.x(), hermite_p1.y()), 0.05, c);

      IMDraw::draw_arrow(hermite_p0, hermite_p0 + hermite_v0, c);
      IMDraw::draw_arrow(hermite_p1, hermite_p1 + hermite_v1, c);

      auto start_t = start_time + rmf_traffic::time::from_seconds(offset_timing);
      auto end_time = start_t + rmf_traffic::time::from_seconds(duration_per_slice);

      Eigen::Vector3d hermite_p0_3d(result(0, 0), result(0, 1), result(0, 2));
      Eigen::Vector3d hermite_p1_3d(result(1, 0), result(1, 1), result(1, 2));
      Eigen::Vector3d hermite_v0_3d(result(2, 0), result(2, 1), result(2, 2));
      Eigen::Vector3d hermite_v1_3d(result(3, 0), result(3, 1), result(3, 2));

      if (!traj.empty())
      {
        // WORKAROUND
        // sometimes the bspline cuts off earlier at its tips than it should.
        // could be the division by 6.0 that fcl uses in its basis matrix
        // so we make sure the points join up
        auto& last_waypoint = traj.back();
        last_waypoint.position(hermite_p0_3d);
        last_waypoint.velocity(hermite_v0_3d / duration_per_slice);
      }

      traj.insert(start_t, hermite_p0_3d, hermite_v0_3d / duration_per_slice);
      traj.insert(end_time,   hermite_p1_3d, hermite_v1_3d / duration_per_slice);
    };

    sf::Color color(0, 127, 0);
    convert_bspline_to_hermite(p0, p0, p1, p2, color, 0.0);
    convert_bspline_to_hermite(p0, p1, p2, p3, color, duration_per_slice);
    convert_bspline_to_hermite(p1, p2, p3, p3, color, duration_per_slice * 2.0);
  }

  void circular_arc_to_trajectory(Eigen::Vector2d circle_center, 
    Eigen::Vector2d start_arc_pt, Eigen::Vector2d end_arc_pt,
    rmf_traffic::Trajectory& traj, 
    std::chrono::time_point<std::chrono::steady_clock> start_time,
    std::chrono::time_point<std::chrono::steady_clock>& end_time_out,
    double velocity, bool anticlockwise)
  {
    Eigen::Vector2d start_arc_vector = start_arc_pt - circle_center;
    start_arc_vector.normalize();
    Eigen::Vector2d end_arc_vector = end_arc_pt - circle_center;
    end_arc_vector.normalize();

    double start_arc_radians = atan2(start_arc_vector.y(), start_arc_vector.x());
    double end_arc_radians = atan2(end_arc_vector.y(), end_arc_vector.x());

    // break into subarcs to better approximate the circular arc
    double arc_difference = end_arc_radians - start_arc_radians;
    if (!anticlockwise)
      arc_difference = -(2.0 * M_PI - arc_difference);

    double min_interval_rot = M_PI_2;
    int intervals = 1;
    if (arc_difference > min_interval_rot)
    {
      double interval_d = arc_difference / min_interval_rot;
      intervals = (int)std::ceil(interval_d);
    }

    double arc_per_interval = arc_difference / (double)intervals;
    for (int i=0; i<intervals; ++i)
    {
      double start_arc = start_arc_radians + arc_per_interval * (double)i;
      double end_arc = start_arc_radians + arc_per_interval * (double)(i + 1);

      if (i != 0)
        start_time = end_time_out;
      circular_subarc_to_trajectory(circle_center, start_arc, end_arc,
        traj, start_time, end_time_out, velocity);
    }
  }

  void to_rmf_trajectory(rmf_traffic::Trajectory& traj, double velocity)
  {
    // also draws some additional drawing to help visualize the intermediate geometry
    Eigen::Vector2d start_circle_pos;
    Eigen::Vector2d end_circle_pos;
    Eigen::Vector2d start_dir(cos(start_yaw), sin(start_yaw));
    Eigen::Vector2d end_dir(cos(end_yaw), sin(end_yaw));
    
    if (modes[0] == CURVE_RIGHT || modes[0] == CURVE_LEFT)
    {
      Eigen::Vector2d start_perp_dir(-start_dir.y(), start_dir.x());
      start_perp_dir *= turning_radius;
      if (modes[0] == CURVE_RIGHT)
        start_perp_dir = -start_perp_dir;
      
      start_circle_pos = start_pos + start_perp_dir;
    }
    if (modes[2] == CURVE_RIGHT || modes[2] == CURVE_LEFT)
    {
      Eigen::Vector2d end_perp_dir(-end_dir.y(), end_dir.x());
      end_perp_dir *= turning_radius;
      if (modes[2] == CURVE_RIGHT)
        end_perp_dir = -end_perp_dir;
      
      end_circle_pos = end_pos + end_perp_dir;
    }

    //figure out tangent lines
    Eigen::Vector2d intermediate_start, intermediate_end;
    Eigen::Vector2d mid_circle_pos;
    if (modes[1] == STRAIGHT_LINE)
    {
      Eigen::Vector2d start_circle_to_end_circle = end_circle_pos - start_circle_pos;
      start_circle_to_end_circle.normalize();

      Eigen::Vector2d offset = start_circle_to_end_circle * turning_radius;
      Eigen::Vector2d perp = Eigen::Vector2d(-offset.y(), offset.x());
      
      if (modes[0] == CURVE_LEFT)
        intermediate_start = start_circle_pos - perp;
      else
        intermediate_start = start_circle_pos + perp;

      if (modes[2] == CURVE_LEFT)
        intermediate_end = end_circle_pos - perp;
      else
        intermediate_end = end_circle_pos + perp;
    }
    else
    {
      // triple curves
      Eigen::Vector2d start_to_end_circle = end_circle_pos - start_circle_pos;
      double d = start_to_end_circle.norm();

      double theta = acos(d / (4.0 * turning_radius));
      if (modes[1] == CURVE_LEFT)
        theta = -theta;
      
      Eigen::Vector2d n = Eigen::Vector2d(cos(theta), sin(theta));
      n *= turning_radius;
      mid_circle_pos = start_circle_pos + n * 2.0;

      intermediate_start = start_circle_pos + n;

      Eigen::Vector2d end_to_mid_circle = mid_circle_pos - end_circle_pos;
      end_to_mid_circle.normalize();
      intermediate_end = end_circle_pos + end_to_mid_circle * turning_radius;
    }

    auto now = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> next_timing;

    // do conversions
    if (modes[1] == STRAIGHT_LINE)
    {
      circular_arc_to_trajectory(start_circle_pos, start_pos, intermediate_start, traj, 
        now, next_timing, velocity, true);

      now = next_timing;
      line_to_trajectory(intermediate_start, intermediate_end, traj, now, next_timing, velocity);

      now = next_timing;
      circular_arc_to_trajectory(end_circle_pos, intermediate_end, end_pos, traj,
        now, next_timing, velocity, true);
    }
    else
    {
      circular_arc_to_trajectory(start_circle_pos, start_pos, intermediate_start, traj, 
        now, next_timing, velocity, true);
      
      now = next_timing;
      circular_arc_to_trajectory(mid_circle_pos, intermediate_start, intermediate_end, traj, 
        now, next_timing, velocity, false);

      now = next_timing;
      circular_arc_to_trajectory(end_circle_pos, intermediate_end, end_pos, traj,
        now, next_timing, velocity, true);
    }
  }
};

struct CurveMeta
{
  std::string name;
  DubinsCurve curve;
};

std::vector<CurveMeta> setup_presets();

int main()
{
  // square window to avoid stretching
  sf::RenderWindow app_window(
        sf::VideoMode(1024, 1024),
        "Test_ReedsShepp_Approximation",
        sf::Style::Default);
  app_window.setFramerateLimit(60);
  app_window.resetGLStates();

  //view centered at origin, with width/height
  sf::View view(sf::Vector2f(0.f, 0.f), sf::Vector2f(20.f, 20.f));
  app_window.setView(view);

  sf::Transformable vqs;
  vqs.setScale(1.f, -1.f);
  auto tx_flipped_2d = vqs.getTransform();

  ImGui::SFML::Init(app_window);

  // initial setup of a RSR dubins curves
  double velocity = 1.0;
  //double acceleration = 1.0;

  std::vector<CurveMeta> curves = setup_presets();
  // end initial setup

  sf::Clock deltaClock;
  while (app_window.isOpen())
  {
    sf::Event event;
    while (app_window.pollEvent(event))
    {
      ImGui::SFML::ProcessEvent(event);

      if (event.type == sf::Event::Closed)
        return 0;

      if (event.type == sf::Event::Resized)
      {
        sf::FloatRect visibleArea(0, 0, event.size.width, event.size.height);
        app_window.setView(sf::View(visibleArea));
      }
    }

    ImGui::SFML::Update(app_window, deltaClock.restart());
    
    ImGui::SetWindowSize(ImVec2(600, 200));
    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    
    static int current_curve_idx = 1;
    static bool show_intermediates = true;
    static bool show_rmf_traffic_traj = true;
    std::string preview_name = curves[current_curve_idx].name;
    
    if (ImGui::BeginCombo("Curve", preview_name.c_str()))
    {
      for (std::size_t idx = 0; idx < curves.size(); ++idx)
      {
        bool is_selected = current_curve_idx == idx;
        std::string name_val = curves[idx].name;
        if (ImGui::Selectable(name_val.c_str(), is_selected))
        {
          current_curve_idx = idx;
        }
      }
      ImGui::EndCombo();
    }
    ImGui::Separator();

    ImGui::Checkbox("Show Intermediates", &show_intermediates);
    ImGui::Checkbox("Show rmf_traffic::Trajectory", &show_rmf_traffic_traj);

    static float interp = 0.0f;
    ImGui::DragFloat("Interp", &interp, 0.01f);

    static float turning_radius = 1.0;
    ImGui::DragFloat("Turning radius", &turning_radius, 0.1f, 0.0f, 20.0f);

    ImGui::Separator();


    // draw
    {
      DubinsCurve c = curves[current_curve_idx].curve;
      c.turning_radius = turning_radius;
      IMDraw::draw_axis();
      
      if (show_intermediates)
      {
        c.draw();

        sf::Color cyan_faded(0, 127, 127);
        IMDraw::draw_circle(c.start_pos, 0.1, cyan_faded);
        Eigen::Vector2d start_dir(cos(c.start_yaw), sin(c.start_yaw));
        IMDraw::draw_arrow(c.start_pos, c.start_pos + start_dir, cyan_faded);

        IMDraw::draw_circle(c.end_pos, 0.1, cyan_faded);
        Eigen::Vector2d end_dir(cos(c.end_yaw), sin(c.end_yaw));
        IMDraw::draw_arrow(c.end_pos, c.end_pos + end_dir, cyan_faded);
      }

      
      rmf_traffic::Trajectory t1;
      c.to_rmf_trajectory(t1, velocity);
      ImGui::Text("rmf::Trajectory size: %d", t1.size());

      if (show_rmf_traffic_traj)
      {
        IMDraw::draw_trajectory(t1);
        IMDraw::draw_sampled_trajectory(t1, interp);
      }
      // draw_reeds_shepp_path();


      // approximate to cubic
      // draw_cubic_spline
    }

    ImGui::End();
    ImGui::EndFrame();

    app_window.clear();
    app_window.setView(view);

    IMDraw::flush_and_render(app_window, tx_flipped_2d);

    ImGui::SFML::Render(app_window);
    app_window.display();
  }

  return 0;
}

std::vector<CurveMeta> setup_presets()
{
  std::vector<CurveMeta> ret;

  {
    CurveMeta meta;
    meta.name = "RSR";
    meta.curve.modes[0] = CURVE_RIGHT;
    meta.curve.modes[1] = STRAIGHT_LINE;
    meta.curve.modes[2] = CURVE_RIGHT;
    
    meta.curve.start_pos = Eigen::Vector2d(-6, -4);
    meta.curve.start_yaw = 90.0 / 180.0 * M_PI;
    meta.curve.end_pos = Eigen::Vector2d(6, -4);
    meta.curve.end_yaw = -90.0 / 180.0 * M_PI;
    
    ret.push_back(meta);
  }

  {
    CurveMeta meta;
    meta.name = "LSL";
    meta.curve.modes[0] = CURVE_LEFT;
    meta.curve.modes[1] = STRAIGHT_LINE;
    meta.curve.modes[2] = CURVE_LEFT;
    
    meta.curve.start_pos = Eigen::Vector2d(6, -4);
    meta.curve.start_yaw = 90.0 / 180.0 * M_PI;
    meta.curve.end_pos = Eigen::Vector2d(-6, -4);
    meta.curve.end_yaw = -90.0 / 180.0 * M_PI;
    
    ret.push_back(meta);
  }

  {
    CurveMeta meta;
    meta.name = "RSL";
    meta.curve.modes[0] = CURVE_RIGHT;
    meta.curve.modes[1] = STRAIGHT_LINE;
    meta.curve.modes[2] = CURVE_LEFT;
    
    meta.curve.start_pos = Eigen::Vector2d(-6, -4);
    meta.curve.start_yaw = 90.0 / 180.0 * M_PI;
    meta.curve.end_pos = Eigen::Vector2d(6, -4);
    meta.curve.end_yaw = -90.0 / 180.0 * M_PI;
    
    ret.push_back(meta);
  }

  {
    CurveMeta meta;
    meta.name = "LSR";
    meta.curve.modes[0] = CURVE_LEFT;
    meta.curve.modes[1] = STRAIGHT_LINE;
    meta.curve.modes[2] = CURVE_RIGHT;
    
    meta.curve.start_pos = Eigen::Vector2d(-6, -4);
    meta.curve.start_yaw = -M_PI / 3.0;
    meta.curve.end_pos = Eigen::Vector2d(6, -3);
    meta.curve.end_yaw = -M_PI / 2.0;
    
    ret.push_back(meta);
  }

  {
    CurveMeta meta;
    meta.name = "RLR";
    meta.curve.modes[0] = CURVE_LEFT;
    meta.curve.modes[1] = CURVE_RIGHT;
    meta.curve.modes[2] = CURVE_LEFT;
    
    meta.curve.start_pos = Eigen::Vector2d(0, -4);
    meta.curve.start_yaw = 0.0;
    meta.curve.end_pos = Eigen::Vector2d(0, 0.85);
    meta.curve.end_yaw = M_PI;
    
    ret.push_back(meta);
  }

  return ret;
}