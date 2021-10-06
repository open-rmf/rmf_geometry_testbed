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

#include <SFML/Graphics.hpp>
#include <imgui.h>

#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include "ReedsShepp.hpp"

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

struct CurveMeta
{
  std::string name;
  NonHolonomicMotion motion;
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
      NonHolonomicMotion motion = curves[current_curve_idx].motion;
      motion.turning_radius = turning_radius;
      IMDraw::draw_axis();
      
      if (show_intermediates)
      {
        motion.draw();

        sf::Color cyan_faded(0, 127, 127);
        IMDraw::draw_circle(motion.start_pos, 0.1, cyan_faded);
        Eigen::Vector2d start_dir(cos(motion.start_yaw), sin(motion.start_yaw));
        IMDraw::draw_arrow(motion.start_pos, motion.start_pos + start_dir, cyan_faded);

        IMDraw::draw_circle(motion.end_pos, 0.1, cyan_faded);
        Eigen::Vector2d end_dir(cos(motion.end_yaw), sin(motion.end_yaw));
        IMDraw::draw_arrow(motion.end_pos, motion.end_pos + end_dir, cyan_faded);
      }

      
      rmf_traffic::Trajectory t1;
      motion.to_rmf_trajectory(t1, velocity);
      ImGui::Text("rmf::Trajectory size: %d", t1.size());

      if (show_rmf_traffic_traj)
      {
        IMDraw::draw_trajectory(t1);
        IMDraw::draw_sampled_trajectory(t1, interp);
      }
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
    meta.motion.modes[0] = CURVE_RIGHT;
    meta.motion.modes[1] = STRAIGHT_LINE;
    meta.motion.modes[2] = CURVE_RIGHT;
    
    meta.motion.start_pos = Eigen::Vector2d(-6, -4);
    meta.motion.start_yaw = 90.0 / 180.0 * M_PI;
    meta.motion.end_pos = Eigen::Vector2d(6, -4);
    meta.motion.end_yaw = -90.0 / 180.0 * M_PI;
    
    ret.push_back(meta);
  }

  {
    CurveMeta meta;
    meta.name = "LSL";
    meta.motion.modes[0] = CURVE_LEFT;
    meta.motion.modes[1] = STRAIGHT_LINE;
    meta.motion.modes[2] = CURVE_LEFT;
    
    meta.motion.start_pos = Eigen::Vector2d(6, -4);
    meta.motion.start_yaw = 90.0 / 180.0 * M_PI;
    meta.motion.end_pos = Eigen::Vector2d(-6, -4);
    meta.motion.end_yaw = -90.0 / 180.0 * M_PI;
    
    ret.push_back(meta);
  }

  {
    CurveMeta meta;
    meta.name = "RSL";
    meta.motion.modes[0] = CURVE_RIGHT;
    meta.motion.modes[1] = STRAIGHT_LINE;
    meta.motion.modes[2] = CURVE_LEFT;
    
    meta.motion.start_pos = Eigen::Vector2d(-6, -4);
    meta.motion.start_yaw = 90.0 / 180.0 * M_PI;
    meta.motion.end_pos = Eigen::Vector2d(6, -4);
    meta.motion.end_yaw = -90.0 / 180.0 * M_PI;
    
    ret.push_back(meta);
  }

  {
    CurveMeta meta;
    meta.name = "LSR";
    meta.motion.modes[0] = CURVE_LEFT;
    meta.motion.modes[1] = STRAIGHT_LINE;
    meta.motion.modes[2] = CURVE_RIGHT;
    
    meta.motion.start_pos = Eigen::Vector2d(-6, -4);
    meta.motion.start_yaw = -M_PI / 3.0;
    meta.motion.end_pos = Eigen::Vector2d(6, -3);
    meta.motion.end_yaw = -M_PI / 2.0;
    
    ret.push_back(meta);
  }

  {
    CurveMeta meta;
    meta.name = "RLR";
    meta.motion.modes[0] = CURVE_LEFT;
    meta.motion.modes[1] = CURVE_RIGHT;
    meta.motion.modes[2] = CURVE_LEFT;
    
    meta.motion.start_pos = Eigen::Vector2d(0, -4);
    meta.motion.start_yaw = 0.0;
    meta.motion.end_pos = Eigen::Vector2d(0, 0.85);
    meta.motion.end_yaw = M_PI;
    
    ret.push_back(meta);
  }

  return ret;
}