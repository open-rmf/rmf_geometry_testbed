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

#include <rmf_geometry_testbed/IMDraw.hpp>

#include <math.h>
#include <iostream>

namespace rmf_geometry_utils {

static std::vector<sf::VertexArray> g_vertexarrays; //internal use only
static const int VERTEX_ARRAYS_LIMIT = 16 * 1024;

void IMDraw::draw_circle(const sf::Vector2f& center, double radius, const sf::Color& color, uint slices)
{
  if (g_vertexarrays.size() >= VERTEX_ARRAYS_LIMIT)
  {
    std::cout << "Vertex array limit exceeded" << std::endl;
    return;
  }

  sf::VertexArray arr(sf::Lines);

  double rotation_per_slice = (2.0 * M_PI) / (double)slices;
  for (uint i=0; i<slices; ++i)
  {
    sf::Vertex v;
    v.color = color;

    double rot = rotation_per_slice * (double)i;
    v.position = center + sf::Vector2f(radius * cos(rot), radius * sin(rot));
    arr.append(v);

    rot = rotation_per_slice * (double)(i + 1);
    v.position = center + sf::Vector2f(radius * cos(rot), radius * sin(rot));
    arr.append(v);
  }
  g_vertexarrays.push_back(arr);
}


void IMDraw::draw_axis(float size)
{
  if (g_vertexarrays.size() >= VERTEX_ARRAYS_LIMIT)
  {
    std::cout << "Vertex array limit exceeded" << std::endl;
    return;
  }

  draw_arrow(sf::Vector2f(0,0), sf::Vector2f(size, 0), sf::Color::Red);
  draw_arrow(sf::Vector2f(0,0), sf::Vector2f(0, size), sf::Color::Green);
}

void IMDraw::draw_line(const sf::Vector2f& start, const sf::Vector2f& end, const sf::Color& color)
{
  if (g_vertexarrays.size() >= VERTEX_ARRAYS_LIMIT)
  {
    std::cout << "Vertex array limit exceeded" << std::endl;
    return;
  }

  sf::VertexArray vtx_arr(sf::Lines);

  sf::Vertex v;
  v.color = color;

  v.position = start;
  vtx_arr.append(v);

  v.position = end;
  vtx_arr.append(v);

  g_vertexarrays.push_back(vtx_arr);
}

void IMDraw::draw_arrow(const sf::Vector2f& start, const sf::Vector2f& end, const sf::Color& color)
{
  if (g_vertexarrays.size() >= VERTEX_ARRAYS_LIMIT)
  {
    std::cout << "Vertex array limit exceeded" << std::endl;
    return;
  }

  sf::VertexArray vtx_arr(sf::Lines);

  sf::Vertex v;
  v.color = color;

  v.position = start;
  vtx_arr.append(v);

  v.position = end;
  vtx_arr.append(v);

  // arrowhead
  sf::Vector2f line_vec = end - start;
  float lengthsq = line_vec.x * line_vec.x + line_vec.y * line_vec.y;
  float length = sqrt(lengthsq);

  auto line_norm = line_vec / length;
  float rotation_rad = (3.0f * M_PI) / 4.0f;
  
  sf::Vector2f left_vec(
    line_norm.x * cos(rotation_rad) - line_norm.y * sin(rotation_rad), 
    line_norm.x * sin(rotation_rad) + line_norm.y * cos(rotation_rad));
  sf::Vector2f right_vec(
    line_norm.x * cos(-rotation_rad) - line_norm.y * sin(-rotation_rad), 
    line_norm.x * sin(-rotation_rad) + line_norm.y * cos(-rotation_rad));
  
  const float arrowhead_len = 0.25f;
  v.position = end;
  vtx_arr.append(v);
  v.position = end + left_vec * arrowhead_len;
  vtx_arr.append(v);

  v.position = end;
  vtx_arr.append(v);
  v.position = end + right_vec * arrowhead_len;
  vtx_arr.append(v);

  g_vertexarrays.push_back(vtx_arr);
}

void IMDraw::draw_aabb(const sf::Vector2f& box_min, const sf::Vector2f& box_max, const sf::Color& color)
{
  if (g_vertexarrays.size() >= VERTEX_ARRAYS_LIMIT)
  {
    std::cout << "Vertex array limit exceeded" << std::endl;
    return;
  }

  sf::VertexArray vtx_arr(sf::Lines);

  sf::Vertex v;
  v.color = color;

  v.position = box_min;
  vtx_arr.append(v);

  v.position.x = box_max.x;
  v.position.y = box_min.y;
  vtx_arr.append(v);

  v.position.x = box_max.x;
  v.position.y = box_min.y;
  vtx_arr.append(v);

  v.position.x = box_max.x;
  v.position.y = box_max.y;
  vtx_arr.append(v);

  v.position.x = box_max.x;
  v.position.y = box_max.y;
  vtx_arr.append(v);

  v.position.x = box_min.x;
  v.position.y = box_max.y;
  vtx_arr.append(v);

  v.position.x = box_min.x;
  v.position.y = box_max.y;
  vtx_arr.append(v);

  v.position = box_min;
  vtx_arr.append(v);

  g_vertexarrays.push_back(vtx_arr);
}

void IMDraw::flush_and_render(sf::RenderWindow& app_window, const sf::Transform& tx_flipped_2d)
{
  for (auto& single_array : g_vertexarrays)
    app_window.draw(single_array, tx_flipped_2d);
  g_vertexarrays.clear();
}

} // namespace rmf_geometry_utils
