
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



#ifndef RMF_GEOMETRY_TESTBED__TEST_SIDECAR_UTILS_HPP
#define RMF_GEOMETRY_TESTBED__TEST_SIDECAR_UTILS_HPP

#include <Eigen/Dense>
#include <fcl/math/motion/spline_motion.h>
#include <float.h>

// Custom Bilateral Advancement algorithm that uses seperable shapes 

struct ModelSpaceShape
{
  ModelSpaceShape(const fcl::Transform3d& tx, double r)
    :_transform(tx), _radius(r)
  { }
  fcl::Transform3d _transform;
  double _radius;
};

// this uses spline motions
bool collide_seperable_circles(
  fcl::SplineMotion<double>& motion_a, 
  fcl::SplineMotion<double>& motion_b,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
  double& impact_time, uint& dist_checks, 
  uint safety_maximum_checks = 120, double tolerance = 0.001);

fcl::SplineMotion<double> to_fcl(const std::array<Eigen::Vector3d, 4>& knots);

// Presets
enum PRESET_TYPE
{
  PRESET_LINEAR = 0,
  PRESET_SPLINEMOTION
};

struct Preset
{
  std::string _description;
  PRESET_TYPE _type = PRESET_SPLINEMOTION;
  double tolerance = 0.01;
  std::vector<ModelSpaceShape> a_shapes;
  std::vector<ModelSpaceShape> b_shapes;

  Eigen::Vector3d a_start = Eigen::Vector3d(0,0,0);
  Eigen::Vector3d a_end = Eigen::Vector3d(0,0,0);
  Eigen::Vector3d b_start = Eigen::Vector3d(0,0,0);
  Eigen::Vector3d b_end = Eigen::Vector3d(0,0,0);
  
  Eigen::Vector3d b_vel = Eigen::Vector3d(0,0,0);
};

std::vector<Preset> setup_presets();

#endif // RMF_GEOMETRY_TESTBED__TEST_SIDECAR_UTILS_HPP
