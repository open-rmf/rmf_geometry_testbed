cmake_minimum_required(VERSION 3.5.0)

project(rmf_geometry_testbed)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find_package(rmf_fleet_adapter REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(SFML COMPONENTS graphics window system REQUIRED)
find_package(ImGui-SFML)


include(GNUInstallDirs)

add_library(
  rmf_geometry_utils SHARED
    src/rmf_geometry_testbed/IMDraw.cpp
)

target_link_libraries(
  rmf_geometry_utils
  PUBLIC
    rmf_traffic::rmf_traffic
    sfml-graphics
)

target_include_directories(
  rmf_geometry_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_executable(test_spline
    test/test_spline.cpp
  )

target_link_libraries(
  test_spline
  PUBLIC
    rmf_geometry_utils
    ImGui-SFML::ImGui-SFML
)

######## FCL test ##########

find_package(fcl 0.6 QUIET)
if(fcl_FOUND)
  set(FCL_LIBRARIES fcl)
  message(STATUS "Using FCL version: ${FCL_VERSION}")

  add_executable(test_reedshepp_cubic
    test/test_reedshepp_cubic.cpp
    test/ReedsShepp.cpp
  )

  target_link_libraries(
    test_reedshepp_cubic
    PUBLIC
      rmf_geometry_utils
      ImGui-SFML::ImGui-SFML
      fcl
  )

  add_executable(test_sidecar
    test/test_sidecar.cpp
    test/test_sidecar_utils.cpp
    test/spline_utils.cpp
  )

  target_link_libraries(
    test_sidecar
    PUBLIC
      rmf_geometry_utils
      ImGui-SFML::ImGui-SFML
      fcl
  )

endif()

## copy font files
install(
  DIRECTORY fonts/
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/fonts/
)


