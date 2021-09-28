# rmf_geometry_testbed: Testbed for testing collision detection/collision primitives approximation

1. Create a new colcon workspace
2. Clone this repo
3. Clone rmf repositories https://github.com/open-rmf/rmf_utils.git, https://github.com/open-rmf/rmf_traffic.git
4. Clone https://github.com/SFML/SFML
5. Clone https://github.com/eliasdaler/imgui-sfml.git
6. Clone the version of https://github.com/ocornut/imgui.git mentioned in the previous step (at the time of this writing, v1.80)
7. go into your imgui directory and do `touch COLCON_IGNORE`
8. `colcon build --cmake-args -DBUILD_SHARED_LIBS=ON -DIMGUI_DIR=<your colcon workspace>/src/imgui`

Then you can start running interactive tests in the `./build/rmf_geometry_testbed/` directory
