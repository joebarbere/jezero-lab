cmake_minimum_required(VERSION 3.8)
project(osr_gz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch models urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

# meshes/ is a symlink into the osr-rover-code submodule. install(DIRECTORY)
# copies a symlinked directory as a (dangling) link, so resolve it first.
get_filename_component(OSR_MESHES "${CMAKE_CURRENT_SOURCE_DIR}/meshes" REALPATH)
install(DIRECTORY ${OSR_MESHES}/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(PROGRAMS scripts/gazebo_command_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
