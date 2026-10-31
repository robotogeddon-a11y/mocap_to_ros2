cmake_minimum_required(VERSION 3.16)
project(mocap_gz_replay)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

# Gazebo system plugin: kinematic (exact) replay of joint positions + root pose
add_library(MocapReplay SHARED src/MocapReplay.cc)
target_compile_features(MocapReplay PRIVATE cxx_std_17)
target_link_libraries(MocapReplay
  gz-sim::gz-sim
  gz-plugin::register
  gz-transport::gz-transport
  gz-msgs::gz-msgs)

install(TARGETS MocapReplay LIBRARY DESTINATION lib)
install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})

ament_package()
