cmake_minimum_required(VERSION 3.8)
project(extra_rviz_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/extra_rviz_panel/rosbag2_panel.hpp
)

add_library(rosbag2_panel src/rosbag2_panel.cpp ${MOC_FILES})

target_include_directories(rosbag2_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(rosbag2_panel
  pluginlib
  rviz_common
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  sensor_msgs
  geometry_msgs
  tf2
  cv_bridge
)
target_link_libraries(rosbag2_panel ${OpenCV_LIBS})
install(TARGETS rosbag2_panel
        EXPORT export_rviz_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_rviz_panel)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()