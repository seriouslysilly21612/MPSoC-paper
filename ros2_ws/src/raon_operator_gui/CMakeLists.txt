cmake_minimum_required(VERSION 3.8)
project(raon_operator_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)   # Q_OBJECT headers get moc'd automatically

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_interfaces REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)  # loaded by name at runtime
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

# Widgets + node shared by the app and the smoke test.
add_library(console_core STATIC
  src/ros_link.cpp
  src/video_pane.cpp
  src/rviz_pane.cpp
  src/joint_pane.cpp
  src/gains_pane.cpp
  src/main_window.cpp
  # headers listed so AUTOMOC generates their moc units
  src/ros_link.hpp
  src/video_pane.hpp
  src/rviz_pane.hpp
  src/joint_pane.hpp
  src/gains_pane.hpp
  src/main_window.hpp
)
target_link_libraries(console_core Qt5::Widgets ${OpenCV_LIBS})
ament_target_dependencies(console_core
  rclcpp sensor_msgs std_msgs my_interfaces rviz_common rviz_rendering
  nav_msgs tf2 tf2_ros visualization_msgs)

add_executable(operator_console src/main.cpp)
target_link_libraries(operator_console console_core)

# assert-based checks; run manually (see README §7) — not registered with
# ctest to keep the build dependency-free
add_executable(test_logic test/test_logic.cpp)
add_executable(test_gui_smoke test/test_gui_smoke.cpp)
target_link_libraries(test_gui_smoke console_core)

install(TARGETS operator_console DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch urdf meshes DESTINATION share/${PROJECT_NAME})

ament_package()
