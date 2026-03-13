cmake_minimum_required(VERSION 3.8)
project(chy_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)


add_executable(node01 src/node01.cpp)
target_include_directories(node01 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(node01 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  node01
  "rclcpp"
)


# 添加可执行文件
add_executable(stereo_split_node src/stereo_split_node.cpp)
# 添加 ROS 依赖（自动处理头文件路径和链接）
ament_target_dependencies(stereo_split_node
  rclcpp
  sensor_msgs
  cv_bridge
)
# 链接非 ROS 库（OpenCV, yaml-cpp）
target_link_libraries(stereo_split_node
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
# 安装可执行文件
install(TARGETS stereo_split_node
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS node01
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
