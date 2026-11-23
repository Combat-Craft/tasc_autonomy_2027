cmake_minimum_required(VERSION 3.8)
project(autonav_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hello_word_example src/hello_word_example.cpp)
target_include_directories(
  hello_word_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(hello_word_example PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(gps_imu_broadcaster src/gps_imu_broadcaster.cpp)
target_include_directories(
  gps_imu_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(gps_imu_broadcaster PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(gps_imu_broadcaster rclcpp sensor_msgs std_msgs)

install(TARGETS
  hello_word_example
  gps_imu_broadcaster
  # add nodes names here
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch # add other folders as needed
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
