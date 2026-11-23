cmake_minimum_required(VERSION 3.8)
project(autonav_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)
find_package(Threads REQUIRED)

#----------------
# Motor controller
#----------------
add_executable(motor_controller src/motor_controller.cpp)

target_include_directories(motor_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(motor_controller PUBLIC cxx_std_17)

ament_target_dependencies(motor_controller
  rclcpp
  std_msgs
)

#----------------
# Panorama capture
#----------------
add_executable(panorama_capture src/panorama_capture.cpp)

target_include_directories(panorama_capture PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(panorama_capture PUBLIC cxx_std_17)

ament_target_dependencies(panorama_capture
  rclcpp
  std_msgs
)

target_link_libraries(panorama_capture
  ${OpenCV_LIBRARIES}
  CURL::libcurl
  Threads::Threads
)

#----------------
# Panorama stitcher (standalone C++ executable)
#----------------
add_executable(panorama_opencv_stitcher
  src/panorama_opencv_stitcher.cpp
)

target_include_directories(panorama_opencv_stitcher PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(panorama_opencv_stitcher PUBLIC cxx_std_17)

target_link_libraries(panorama_opencv_stitcher PRIVATE
  ${OpenCV_LIBRARIES}
)

# Install executables
install(TARGETS
  motor_controller
  panorama_capture
  panorama_opencv_stitcher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and configuration
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
