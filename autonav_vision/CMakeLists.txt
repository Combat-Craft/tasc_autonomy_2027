cmake_minimum_required(VERSION 3.8)
project(autonav_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-app-1.0)

add_executable(hello_word_example src/hello_word_example.cpp)
target_include_directories(
  hello_word_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(hello_word_example PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

foreach(camera_node IN ITEMS usb_camera webcam_test)
  add_executable(${camera_node} src/${camera_node}.cpp)
  target_compile_features(${camera_node} PRIVATE cxx_std_17)
  ament_target_dependencies(${camera_node} rclcpp sensor_msgs)
  target_link_libraries(${camera_node} PkgConfig::GST)
endforeach()

install(TARGETS 
	hello_word_example
	usb_camera
  webcam_test
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
