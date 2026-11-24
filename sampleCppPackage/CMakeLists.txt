cmake_minimum_required(VERSION 3.8)
project(sampleCppPackage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# =====================================================================
# ADD YOUR DEPENDENCIES HERE, and comment they where and why! 
# =====================================================================

# default required node dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# find_package(<dependency> REQUIRED) # the format

# =====================================================================
# ADD YOUR EXECUTABLES HERE (i.e. nodes), and comment they where and why! 
# =====================================================================
# hello world example
add_executable(helloWorldExample src/helloWorldExample.cpp)
target_include_directories(helloWorldExample PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(helloWorldExample PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# publisher node example
add_executable(talker src/sample_Publisher.cpp) # see how we mention the path to the node
ament_target_dependencies(talker rclcpp std_msgs) # <-- ADD DEPENDENCIES HERE after the named executable, so that the compiler can find them

# subscriber node example
add_executable(listener src/sample_subscriber.cpp)
## note that the 1st parameter is the exe's name, matches above add_executable
ament_target_dependencies(listener rclcpp std_msgs) 


# parameter node example
add_executable(minimal_param_node src/cpp_parameters_example.cpp)
ament_target_dependencies(minimal_param_node rclcpp)

# =====================================================================
# ADD YOUR EXECUTABLES HERE again, so that 'ros2 run' can find it
# =====================================================================
# up to you to have them all in 1 install(), or have a seperate install() per executable
install(TARGETS 
  helloWorldExample
  talker
  listener 
  minimal_param_node
  DESTINATION lib/${PROJECT_NAME} # compiled exe will be saved in a lib folder
)

# we don't really need this, but I'll leave it here for now
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
