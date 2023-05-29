cmake_minimum_required(VERSION 3.8)
project(ros-crystalfontz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

add_executable(crystalfontz_driver_node src/driver_main.cpp src/cf_packet.cpp src/serial.cpp src/show_packet.cpp)
target_include_directories(crystalfontz_driver_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(crystalfontz_driver_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
        crystalfontz_driver_node
        std_msgs
        rclcpp
)

add_executable(crystalfontz_logic_node src/logic_main.cpp)
target_include_directories(crystalfontz_logic_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(crystalfontz_logic_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
        crystalfontz_logic_node
        std_msgs
        rclcpp
        Boost
)

install(TARGETS crystalfontz_driver_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS crystalfontz_logic_node
        DESTINATION lib/${PROJECT_NAME})

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
