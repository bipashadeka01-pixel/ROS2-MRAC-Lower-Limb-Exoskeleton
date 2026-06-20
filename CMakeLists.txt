cmake_minimum_required(VERSION 3.8)
project(adaptive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Include paths ─────────────────────────────────────────────────────────────
include_directories(include)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(mrac_controller
  src/mrac_controller.cpp
)

ament_target_dependencies(mrac_controller
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(mrac_controller
  Eigen3::Eigen
)

# ── Install binary ────────────────────────────────────────────────────────────
install(TARGETS mrac_controller
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install headers ───────────────────────────────────────────────────────────
# FIX: was missing — caused "file not found" on clean builds
install(DIRECTORY include/
  DESTINATION include/
)
install(PROGRAMS
  scripts/mrac_error_analysis_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)
# ── Install share resources ───────────────────────────────────────────────────
# FIX: added rviz/ directory; OPTIONAL prevents errors if dirs don't exist yet
install(DIRECTORY
  launch
  urdf
  meshes
  rviz
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()