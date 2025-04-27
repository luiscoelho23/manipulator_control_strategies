cmake_minimum_required(VERSION 3.8)
project(manipulator_control_strategies)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(manipulator REQUIRED)
find_package(mplibrary REQUIRED)
find_package(manipulator_skill_acquisition REQUIRED)

ament_package()

# Install Python scripts
install(PROGRAMS
  manipulator_control_strategies/load_dmps.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

if(BUILD_TESTING)
# ... existing code ...
endif()
