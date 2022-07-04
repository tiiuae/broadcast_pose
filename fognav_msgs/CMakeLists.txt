cmake_minimum_required(VERSION 3.5)
project(fognav_msgs)

find_package(ament_cmake REQUIRED)

# find_package(fognav_common REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

# fognav_package()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to Release as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE
    STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC)
endif()

option(COVERAGE_ENABLED "Enable code coverage" FALSE)

if(COVERAGE_ENABLED)
  add_compile_options(--coverage)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Costmap.msg"
  "msg/CostmapMetaData.msg"
  "msg/CostmapFilterInfo.msg"
  "msg/SpeedLimit.msg"
  "msg/VoxelGrid.msg"
  "msg/BehaviorTreeStatusChange.msg"
  "msg/BehaviorTreeLog.msg"
  "msg/Particle.msg"
  "msg/ParticleCloud.msg"
  "msg/ControlInterfaceDiagnostics.msg"
  "msg/NavigationDiagnostics.msg"
  "msg/TrajectoryPose.msg"
  "srv/GetCostmap.srv"
  "srv/IsPathValid.srv"
  "srv/ResamplePath.srv"
  "srv/ClearCostmapExceptRegion.srv"
  "srv/ClearCostmapAroundRobot.srv"
  "srv/ClearEntireCostmap.srv"
  "srv/ManageLifecycleNodes.srv"
  "srv/LoadMap.srv"
  "srv/TransformPath.srv"
  "srv/TransformPose.srv"
  "srv/SaveMap.srv"
  "action/BackUp.action"
  "action/ComputePathToPose.action"
  "action/ComputePathThroughPoses.action"
  "action/SmoothPath.action"
  "action/FollowPath.action"
  "action/NavigateToPose.action"
  "action/NavigateThroughPoses.action"
  "action/Land.action"
  "action/Takeoff.action"
  "action/Wait.action"
  "action/Spin.action"
  "action/DummyRecovery.action"
  "action/FollowWaypoints.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
