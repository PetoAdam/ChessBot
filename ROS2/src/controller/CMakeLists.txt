cmake_minimum_required(VERSION 3.16)
project(chessbot LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wno-deprecated-declarations)
endif()

include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})


find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()



add_library(
  chessbot
  SHARED
  hardware/lbr3r760iisy_hardware.cpp
  hardware/RobotController.cpp
  hardware/simulator_control.pb.cc
  hardware/ZMQClient.cpp
  controller/gripper_controller.cpp
)

## INCLUDE DIRECTORIES
target_include_directories(chessbot PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_include_directories(chessbot PUBLIC ${ZeroMQ_INCLUDE_DIR})


## LINK LIBRARIES
target_link_libraries(chessbot PUBLIC ${ZeroMQ_LIBRARY})
target_link_libraries(chessbot PRIVATE protobuf::libprotobuf)

target_compile_features(chessbot PUBLIC cxx_std_17)
target_include_directories(chessbot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/chessbot>
)
ament_target_dependencies(
  chessbot PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "chessbot_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface chessbot.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface chessbot.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/chessbot
)
install(
  DIRECTORY description/launch description/urdf
  DESTINATION share/chessbot
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/chessbot
)

install(TARGETS chessbot
  EXPORT export_chessbot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_chessbot HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
