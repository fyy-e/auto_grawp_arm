cmake_minimum_required(VERSION 3.10)
project(arm_foc)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_executable(main
    csrc/main.cpp
    csrc/socketcan.cpp
    csrc/motor_controller.cpp
    csrc/RobotArm6Axis.cpp
    csrc/6dof_kinematic.cpp
)

set_property(TARGET main PROPERTY CXX_STANDARD 17)

target_include_directories(main PRIVATE csrc)


target_link_libraries(main pthread)