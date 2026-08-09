cmake_minimum_required(VERSION 3.10)
project(arm_foc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# --- 查找依赖 ---
find_package(rclcpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(realsense2 REQUIRED)
# 1. 查找包时，添加必要的组件（通常建议加上 rosidl_default_runtime）
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Pinocchio 手动路径设置 (保持你原有的)
set(PINOCCHIO_INCLUDE_DIR "/opt/ros/humble/include")
set(PINOCCHIO_LIB_DIR "/opt/ros/humble/lib/aarch64-linux-gnu")

find_library(PINOCCHIO_DEFAULT_LIB NAMES pinocchio_default PATHS ${PINOCCHIO_LIB_DIR} REQUIRED NO_DEFAULT_PATH)
find_library(PINOCCHIO_PARSERS_LIB NAMES pinocchio_parsers PATHS ${PINOCCHIO_LIB_DIR} REQUIRED NO_DEFAULT_PATH)

# --- 文件搜索 ---
file(GLOB_RECURSE SRC_FILES
    "src/*.cpp"
    "src/algorithms/*.cpp"
    "src/ctrl_step/*.cpp"
)

# --- 可执行文件 ---
# 只要 SRC_FILES 变量里包含了你的主程序文件，这样写就足够了
add_executable(${PROJECT_NAME} ${SRC_FILES})

# --- 包含目录 ---
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ctrl_step
    ${EIGEN3_INCLUDE_DIR}
    ${PINOCCHIO_INCLUDE_DIR}
)

# --- 链接库 ---
target_link_libraries(${PROJECT_NAME} PRIVATE
    rclcpp::rclcpp
    # 修改这里：使用具体的 C++ 接口
    action_msgs::action_msgs__rosidl_typesupport_cpp
    Threads::Threads
    Eigen3::Eigen
    yaml-cpp
    realsense2::realsense2
    ${OpenCV_LIBS}
    ${PINOCCHIO_DEFAULT_LIB}
    ${PINOCCHIO_PARSERS_LIB}
    m
)

# --- 运行时路径与安装设置 ---
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    BUILD_RPATH "${PINOCCHIO_LIB_DIR}"
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall
        -Wextra
        -Wno-unused-parameter
        -Wno-deprecated-declarations
        -Wno-psabi  # <--- 添加这一行，专门用来关闭这种 ABI 变动的提示
    )
endif()