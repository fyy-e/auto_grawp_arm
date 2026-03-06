cmake_minimum_required(VERSION 3.10)

project(arm_foc)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# 查找必要的库
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)  # 用于查找 pinocchio
find_package(yaml-cpp REQUIRED)

# 使用 pkg-config 查找 pinocchio（如果已安装）
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

# 包含目录
include_directories(.)

# 子目录（如果有需要可以添加）
# add_subdirectory(algorithms)
# add_subdirectory(ctrl_step)

# 收集所有源文件
file(GLOB_RECURSE SRC_FILES
    "src/*.cpp"
    "src/algorithms/*.cpp"           # 新增：确保能搜到 VisionGraspPlanner.cpp
    "src/algorithms/kinematic/*.cpp"
    "src/ctrl_step/*.cpp"
)

file(GLOB_RECURSE HEADER_FILES
    "src/*.h"
    "src/algorithms/*.h"           # 新增：确保能搜到 VisionGraspPlanner.h
    "src/algorithms/kinematic/*.h"
    "src/ctrl_step/*.h"
)

# 显示找到的文件
message(STATUS "Found source files: ${SRC_FILES}")
message(STATUS "Found header files: ${HEADER_FILES}")

# 创建可执行文件
add_executable(${PROJECT_NAME}
    ${SRC_FILES}
    ${HEADER_FILES}
)

# 设置目标属性 - 关键修改：添加 Eigen3 和 Pinocchio 路径
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms           # 新增：允许直接引用 algorithms/xxx.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/kinematic
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ctrl_step
    ${EIGEN3_INCLUDE_DIR}
    ${PINOCCHIO_INCLUDE_DIRS}
)

# 定义 URDF 路径宏（方便代码中引用）
target_compile_definitions(${PROJECT_NAME} PRIVATE
    URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/urdf/"
)

# 链接必要的库 - 关键修改：添加 Pinocchio
target_link_libraries(${PROJECT_NAME} PRIVATE
    Threads::Threads
    Eigen3::Eigen                      # 显式链接 Eigen
    ${PINOCCHIO_LIBRARIES}             # Pinocchio 库
    m                                  # 数学库
    yaml-cpp                            # YAML 库
)

# 设置链接选项（如果需要）
target_link_options(${PROJECT_NAME} PRIVATE
    ${PINOCCHIO_LDFLAGS_OTHER}
)

# 设置输出目录
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# 添加安装规则（可选）
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# 添加测试（可选）
enable_testing()

# 如果使用Qt（可选，如果需要GUI）
# find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
# target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Core Qt5::Widgets)

# 添加编译警告选项
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall
        -Wextra
        -Wno-unused-parameter
        -Wno-deprecated-declarations
        ${PINOCCHIO_CFLAGS_OTHER}      # Pinocchio 编译选项
    )
endif()

# 如果使用MSVC
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        /W4
        /WX
    )
endif()