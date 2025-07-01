cmake_minimum_required(VERSION 3.16)
project(communicate_2025_aatest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_BUILD_TYPE "Debug")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

# find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(communicate_2025 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

ament_auto_add_executable(${PROJECT_NAME}_node
    DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}_node
    ${cpp_typesupport_target}
    ${OpenCV_LIBS}
)
install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)
ament_auto_package()
