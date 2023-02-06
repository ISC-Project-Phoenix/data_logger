cmake_minimum_required(VERSION 3.8)
project(data_logger)
include(FetchContent)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)

add_executable(data_logger src/data_logger.cpp src/data_logger_node.cpp)
target_include_directories(data_logger PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(data_logger PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Make deps a variable so they get linked to tests as well
set(dependencies
        rclcpp
        ackermann_msgs
        sensor_msgs
        std_msgs
        cv_bridge
        OpenCV
        )

ament_target_dependencies(
        data_logger
        ${dependencies}
)

install(TARGETS data_logger
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

    find_package(ament_cmake_gtest REQUIRED)

    # Add unit tests
    ament_add_gtest(dl_unit_tests tests/unit.cpp)
    ament_target_dependencies(dl_unit_tests ${dependencies})
endif ()

ament_package()
