cmake_minimum_required(VERSION 3.5)
project(metro_benchmark_pub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(metro_benchmark_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} src/benchmark_publisher.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  metro_benchmark_msgs
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_utest test/utest.cpp)
  target_include_directories(${PROJECT_NAME}_utest PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME})

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(timer_test test/test_timer.py
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif()

install(FILES resource/${PROJECT_NAME}
        DESTINATION share/${PROJECT_NAME}/resource
)
ament_python_install_package(${PROJECT_NAME})

ament_package()
