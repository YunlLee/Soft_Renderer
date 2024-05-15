cmake_minimum_required(VERSION 3.5)

project(games101)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR "${PROJECT_SOURCE_DIR}/OpenCV")
find_package(OpenCV REQUIRED)

file(GLOB source CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.cpp" 
    "${PROJECT_SOURCE_DIR}/src/*.hpp")
message(STATUS "INFO: ${source}")

add_executable(${PROJECT_NAME} ${source})
target_include_directories(${PROJECT_NAME} 
    PUBLIC ${PROJECT_SOURCE_DIR}/Eigen3/include
    PUBLIC ${PROJECT_SOURCE_DIR}/OpenCV/include)
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBRARIES})