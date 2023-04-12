# PROJECT CONFIGURATION
cmake_minimum_required(VERSION 3.5)

project(DPGO)
set(CMAKE_CXX_STANDARD 17)

# Thirdparty code (ROPTLIB)
add_subdirectory(thirdparty)

# Dependencies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(SPQR REQUIRED)
find_package(Cholmod REQUIRED)
find_package(Threads REQUIRED)
find_package(Glog REQUIRED)

set(EXTERNAL_INCLUDES ${SPQR_INCLUDES} ${CHOLMOD_INCLUDES})
set(EXTERNAL_LIBRARIES roptlib glog ${SPQR_LIBRARIES} ${CHOLMOD_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

# Eigen Source
option(BUILD_AGAINST_GTSAM_EIGEN 
	"If True builds against GTSAM's internal EIGEN else uses system eigen" 
	ON
)
if(BUILD_AGAINST_GTSAM_EIGEN)
	message(STATUS "Building DPGO against GTSAM's Eigen")
	find_package(GTSAM REQUIRED)
	list(APPEND EXTERNAL_INCLUDES ${GTSAM_INCLUDE_DIR})
	list(APPEND EXTERNAL_LIBRARIES gtsam)
else()
	message(STATUS "Building DPGO against System Eigen")
	find_package(Eigen3 REQUIRED)
	list(APPEND EXTERNAL_INCLUDES ${EIGEN3_INCLUDE_DIR})
	list(APPEND EXTERNAL_LIBRARIES Eigen3::Eigen)
endif()

# Aggregate the library soures
file(GLOB_RECURSE dpgo_srcs 
	"${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" 
	"${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
add_library(DPGO SHARED ${dpgo_srcs})

target_link_libraries(DPGO PUBLIC ${EXTERNAL_LIBRARIES})
target_include_directories(DPGO PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" ${EXTERNAL_INCLUDES})

# Build Distributed PGO example
add_executable(multi-robot-example examples/MultiRobotExample.cpp)
target_link_libraries(multi-robot-example DPGO)

add_executable(chordal-initialization-example examples/ChordalInitializationExample.cpp)
target_link_libraries(chordal-initialization-example DPGO)

add_executable(single-robot-example examples/SingleRobotExample.cpp)
target_link_libraries(single-robot-example DPGO)

# Add testing
option(BUILD_DPGO_TESTS "Build tests" OFF)
if(BUILD_DPGO_TESTS)
	find_package(GTest)
	add_subdirectory(tests)
endif(BUILD_DPGO_TESTS)