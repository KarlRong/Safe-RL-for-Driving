cmake_minimum_required(VERSION 3.5)
project (fmt)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -O0 -g")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(Boost  REQUIRED COMPONENTS filesystem system)
# check if boost was found
if(Boost_FOUND)
    message ("boost found")
else()
    message (FATAL_ERROR "Cannot find Boost")
endif()

find_package(ccd QUIET)
find_package(fcl REQUIRED)
find_package(Eigen3 3.0.5 QUIET CONFIG)

find_package(Python2 COMPONENTS Development NumPy)

set(SOURCES
    main.cpp
    src/doubleintegrator.cpp
    src/doublebvp.cpp
    src/doublebvp.cpp
    src/fmt.cpp
    src/world.cpp
)

add_executable(fmt  ${SOURCES})

target_include_directories(fmt
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
        ${FCL_INCLUDE_DIRS}
        ${CCD_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Python2_INCLUDE_DIRS} 
)

target_link_libraries( fmt
    PRIVATE
        # Boost::filesystem
        ${Boost_LIBRARIES}
        # ${fcl_LIBRARIES}
        fcl
        ${CCD_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${M_LIBRARY}
        Python2::Python 
)
