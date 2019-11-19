CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

option(USE_EIGEN "Use Eigen instead of std::vector" OFF)

if(USE_EIGEN)
    add_definitions(-DUSE_EIGEN)
endif()
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})


set(LIBS "mpi_cxx")

add_executable(kdtest kd_tree_test.cpp KDTree.cpp)
target_link_libraries(kdtest ${LIBS})

add_executable(listtest test_lock.cpp)
target_link_libraries(listtest pthread)