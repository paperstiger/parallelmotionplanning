CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

option(USE_EIGEN "Use Eigen instead of std::vector" OFF)

if(USE_EIGEN)
    add_definitions(-DUSE_EIGEN)
endif()
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} prrts-c/src/)

set(KD_FILE kdtree.h kdtree.c alloc.cpp alloc.h)

add_executable(listtest test_lock.cpp)
target_link_libraries(listtest pthread)

add_executable(serialrrt serial_rrt.cpp KDTree.cpp)

add_executable(serialrrt_kd serial_rrt_kd.cpp kdtree.c alloc.cpp)
target_link_libraries(serialrrt_kd pthread)

add_executable(prrt parallel_rrt_kd.cpp kdtree.c alloc.cpp)
target_link_libraries(prrt pthread)

add_executable(prrts_nomem parallel_rrtstar_kd.cpp kdtree.c alloc.cpp)
target_link_libraries(prrts_nomem pthread)

add_executable(prrts prrtstar_memory.cpp kdtree.c kdtree.h alloc.cpp alloc.h tree_node.h planner_common.h prrts-c/src/naocup.c prrts-c/src/hrtimer.c prrts-c/src/linear.c prrts-c/src/collide.c
prrts-c/src/prrts.c prrts-c/src/mt19937a.c prrts-c/src/stats.c)
target_link_libraries(prrts pthread)

add_executable(kinorrt kino_rrt_entry.cpp kino_rrt_test_pen.cpp kino_rrt_test_acrobot.cpp kino_tree_helper.h kino_tree_helper.cpp ${KD_FILE})
target_link_libraries(kinorrt pthread)

#add_library(naocup prrts-c/src/naocup.o)
#add_executable(naocup prrts-c/src/naocup.o)
#target_link_libraries(naocup)