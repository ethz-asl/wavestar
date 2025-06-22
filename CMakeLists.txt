set(WAVEMAP_VERSION v2.2.1)
cmake_minimum_required(VERSION 3.18)
project(rss2025_paper499_code)

# Find OMPL
find_package(ompl REQUIRED)

# Load wavemap
include(FetchContent)
FetchContent_Declare(wavemap
    GIT_REPOSITORY https://github.com/ethz-asl/wavemap.git
    GIT_TAG ${WAVEMAP_VERSION}
    GIT_SHALLOW 1
    SOURCE_SUBDIR library/cpp)
FetchContent_MakeAvailable(wavemap)

# Library
add_library(${PROJECT_NAME}
    src/a_star.cc
    src/theta_star.cc
    src/lazy_theta_star.cc
    src/ompl_planner.cc
    src/multi_res_theta_star.cc
    src/multi_res_lazy_theta_star.cc
    src/planner_base.cc
    src/planner_factory.cc
    src/pointwise_heuristics.cc
    src/volumetric_heuristics.cc)
set_wavemap_target_properties(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${OMPL_LIBRARIES} wavemap::wavemap_core)
