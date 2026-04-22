cmake_minimum_required(VERSION 3.10)
project(gcs_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(drake REQUIRED)
find_package(CGAL REQUIRED)

# Include headers
include_directories(include)

# Build executable
add_executable(gcs_planner
    src/main.cpp
    src/gcs_planner_pointrobot.cpp
    src/free_space_cdt.cpp
    src/gcs_planner_robotarm.cpp
)

# Link Drake and CGAL
target_link_libraries(gcs_planner
    drake::drake
    CGAL::CGAL
)