cmake_minimum_required(VERSION 3.14) 
project(cad_image_markup)

SET(CMAKE_CXX_STANDARD 17)

#find_package(gflags REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10.0 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Catch2 REQUIRED)
find_package(Ceres REQUIRED)

# TODO uninstall pcl 1.11.1 and install 1.8 and vtk 7.1 as described in the site that worked last time 

# Add the core library
add_subdirectory(src/lib)

# Add main executables
add_executable(${PROJECT_NAME}_main
	src/cad_image_markup.cpp
)

target_link_libraries(${PROJECT_NAME}_main
	${PROJECT_NAME}
  #gflags
)

################## tests ##################
add_executable(${PROJECT_NAME}_load_and_build_tests
  tests/load_and_build_tests.cpp
)

target_include_directories(${PROJECT_NAME}_load_and_build_tests
  PUBLIC
    include/cad_image_markup
)

target_link_libraries(${PROJECT_NAME}_load_and_build_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)