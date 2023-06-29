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

# Add the core library
add_subdirectory(src/lib)

# Add main executables
add_executable(${PROJECT_NAME}_main_obsolete
	src/cad_image_markup.cpp
)

target_link_libraries(${PROJECT_NAME}_main_obsolete
	${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_main
	src/align_and_markup_cad.cpp
)

target_link_libraries(${PROJECT_NAME}_main
	${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_extract_edges_canny
	src/extract_edges_using_canny.cpp
)

target_link_libraries(${PROJECT_NAME}_extract_edges_canny
	${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_extract_annotated_defects
	src/extract_annotated_defects.cpp
)

target_link_libraries(${PROJECT_NAME}_extract_annotated_defects
	${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_visualize_markups
	src/visualize_markups.cpp
)

target_link_libraries(${PROJECT_NAME}_visualize_markups
	${PROJECT_NAME}
)

################## tests ##################
add_executable(${PROJECT_NAME}_run_convergence_test
  tests/convergence_tests.cpp
)

target_include_directories(${PROJECT_NAME}_run_convergence_test
  PUBLIC
    include/cad_image_markup
)

target_link_libraries(${PROJECT_NAME}_run_convergence_test
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_run_edge_extractor_canny_test
  tests/edge_extractor_canny_tests.cpp
)

target_include_directories(${PROJECT_NAME}_run_edge_extractor_canny_test
  PUBLIC
    include/cad_image_markup
)

target_link_libraries(${PROJECT_NAME}_run_edge_extractor_canny_test
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_run_defect_extractor_test
  tests/defect_extractor_tests.cpp
)

target_include_directories(${PROJECT_NAME}_run_defect_extractor_test
  PUBLIC
    include/cad_image_markup
)

target_link_libraries(${PROJECT_NAME}_run_defect_extractor_test
  ${PROJECT_NAME}
  Catch2::Catch2
)

add_executable(${PROJECT_NAME}_run_align_and_markup_test
  tests/align_and_markup_tests.cpp
)

target_include_directories(${PROJECT_NAME}_run_align_and_markup_test
  PUBLIC
    include/cad_image_markup
)

target_link_libraries(${PROJECT_NAME}_run_align_and_markup_test
  ${PROJECT_NAME}
  Catch2::Catch2
)