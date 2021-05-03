SET(CMAKE_CXX_STANDARD 17)

# Core library
add_library(${PROJECT_NAME}
  camera_models/CameraModel.cpp
  camera_models/DoubleSphere.cpp
  camera_models/KannalaBrandt.cpp
  camera_models/Radtan.cpp
  CadImageMarkup.cpp
  Gflags.cpp
)
target_compile_options(${PROJECT_NAME}
  PUBLIC
    -march=native -msse4.2 -mfpmath=sse
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ../../include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ceres
  nlohmann_json::nlohmann_json
)
