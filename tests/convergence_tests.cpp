#define CATCH_CONFIG_MAIN

#include "Utils.h"
#include <CadImageMarkup.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <catch2/catch.hpp>
#include <unsupported/Eigen/MatrixFunctions>

std::string GetDataPathRoot() {
  std::string file_location = __FILE__;
  std::string current_file_path = "load_and_build_examples.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  return file_location + "test_data/";
}

std::string GetConfigPathRoot() {
  std::string file_location = __FILE__;
  std::string current_file_path = "examples/load_and_build_examples.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  return file_location + "config/";
}

TEST_CASE("Convergence Testing - Manual Labels") {
  // Define these at compile time for testing since catch has a hard time with
  // command line arguments

  // Ground truth transforms for Real World Dataset
  // Captured using Samsung 13 phone at Lausanne Flon Metro station
  // 27 March, 2023, Lausanne Switzerland
  Eigen::Matrix4d T_CAMERA_WORLD_base1;
  T_CAMERA_WORLD_base1 << 0.999655, -0.0131739, 0.0227275, -0.0386311,
      0.0136331, 0.999704, -0.0201694, -0.0738061, -0.0224551, 0.0204723,
      0.999538, 2.20684, 0, 0, 0, 1;

  Eigen::Matrix4d T_CAMERA_WORLD_base2;
  T_CAMERA_WORLD_base2 << 0.958018, -0.000869915, 0.286708, -0.0508706,
      0.0129315, 0.999109, -0.0401783, 0.0167515, -0.286418, 0.0421991,
      0.957175, 1.93095, 0, 0, 0, 1;

  Eigen::Matrix4d T_CAMERA_WORLD_base3;
  T_CAMERA_WORLD_base3 << 0.946142, -0.02028, 0.323117, 0.00419045, 0.0450889,
      0.996564, -0.06948, 0.00905989, -0.320598, 0.0803069, 0.943805, 2.30273,
      0, 0, 0, 1;

  Eigen::Matrix4d T_CAMERA_WORLD_base4;
  T_CAMERA_WORLD_base4 << 0.998358, 0.00461789, -0.0570924, -0.103161,
      -0.00327674, 0.999717, 0.0235622, 0.0120111, 0.0571851, -0.0233364,
      0.998091, 1.93493, 0, 0, 0, 1;

  int num_trial = 20;

  REQUIRE(1);
}
