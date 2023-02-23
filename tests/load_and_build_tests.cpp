#define CATCH_CONFIG_MAIN

#include <CadImageMarkup.h>

#include <catch2/catch.hpp>

std::string GetDataPathRoot() {
  std::string file_location = __FILE__;
  std::string current_file_path = "load_and_build_tests.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  return file_location + "test_data/";
}

std::string GetConfigPathRoot() {
  std::string file_location = __FILE__;
  std::string current_file_path = "tests/load_and_build_tests.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  return file_location + "config/";
}

TEST_CASE("Initial test.") {
  // Define these at compile time for testing since catch has a hard time with
  // command line arguments
  std::string data_root = GetDataPathRoot();
  std::string config_root = GetConfigPathRoot();
  cad_image_markup::CadImageMarkup::Inputs inputs{
      .cad_path = data_root + "labelled_images/sim_CAD.json",
      .cad_image_path = data_root + "cad/cad.png",
      .image_path = data_root + "labelled_images/-3.000000_0.000000.json",
      .defect_path = data_root + "marked_up_images/-3.000000_0.000000_mk.png",
      .intrinsics_path = data_root + "Radtan_intrinsics.json",
      .config_path = config_root + "SolutionParamsDefault.json",
      .ceres_config_path = config_root + "CeresParamsDefault.json",
      .initial_pose_path = data_root + "poses/-3.000000_0.000000.json",
      .output_image_path =
          data_root + "marked_up_cad/-3.000000_0.000000_cad_mk.png"};

  cad_image_markup::CadImageMarkup markup(inputs);

  REQUIRE(1);
}
/*
TEST_CASE("Solution test.") {
  // Define these at compile time for testing since catch has a hard time with
  // command line arguments
  cad_image_markup::CadImageMarkup::Inputs inputs{
      .cad_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/tests/test_data/"
          "labelled_images/sim_CAD.json",
      .image_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/tests/test_data/"
          "labelled_images/-3.000000_0.000000.json",
      .intrinsics_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/tests/test_data/"
          "Radtan_intrinsics.json",
      .config_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/config/"
          "SolutionParamsDefault.json",
      .ceres_config_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/config/"
          "CeresParamsDefault.json",
      .initial_pose_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/data/test1/initial_pose.json"};

  cad_image_markup::CadImageMarkup markup(inputs);

  REQUIRE(markup.Run());
}
*/
TEST_CASE("Defect transfer test.") {
  // Define these at compile time for testing since catch has a hard time with
  // command line arguments
  std::string data_root = GetDataPathRoot();
  std::string config_root = GetConfigPathRoot();
  cad_image_markup::CadImageMarkup::Inputs inputs{
      .cad_path = data_root + "labelled_images/sim_cad_feature_label.json",
      .cad_image_path = data_root + "cad/sim_cad.png",
      .image_path = data_root + "labelled_images/sim_view_2_feature_label.json",
      .defect_path = data_root + "marked_up_images/sim_view_2_image_markup.png",
      .intrinsics_path = data_root + "Radtan_intrinsics.json",
      .config_path = config_root + "SolutionParamsDefault.json",
      .ceres_config_path = config_root + "CeresParamsDefault.json",
      .initial_pose_path = data_root + "poses/initial_pose.json",
      .output_image_path =
          data_root + "marked_up_cad/sim_cad_view_2_markup.png"};

  cad_image_markup::CadImageMarkup markup(inputs);

  REQUIRE(markup.Run());
}
