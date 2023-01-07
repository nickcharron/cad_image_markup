#define CATCH_CONFIG_MAIN

#include <CadImageMarkup.h>

#include <catch2/catch.hpp>

TEST_CASE("Initial test.") {
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
          "/home/user/sdic_cad_reprojection/cad_image_markup/tests/test_data/poses/"
          "-3.000000_0.000000.json"};

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
  cad_image_markup::CadImageMarkup::Inputs inputs{
      .cad_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/tests/test_data/"
          "labelled_images/sim_CAD.json",
      .cad_image_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/tests/test_data/cad/cad.png",
      .image_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/tests/test_data/"
          "labelled_images/-3.000000_0.000000.json",
      .defect_path = 
          "/home/user/sdic_cad_reprojection/cad_image_markup/tests/test_data/marked_up_images/-3.000000_0.000000_mk.png",
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
          "/home/user/sdic_cad_reprojection/cad_image_markup/data/test1/initial_pose.json", 
      .output_image_path = 
          "/home/user/sdic_cad_reprojection/cad_image_markup/tests/test_data/marked_up_cad/-3.000000_0.000000_cad_mk.png" };

  cad_image_markup::CadImageMarkup markup(inputs);

  REQUIRE(markup.Run());
}
