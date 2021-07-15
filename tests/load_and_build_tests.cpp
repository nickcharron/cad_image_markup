#define CATCH_CONFIG_MAIN

#include <CadImageMarkup.h>

#include <catch2/catch.hpp>

TEST_CASE("Initial test.") {
  // Define these at compile time for testing since catch has a hard time with
  // command line arguments
  cad_image_markup::CadImageMarkup::Inputs inputs{
      .cad_path =
          "/home/cameron/Projects/cad_image_markup/tests/test_data/"
          "labelled_images/sim_CAD.json",
      .image_path =
          "/home/cameron/Projects/cad_image_markup/tests/test_data/"
          "labelled_images/-3.000000_0.000000.json",
      .intrinsics_path =
          "/home/cameron/Projects/cad_image_markup/tests/test_data/"
          "Radtan_intrinsics.json",
      .config_path =
          "/home/cameron/Projects/cad_image_markup/config/"
          "SolutionParamsDefault.json",
      .ceres_config_path =
          "/home/cameron/Projects/cad_image_markup/config/"
          "CeresParamsDefault.json",
      .initial_pose_path =
          "/home/cameron/Projects/cad_image_markup/tests/test_data/poses/"
          "-3.000000_0.000000.json"};

  cad_image_markup::CadImageMarkup markup(inputs);

  REQUIRE(1);
}
