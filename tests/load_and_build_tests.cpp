#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

#include <cad_image_markup/CadImageMarkup.h>
#include <cad_image_markup/Utils.h>

using namespace cad_image_markup;

std::string GetTestDataFilepath(const std::string& path_from_root) {
  boost::filesystem::path p(__FILE__);
  return utils::CombinePaths(p.parent_path().string(), path_from_root);
}

TEST_CASE("Initial test.") {
  // Define these at compile time for testing since catch has a hard time with
  // command line arguments
  CadImageMarkup::Inputs inputs{
      .cad_path = GetTestDataFilepath(
          utils::CombinePaths("labelled_images", "sim_CAD.json")),
      .image_path = GetTestDataFilepath(
          utils::CombinePaths("labelled_images", "-3.000000_0.000000.json")),
      .intrinsics_path = GetTestDataFilepath("Radtan_intrinsics.json"),
      .config_path = utils::GetConfigDataFilepath("SolutionParamsDefault.json"),
      .ceres_config_path =
          utils::GetConfigDataFilepath("CeresParamsDefault.json"),
      .initial_pose_path = GetTestDataFilepath(
          utils::CombinePaths("poses", "-3.000000_0.000000.json"))};

  CadImageMarkup markup(inputs);

  REQUIRE(1);
}

TEST_CASE("Solution test.") {
  // Define these at compile time for testing since catch has a hard time with
  // command line arguments
  CadImageMarkup::Inputs inputs{
      .cad_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/tests/test_data/"
          "labelled_images/sim_CAD.json",
      .image_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/tests/test_data/"
          "labelled_images/-3.000000_0.000000.json",
      .intrinsics_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/tests/test_data/"
          "Radtan_intrinsics.json",
      .config_path = "/home/user/sdic_cad_reprojection/cad_image_markup/config/"
                     "SolutionParamsDefault.json",
      .ceres_config_path =
          "/home/user/sdic_cad_reprojection/cad_image_markup/config/"
          "CeresParamsDefault.json",
      .initial_pose_path = "/home/user/sdic_cad_reprojection/cad_image_markup/"
                           "data/test1/initial_pose.json"};

  CadImageMarkup markup(inputs);

  REQUIRE(markup.Run());
}
