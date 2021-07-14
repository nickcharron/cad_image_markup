#define CATCH_CONFIG_MAIN

#include <CadImageMarkup.h>
#include <ImageBuffer.h>

#include <catch2/catch.hpp>
//#include <cad_image_markup/Gflags.h>
//#include <gflags/gflags.h>

/*
DEFINE_string(cad, "", "Full path to CAD file in json format (Required).");
DEFINE_validator(cad, &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(image, "", "Full path to image (Required).");
DEFINE_validator(image, &cad_image_markup::gflags::ValidateFileMustExist);
DEFINE_string(intrinsics, "",
              "Full path to intrinsics file in json format (Required).");
DEFINE_validator(intrinsics,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(config, "", "Full path to json config file (Optional).");
DEFINE_string(ceres_config, "",
              "Full path to json config file for Ceres optimizer (Optional).");
DEFINE_string(initial_pose, "",
              "Full path to initial pose json (Optional). This is the "
              "translation and rotation of the camera w.r.t. the world frame. "
              "The world frame is the centroid of the structural element in "
              "the CAD mode. (TODO CAM: Is this correct?)");
*/


TEST_CASE("Initial test.") {

  // Define these at compile time for testing since catch has a hard time with command line arguments
  cad_image_markup::CadImageMarkup::Inputs inputs{
      .cad_path = "/home/cameron/Projects/cad_image_markup/tests/test_data/labelled_images/sim_CAD.json",
      .image_path = "/home/cameron/Projects/cad_image_markup/tests/test_data/labelled_images/-3.000000_0.000000.json",
      .intrinsics_path = "/home/cameron/Projects/cad_image_markup/tests/test_data/Radtan_intrinsics.json",
      .config_path = "/home/cameron/Projects/cad_image_markup/config/SolutionParamsDefault.json",
      .ceres_config_path = "/home/cameron/Projects/cad_image_markup/config/CeresParamsDefault.json",
      .initial_pose_path = "/home/cameron/Projects/cad_image_markup/tests/test_data/poses/-3.000000_0.000000.json"
  };

  cad_image_markup::Params params;

  cad_image_markup::CadImageMarkup markup(inputs,params);
    
    REQUIRE(1);
}
