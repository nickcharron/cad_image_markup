#include <cad_image_markup/CadImageMarkup.h>
#include <cad_image_markup/Gflags.h>
#include <cad_image_markup/Log.h>

DEFINE_string(cad, "", "Full path to CAD file in json format (Required).");
DEFINE_validator(cad, &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(cad_image, "",
              "Full path to CAD image file in png or jpg (Required).");
DEFINE_validator(cad_image, &cad_image_markup::gflags::ValidateFileMustExist);
DEFINE_string(canny_edge_cad, "",
              "Full path to canny_edge detected cad image (Optional, used when "
              "AUTOMATIC feature detection is enabled).");
DEFINE_string(image, "", "Full path to image (Required).");
DEFINE_validator(image, &cad_image_markup::gflags::ValidateFileMustExist);
DEFINE_string(canny_edge_image, "",
              "Full path to canny_edge detected image (Optional, used when "
              "AUTOMATIC feature detection is enabled).");
DEFINE_string(intrinsics, "",
              "Full path to intrinsics file in json format (Required).");
DEFINE_validator(intrinsics,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(solution_config, "", "Full path to json solution config file (Optional).");
DEFINE_string(ceres_config, "",
              "Full path to json config file for Ceres optimizer (Optional).");
DEFINE_string(canny_config, "",  "Full path to json canny config file (Optional).");
DEFINE_string(defect_image, "",
              "Full path to image file with labelled defects (Optional)");
DEFINE_string(output_directory, "", "Full path to output directory (Optional)");
DEFINE_string(
    initial_pose, "",
    "Full path to initial pose json (Optional). This is the "
    "translation and rotation of the camera w.r.t. the world frame "
    "(T_WORLD_CAM). The world frame is the centroid of the structural element "
    "in the CAD mode. If not provided, we will assume the inspector was ~3m "
    "from the surface and the image was collected perpendicular to the "
    "surface.");

int main(int argc, char** argv) {

// OBSOLETE:
/*
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cad_image_markup::CadImageMarkup::Inputs inputs{
      .cad_path = FLAGS_cad,
      .cad_image_path = FLAGS_cad_image,
      .canny_edge_cad_path = FLAGS_canny_edge_cad,
      .image_path = FLAGS_image,
      .canny_edge_image_path = FLAGS_canny_edge_image,
      .defect_path = FLAGS_defect_image,
      .intrinsics_path = FLAGS_intrinsics,
      .solution_config_path = FLAGS_solution_config,
      .ceres_config_path = FLAGS_ceres_config,
      .canny_config_path = FLAGS_canny_config,
      .initial_pose_path = FLAGS_initial_pose};

  cad_image_markup::CadImageMarkup markup(inputs);
  if (markup.Run()) {
    LOG_INFO("Success completed CAD markup!");
  } else {
    LOG_ERROR("Failed CAD markup!");
  }
  markup.SaveResults(FLAGS_output_directory);
*/
  return 0;
}
