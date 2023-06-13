#include <cad_image_markup/CadImageMarkup.h>
#include <cad_image_markup/Gflags.h>
#include <cad_image_markup/Log.h>

DEFINE_string(image_labels, "", "Full path to image labels json (Required).");
DEFINE_validator(image_labels,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(cad_labels, "", "Full path to cad labels json (Required).");
DEFINE_validator(cad_labels,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(defect_labels, "", "Full path to defect labels json (Required).");
DEFINE_validator(defect_labels,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);

DEFINE_string(intrinsics, "",
              "Full path to intrinsics file in json format (Required).");
DEFINE_validator(intrinsics,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(output_directory, "",
              "Full path to output directory. We will generate a new folder "
              "inside this direcory named based on time of run (Required)");
DEFINE_validator(output_directory,
                 &cad_image_markup::gflags::ValidateDirMustExist);
DEFINE_string(
    initial_pose, "",
    "Full path to initial pose json (Optional). This is the "
    "translation and rotation of the camera w.r.t. the world frame. "
    "The world frame is the centroid of the structural element in "
    "the CAD mode. If not provided, we will assume the inspector was ~3m from "
    "the surface and the image was collected perpendicular to the surface.");
DEFINE_string(config, "", "Full path to json config file (Optional).");
DEFINE_string(ceres_config, "",
              "Full path to json config file for Ceres optimizer (Optional).");

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  /** TODO [CAM]: uncomment once ready
  cad_image_markup::CadImageMarkup::Inputs inputs{
      .image_labels = FLAGS_image_labels,
      .cad_labels = FLAGS_cad_labels,
      .defect_labels = FLAGS_defect_labels,
      .intrinsics = FLAGS_intrinsics,
      .initial_pose = FLAGS_initial_pose,
      .config = FLAGS_config,
      .ceres_config = FLAGS_ceres_config};

  cad_image_markup::CadImageMarkup markup(inputs);
  if (markup.Run()) {
    LOG_INFO("Success completed CAD markup!");
    markup.SaveResults(FLAGS_output_directory);
  } else {
    LOG_ERROR("Failed CAD markup!");
  }
  */
  return 0;
}
