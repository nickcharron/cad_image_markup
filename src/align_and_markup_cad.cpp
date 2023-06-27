#include <cad_image_markup/CadImageMarkup.h>
#include <cad_image_markup/Gflags.h>
#include <cad_image_markup/Log.h>

DEFINE_string(image_label_path, "", "Full path to image labels json (Required).");
DEFINE_validator(image_label_path,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(image_path, "", "Full path to image file (Required).");
DEFINE_validator(image_path,
                 &cad_image_markup::gflags::ValidateFileMustExist);
DEFINE_string(cad_label_path, "", "Full path to cad labels json (Required).");
DEFINE_validator(cad_label_path,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(cad_image_path, "", "Full path to cad section (in image format) (Required).");
DEFINE_validator(cad_image_path,
                 &cad_image_markup::gflags::ValidateFileMustExist);
DEFINE_string(defect_path, "", "Full path to defect labels json (Required).");
DEFINE_validator(defect_path,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(intrinsics_path, "",
              "Full path to camera intrinsics file in json format (Required).");
DEFINE_validator(intrinsics_path,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(initial_pose_path, "",
              "Full path to initial camera pose estimate in json format (Optional).");
DEFINE_validator(initial_pose_path,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(solution_config_path, "",
              "Full path to solution configurations in json format (Optional).");
DEFINE_validator(solution_config_path,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(ceres_config_path, "",
              "Full path to ceres solver configurations in json format (Optional).");
DEFINE_validator(ceres_config_path,
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
DEFINE_string(solution_config, "", "Full path to json solution config file (Optional).");
DEFINE_string(ceres_config, "",
              "Full path to json config file for Ceres optimizer (Optional).");

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cad_image_markup::CadImageMarkup::Inputs inputs{
      .image_label_path = FLAGS_image_label_path,
      .image_path = FLAGS_image_path, 
      .cad_label_path = FLAGS_cad_label_path,
      .cad_image_path = FLAGS_cad_image_path, 
      .defect_path = FLAGS_defect_path, 
      .intrinsics_path = FLAGS_intrinsics_path, 
      .initial_pose_path = FLAGS_initial_pose_path, 
      .solution_config_path = FLAGS_solution_config_path, 
      .ceres_config_path = FLAGS_ceres_config_path};

  cad_image_markup::CadImageMarkup markup(inputs);
  if (markup.Run()) {
    LOG_INFO("Success completed CAD markup!");
    if(markup.SaveResults(FLAGS_output_directory));
      LOG_INFO("Saved CAD markup results!");
  } else {
    LOG_ERROR("Failed CAD markup!");
  }
  return 0;
}
