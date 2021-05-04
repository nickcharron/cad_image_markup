#include <cad_image_markup/CadImageMarkup.h>
#include <cad_image_markup/Gflags.h>
#include <gflags/gflags.h>

DEFINE_string(cad, "", "Full path to CAD file in json format (Required).");
DEFINE_validator(cad, &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(image, "", "Full path to image (Required).");
DEFINE_validator(image, &cad_image_markup::gflags::ValidateFileMustExist);
DEFINE_string(intrinsics, "",
              "Full path to intrinsics file in json format (Required).");
DEFINE_validator(intrinsics,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(config, "", "Full path to json config file (Required).");
DEFINE_validator(config, &cad_image_markup::gflags::ValidateJsonFileMustExist);

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cad_image_markup::CadImageMarkup::Inputs inputs{
      .cad_path = FLAGS_cad,
      .image_path = FLAGS_image,
      .intrinsics_path = FLAGS_intrinsics,
      .config_path = FLAGS_config};

  cad_image_markup::CadImageMarkup markup(inputs);
  if (markup.Run()) {
    LOG_INFO("Success completed CAD markup!");
  } else {
    LOG_ERROR("Failed CAD markup!");
  }
  return 0;
}