#include <cad_image_markup/ImageDefectExtractor.h>
#include <cad_image_markup/Gflags.h>
#include <cad_image_markup/Log.h>

DEFINE_string(image_path, "", "Full path to image (Required)");
DEFINE_validator(image_path, &cad_image_markup::gflags::ValidateFileMustExist);
DEFINE_string(output_json, "", "Full path to output json (Required)");
DEFINE_string(defect_color, "", "Defect annotation color (red, blue, or green)");

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cad_image_markup::ImageDefectExtractor extractor(FLAGS_image_path,FLAGS_defect_color);

  if (extractor.ExtractDefects()) {
    LOG_INFO("Successfully extracted defects!");
  } else {
    LOG_ERROR("Failed extracting defects!");
  }

  if (extractor.SaveResults(FLAGS_output_json)) {
    LOG_INFO("Successfully saved defects!");
  } else {
    LOG_ERROR("Failed saving defects!");
  }
  return 0;
}