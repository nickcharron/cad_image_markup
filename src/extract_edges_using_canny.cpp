#include <cad_image_markup/EdgeExtractorCanny.h>
#include <cad_image_markup/Gflags.h>
#include <cad_image_markup/Log.h>

DEFINE_string(image_path, "", "Full path to image (Required)");
DEFINE_validator(image_path, &cad_image_markup::gflags::ValidateFileMustExist);
DEFINE_string(output_json, "", "Full path to output json (Required)");
DEFINE_string(config, "", "Full path to json config file (Optional)");

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cad_image_markup::EdgeExtractorCanny extractor(FLAGS_image_path,
                                                 FLAGS_config);
  if (extractor.ExtractEdges()) {
    LOG_INFO("Successfully extracted edges!");
    extractor.SaveResults(FLAGS_output_json);
  } else {
    LOG_ERROR("Failed extracting edges!");
  }
  return 0;
}
