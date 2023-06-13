#include <cad_image_markup/Gflags.h>
#include <cad_image_markup/MarkupVisualizer.h>
#include <cad_image_markup/Log.h>

DEFINE_string(image_path, "", "Full path to image (Required)");
DEFINE_validator(image_path, &cad_image_markup::gflags::ValidateFileMustExist);
DEFINE_string(labels_path, "", "Full path to image (Required)");
DEFINE_validator(labels_path,
                 &cad_image_markup::gflags::ValidateJsonFileMustExist);
DEFINE_string(output_file, "",
              "Full path to output image. If empty, it will only show the "
              "image, otherwise it will save it (Optional)");

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cad_image_markup::MarkupVisualizer viz(FLAGS_image_path, FLAGS_labels_path);
  std::string output_file = FLAGS_output_file;
  if (output_file.empty()) {
    LOG_INFO("visualizing markup...");
    viz.Visualize();
  } else {
    LOG_INFO("saving markup to: %s", output_file.c_str());
    viz.Save(output_file);
  }
  return 0;
}
