#include <cad_image_markup/EdgeExtractorCanny.h>
#include <cad_image_markup/ImageBuffer.h>
#include <cad_image_markup/Log.h>

namespace cad_image_markup {

EdgeExtractorCanny::EdgeExtractorCanny(const std::string& image_path,
                                       const std::string& config) {
  LoadConfig(config);
  LoadImage(image_path);
}

void EdgeExtractorCanny::LoadConfig(const std::string& config) {
  // TODO
}

void EdgeExtractorCanny::LoadImage(const std::string& image_path) {
  // TODO
}

bool EdgeExtractorCanny::ExtractEdges() {
  // TODO
  return false;
}

void EdgeExtractorCanny::SaveResults(const std::string& output_json) {
  // TODO
}

} // namespace cad_image_markup