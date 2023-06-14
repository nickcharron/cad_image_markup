#include <cad_image_markup/EdgeExtractorCanny.h>
#include <cad_image_markup/ImageBuffer.h>
#include <cad_image_markup/Log.h>

namespace cad_image_markup {

EdgeExtractorCanny::EdgeExtractorCanny(const std::string& image_path,
                                       const std::string& config_path) {
  
  if (LoadConfig(config_path));
    LOG_INFO("MARKUP: CANNY: Successfully loaded canny params.");
  image_path_ = image_path;
}

bool EdgeExtractorCanny::LoadConfig(const std::string& config_path) {

  if (!params_.LoadCannyParamsFromJSON(config_path)) {
    LOG_ERROR("MARKUP: CANNY: Could not load solution params. Exiting ...");
    return false;
  }

  return true;

}


bool EdgeExtractorCanny::ExtractEdges() {
  /*
  ImageBuffer::CannyEdgeDetectToCloud(image_path_,
                                      edges_cloud_,
                                      params_.cannny_low_threshold,
                                      params_.canny_ratio,
                                      params_.canny_kernel_size);
  */
  return false;
}

void EdgeExtractorCanny::SaveResults(const std::string& output_json) {
  // TODO
}

} // namespace cad_image_markup