#include <fstream>

#include <cad_image_markup/EdgeExtractorCanny.h>
#include <cad_image_markup/Log.h>
#include <cad_image_markup/nlohmann/json.h>


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


void EdgeExtractorCanny::ExtractEdges() {
  PointCloud::Ptr edges_cloud_interim = std::make_shared<PointCloud>();;

  utils::CannyEdgeDetectToCloud(image_path_,
                                edges_cloud_interim,
                                params_.cannny_low_threshold,
                                params_.canny_ratio,
                                params_.canny_kernel_size);

  if (params_.downsample_image_cloud)  {
    edges_cloud_ = utils::DownSampleCloud(edges_cloud_interim, params_.downsample_grid_size);
  }
  else {
    edges_cloud_ = edges_cloud_interim;
  }
  
  return;
                        
}

bool EdgeExtractorCanny::SaveResults(const std::string& output_json) {

  // Generate json file from the edge points
  nlohmann::json J = CreateEdgesJSON();

  std::ofstream file(output_json);
  file << J;
  
  return true;
}

PointCloud::Ptr EdgeExtractorCanny::GetEdgesCloud() {
  return edges_cloud_;
}

nlohmann::json EdgeExtractorCanny::CreateEdgesJSON() {

  nlohmann::json j; 
  nlohmann::json j_aux;

  std::vector<std::vector<int>> edges_points;

  for (uint32_t i = 0; i < edges_cloud_->size(); i++) {
    edges_points.push_back({(int)edges_cloud_->at(i).x, (int)edges_cloud_->at(i).y});
  }

  j["shapes"] = {};
  j_aux["points"] = edges_points;

  j["shapes"].push_back(j_aux);

  return j;

}

} // namespace cad_image_markup