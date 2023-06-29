#include <fstream>

#include <cad_image_markup/ImageDefectExtractor.h>
#include <cad_image_markup/Log.h>
#include <cad_image_markup/nlohmann/json.h>


namespace cad_image_markup {

ImageDefectExtractor::ImageDefectExtractor(const std::string& image_path,
                                       const std::string& defect_color) {
  defect_color_ = defect_color;
  image_path_ = image_path;
}


bool ImageDefectExtractor::ExtractDefects() {

    defect_cloud_ = std::make_shared<PointCloud>();

    return utils::ReadPointsPNG(image_path_, defect_cloud_,
                         defect_color_, 1);
                        
}

bool ImageDefectExtractor::SaveResults(const std::string& output_json) {

  // Generate json file from the edge points
  nlohmann::json J = CreateDefectJSON();

  std::ofstream file(output_json);
  file << std::setw(4) << J;
  
  return true;
}

nlohmann::json ImageDefectExtractor::CreateDefectJSON() {

  nlohmann::json j; 
  nlohmann::json j_aux;

  std::vector<std::vector<int>> defect_points;

  for (uint32_t i = 0; i < defect_cloud_->size(); i++) {
    defect_points.push_back({(int)defect_cloud_->at(i).x, (int)defect_cloud_->at(i).y});
  }

  j["shapes"] = {};
  j_aux["points"] = defect_points;

  j["shapes"].push_back(j_aux);

  return j;

}

} // namespace cad_image_markup