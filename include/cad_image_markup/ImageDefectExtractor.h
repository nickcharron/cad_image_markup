#pragma once

#include <string>

#include <cad_image_markup/Utils.h>
#include <cad_image_markup/Params.h>

namespace cad_image_markup {

class ImageDefectExtractor {
public:

  /**
   * @brief constructor
   */
  ImageDefectExtractor(const std::string& image_path, const std::string& defect_color);

  /**
   * @brief default deconstructor
   */
  ~ImageDefectExtractor() = default;

  bool ExtractDefects();

  bool SaveResults(const std::string& output_json);

private:

  std::string image_path_;
  std::string defect_color_;
  PointCloud::Ptr defect_cloud_;

  nlohmann::json CreateDefectJSON();

};

} // namespace cad_image_markup