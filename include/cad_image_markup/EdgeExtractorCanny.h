#pragma once

#include <string>

#include <cad_image_markup/Utils.h>
#include <cad_image_markup/Params.h>

namespace cad_image_markup {

class EdgeExtractorCanny {
public:

  /**
   * @brief constructor
   */
  EdgeExtractorCanny(const std::string& image_path, const std::string& config);

  /**
   * @brief default deconstructor
   */
  ~EdgeExtractorCanny() = default;

  bool ExtractEdges();

  void SaveResults(const std::string& output_json);

private:

  Params params_;
  std::string image_path_;
  PointCloud::Ptr edges_cloud_;

  bool LoadConfig(const std::string& config);

};

} // namespace cad_image_markup