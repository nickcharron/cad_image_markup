#pragma once

#include <string>

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
  void LoadConfig(const std::string& config);

  void LoadImage(const std::string& image_path);
};

} // namespace cad_image_markup