#pragma once

#include <string>

namespace cad_image_markup {

class MarkupVisualizer {
public:
  /**
   * @brief constructor
   */
  MarkupVisualizer(const std::string& image_path, const std::string& image_labels);

  /**
   * @brief default deconstructor
   */
  ~MarkupVisualizer() = default;

  void Visualize();

  void Save(const std::string& output_image);

private:
  void LoadImageLabels(const std::string& image_labels);

  void LoadImage(const std::string& image_path);

  void MarkupImage();
};

} // namespace cad_image_markup