#include <cad_image_markup/MarkupVisualizer.h>

#include <cad_image_markup/Log.h>

namespace cad_image_markup {

MarkupVisualizer::MarkupVisualizer(const std::string& image_path,
                                       const std::string& image_labels) {
  LoadImageLabels(image_labels);
  LoadImage(image_path);
  MarkupImage();
}

void MarkupVisualizer::LoadImageLabels(const std::string& image_labels) {
  // TODO
}

void MarkupVisualizer::LoadImage(const std::string& image_path) {
  // TODO
}

void MarkupVisualizer::MarkupImage() {
  // TODO
}

void MarkupVisualizer::Visualize(){
  // TODO
}

 void MarkupVisualizer::Save(const std::string& output_image){
  // TODO
 }

} // namespace cad_image_markup