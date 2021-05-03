#include <cad_image_markup/CadImageMarkup.h>

namespace cad_image_markup {

CadImageMarkup::CadImageMarkup(const Inputs& inputs) : inputs_(inputs) {}

void CadImageMarkup::Run() {
  bool load_successful = LoadCadModel();
  if (!load_successful) {
    return false;
  }

  return true;
}

void CadImageMarkup::LoadCadModel() {}

}  // namespace cad_image_markup