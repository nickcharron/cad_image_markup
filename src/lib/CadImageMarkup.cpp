#include <cad_image_markup/CadImageMarkup.h>

#include <boost/filesystem.hpp>

namespace cad_image_markup {

bool CadImageMarkup::Params::LoadFromJson(const std::string& path) {
  LOG_INFO("Loading config file from: %s", path.c_str());
  // TODO CAM: add function for loading json config settings. Also add a
  // ConfigDefault.json in the config folder of this repo
  return true;
}

CadImageMarkup::CadImageMarkup(const Inputs& inputs) : inputs_(inputs) {}

void CadImageMarkup::Run() {
  if (!Setup()) {
    return false;
  }

  if (!LoadData()) {
    return false;
  }

  return true;
}

bool Setup() {
  solver_visualizer_ = std::make_unique<Visualizer>("solution visualizer");
  input_camera_points_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  input_cad_points_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  if (!config_path.empty()) {
    if (!boost::filesystem::exists(inputs_.config_path)) {
      LOG_ERROR("Invalid path to config file: %s", inputs_.config_path.c_str());
      return false;
    }
    if (!params_.LoadFromJson(inputs_.config_path)) {
      return false;
    }
  }
}

bool CadImageMarkup::LoadData() {
  // read image points
  if (!image_buffer_.ReadPoints(inputs_.image_path, input_camera_points_)) {
    LOG_ERROR("Cannot read image file at: %s", inputs_.image_path.c_str());
    return false;
  }

  // read cad model points
  if (!image_buffer_.ReadPoints(inputs_.cad_path, input_cad_points_)) {
    LOG_ERROR("Cannot read CAD file at: %s", inputs_.cad_path.c_str());
    return false;
  }

  // densify points
  image_buffer_.DensifyPoints(input_camera_points_, params_.);
  image_buffer_.DensifyPoints(input_camera_points_);
}

}  // namespace cad_image_markup