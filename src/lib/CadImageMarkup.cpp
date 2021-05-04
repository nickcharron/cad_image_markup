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

  if (!Solve()) {
    return false;
  }

  return true;
}

bool CadImageMarkup::Setup() {
  solver_visualizer_ = std::make_shared<Visualizer>("solution visualizer");
  input_camera_points_ = std::make_shared<PointCloud>();
  input_cad_points_ = std::make_shared<PointCloud>();
  solver_ =
      std::make_unique<Solver>(solver_visualizer_, inputs_.ceres_config_path);

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
  image_buffer_.DensifyPoints(input_camera_points_, params_.cam_density_index);
  image_buffer_.DensifyPoints(input_cad_points_, params_.cad_density_index);

  // TODO CAM: I don't understand why we'd need to do this?
  utils::OriginCloudxy(input_cad_points_);

  return true;
}

bool CadImageMarkup::Solve() {
  // TODO CAM: we should only need the pose from the camera to world, or camera
  // to structure. The world frame is just the cad image coordinate frame (top
  // left corner). If you're worried about that being different for each CAD
  // model and hard for an inspector to know, then we can set the world frame to
  // the centroid of the structural element (e.g., center of column) in which
  // case we just calculate that ourselves and store it. So optionally, we can
  // feed in a T that we calculate above.
  bool converged = solver_.Solve(input_cloud_CAD, input_cloud_camera,
                                 inputs_.initial_pose_path);

  if (!converged) {
    LOG_ERROR("Solver failed, exiting.");
    return false;
  }
  LOG_INFO("Solver successful.");

  // TODO CAM: output results here? We can just create a new CAD image with the
  // markups, and rename it using the orginal name. We should also output other
  // things like:
  // * T_WORLD_CAMERA_final
  // * T_WORLD_CAMERA_initial
  // * Intrinsics used
  // * config copies (so we can go back and see the results that produced that)
  // Overall structure would be:
  // path_to_cad_in/cad_name.json
  //               /cad_name_results/
  //                                T_WORLD_CAMERA_final.json
  //                                T_WORLD_CAMERA_initial.json
  //                                intrinsics.json
  //                                config.json
  //                                ceres_config.json
  Eigen::Matrix4d T_WORLD_CAMERA = solver.GetT_WORLD_CAMERA();
  std::cout << "T_WORLD_CAMERA: \n" << T_WORLD_CAMERA << "\n";
}

}  // namespace cad_image_markup