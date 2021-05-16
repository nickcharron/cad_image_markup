#include <cad_image_markup/CadImageMarkup.h>

#include <boost/filesystem.hpp>


namespace cad_image_markup {

#define LOSS_CONVERGENCE 0 
#define GEO_CONVERGENCE 1

#define DIFF_CONVERGENCE 0
#define ABS_CONVERGENCE 1


bool CadImageMarkup::Params::LoadFromJson(const std::string& path) {
  LOG_INFO("Loading config file from: %s", path.c_str());
  // TODO CAM: add function for loading json config settings. Also add a
  // ConfigDefault.json in the config folder of this repo. Look at examples in
  // libbeam for parsing jsons
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
  
  camera_points_CAMFRAME_ = std::make_shared<PointCloud>();
  cad_points_CADFRAME_ = std::make_shared<PointCloud>();
  solver_ =
      std::make_unique<Solver>(inputs_.ceres_config_path);

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
  if (!image_buffer_.ReadPoints(inputs_.image_path, camera_points_CAMFRAME_)) {
    LOG_ERROR("Cannot read image file at: %s", inputs_.image_path.c_str());
    return false;
  }

  // read cad model points
  if (!image_buffer_.ReadPoints(inputs_.cad_path, cad_points_CADFRAME_)) {
    LOG_ERROR("Cannot read CAD file at: %s", inputs_.cad_path.c_str());
    return false;
  }

  // densify points
  image_buffer_.DensifyPoints(camera_points_CAMFRAME_, params_.cam_density_index);
  image_buffer_.DensifyPoints(cad_points_CADFRAME_, params_.cad_density_index);

  // TODO CAM: I don't understand why we'd need to do this?
  // Based on our convo: we want to calculate T_WORLD_CAD where the world frame
  // is the centroid of the object, and the cad frame is the top left corner. To
  // calculate this, just get the translation in x and y to the centroid. Then I
  // think we can remove this function
  utils::OriginCloudxy(cad_points_CADFRAME_);
  // Code would look something like this:
  // Eigen::Vector2d centroid = utils::GetCentroid(cad_points_CADFRAME_);
  // T_WORLD_CAD = ...
  // cad_points_WORLDFRAME_ = std::make_shared<PointCloud>();->move to
  // Setup
  // pcl::transformPointCloud(*cad_points_CADFRAME_,*cad_points_WORLDFRAME_,
  // T_WORLD_CAD);

  return true;
}

bool CadImageMarkup::Solve() {
  Eigen::Matrix4d T_WORLD_CAMERA_init =
      utils::LoadInitialPose(inputs_.initial_pose_path);

  // TODO CAM: we should only need the pose from the camera to world, or camera
  // to structure. The world frame is just the cad image coordinate frame (top
  // left corner). If you're worried about that being different for each CAD
  // model and hard for an inspector to know, then we can set the world frame to
  // the centroid of the structural element (e.g., center of column) in which
  // case we just calculate that ourselves and store it. So optionally, we can
  // feed in a T that we calculate above.
  bool converged = solver_.Solve(cad_points_WORLDFRAME_,
                                 camera_points_CAMFRAME_, T_WORLD_CAMERA_init);

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