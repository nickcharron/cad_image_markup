#include <cad_image_markup/CadImageMarkup.h>

#include <boost/filesystem.hpp>

namespace cad_image_markup {

CadImageMarkup::CadImageMarkup(const Inputs& inputs) : inputs_(inputs) {}

bool CadImageMarkup::Run() {
  if (!Setup()) {
    return false;
  }

  // todo here:
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

  if (!params_.LoadFromJson(inputs_.config_path)) {
    LOG_ERROR("Could not load params. Exiting ...");
    return false;
  }

  std::shared_ptr<CameraModel> camera_model =
      CameraModel::Create(inputs_.intrinsics_path);

  solver_ = std::make_unique<Solver>(camera_model, params_,
                                     inputs_.ceres_config_path);
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
  image_buffer_.DensifyPoints(camera_points_CAMFRAME_,
                              params_.cam_density_index);
  image_buffer_.DensifyPoints(cad_points_CADFRAME_, params_.cad_density_index);

  // TODO CAM: I don't understand why we'd need to do this?
  // Based on our convo: we want to calculate T_WORLD_CAD where the world frame
  // is the centroid of the object, and the cad frame is the top left corner. To
  // calculate this, just get the translation in x and y to the centroid. Then I
  // think we can remove this function

  // CAM NOTE: This is just how I am doing that, the T_WORLD_CAMERA needs to
  // operate initially on the cad cloud with its centroid aligned with the
  // camera, when the cad cloud and back-projected defects are flattened,
  // shifting back by the origin coodinates puts everything back in CAD frame
  cad_centroid_ = utils::GetCloudCentroid(cad_points_CADFRAME_);
  utils::OriginCloudxy(cad_points_CADFRAME_, cad_centroid_);

  return true;
}

bool CadImageMarkup::Solve() {
  Eigen::Matrix4d T_WORLD_CAMERA_init;
  utils::LoadInitialPose(inputs_.initial_pose_path, T_WORLD_CAMERA_init);

  bool converged =
      solver_->Solve(cad_points_WORLDFRAME_, camera_points_CAMFRAME_,
                     T_WORLD_CAMERA_init, params_.visualize);

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
  Eigen::Matrix4d T_WORLD_CAMERA = solver_->GetT_WORLD_CAMERA();
  std::cout << "T_WORLD_CAMERA: \n" << T_WORLD_CAMERA << "\n";
}

}  // namespace cad_image_markup