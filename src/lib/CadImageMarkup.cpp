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
  LOG_INFO("Setting up problem");
  camera_points_CAMFRAME_ = boost::make_shared<PointCloud>();
  cad_points_CADFRAME_ = boost::make_shared<PointCloud>();

  if (!params_.LoadFromJson(inputs_.config_path)) {
    LOG_ERROR("Could not load params. Exiting ...");
    return false;
  }

  std::shared_ptr<CameraModel> camera_model =
      CameraModel::Create(inputs_.intrinsics_path);

  solver_ = std::make_unique<Solver>(camera_model, params_,
                                     inputs_.ceres_config_path);

  return true;                                   
}

bool CadImageMarkup::LoadData() {
  LOG_INFO("Loading camera data");
  // read image points
  if (!image_buffer_.ReadPoints(inputs_.image_path, camera_points_CAMFRAME_)) {
    LOG_ERROR("Cannot read image file at: %s", inputs_.image_path.c_str());
    return false;
  }

  LOG_INFO("Densifying points");
  image_buffer_.DensifyPoints(camera_points_CAMFRAME_,
                              params_.cam_density_index);
  LOG_INFO("Camera data loaded successfully");

  LOG_INFO("Loading CAD model data");
  // read cad model points
  if (!image_buffer_.ReadPoints(inputs_.cad_path, cad_points_CADFRAME_)) {
    LOG_ERROR("Cannot read CAD file at: %s", inputs_.cad_path.c_str());
    return false;
  }
  LOG_INFO("Densifying points");
  image_buffer_.DensifyPoints(cad_points_CADFRAME_, params_.cad_density_index);
  LOG_INFO("CAD data loaded successfully");

  // TODO CAM: I don't understand why we'd need to do this?
  // Based on our convo: we want to calculate T_WORLD_CAD where the world frame
  // is the centroid of the object, and the cad frame is the top left corner. To
  // calculate this, just get the translation in x and y to the centroid. Then I
  // think we can remove this function

  // CAM NOTE: This is just how I am doing that, the T_WORLD_CAMERA needs to
  // operate initially on the cad cloud with its centroid aligned with the
  // camera, when the cad cloud and back-projected defects are flattened,
  // shifting back by the origin coodinates puts everything back in CAD frame
  LOG_INFO("Getting CAD centroid");
  cad_centroid_ = utils::GetCloudCentroid(cad_points_CADFRAME_);
  utils::OriginCloudxy(cad_points_CADFRAME_, cad_centroid_);
  LOG_INFO("Done loading data");

  return true;
}

bool CadImageMarkup::Solve() {
  Eigen::Matrix4d T_WORLD_CAMERA_init;
  LoadInitialPose(inputs_.initial_pose_path, T_WORLD_CAMERA_init);

  LOG_INFO("Running solver");
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

//////////////////////////////////////////////////////////////////////
// TODO CAM: Double check this

void CadImageMarkup::LoadInitialPose(const std::string& path,
                                     Eigen::Matrix4d& T_WORLD_CAMERA) {
  // TODO CAM: double check this. I assume the coordinate frame of the world
  // frame is centered at the centroid of the CAD model, with the axes aligned
  // with the camera axes. I.e., z pointing from camera to cad model, x - right,
  // y - down
  if (path.empty()) {
    LOG_INFO(
        "No initial pose path provided. Assuming the image was collected about "
        "3 m from the structure, and taken perpendicularly.");
    T_WORLD_CAMERA = Eigen::Matrix4d::Identity();
    T_WORLD_CAMERA(2,3) = -3; // cad model is assumed to be 3 m ahead of camera in z
    return;
  }

  if (!boost::filesystem::exists(path)) {
    LOG_ERROR("Invalid path to initial pose file: %s", path.c_str());
    return;
  }

  LOG_INFO("Loading initial pose file from: %s", path.c_str());

  // load file
  nlohmann::json J;
  std::ifstream file(path);
  file >> J;

  Eigen::Matrix3d R;
  // TODO CAM: check order and put in README
  R = Eigen::AngleAxisd(utils::DegToRad(J["pose"][3]), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(utils::DegToRad(J["pose"][4]), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(utils::DegToRad(J["pose"][5]), Eigen::Vector3d::UnitZ());

  T_WORLD_CAMERA = Eigen::Matrix4d::Identity();
  T_WORLD_CAMERA.block(0, 0, 3, 3) = R;
  T_WORLD_CAMERA(0, 3) = J["pose"][0];
  T_WORLD_CAMERA(1, 3) = J["pose"][1];
  T_WORLD_CAMERA(2, 3) = J["pose"][2];
}

}  // namespace cad_image_markup