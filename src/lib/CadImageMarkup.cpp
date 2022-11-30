#include <cad_image_markup/CadImageMarkup.h>

#include <boost/filesystem.hpp>

namespace cad_image_markup {

CadImageMarkup::CadImageMarkup(const Inputs& inputs) : inputs_(inputs) {}

bool CadImageMarkup::Run() {

  LOG_INFO("Markup Run Starting");

  if (!Setup()) {
    return false;
  }

  LOG_INFO("Markup Setup Complete");

  // todo here:
  if (!LoadData()) {
    return false;
  }

  LOG_INFO("Markup Load Data Complete");

  if (!Solve()) {
    return false;
  }

  LOG_INFO("Markup Solution Complete");

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

  // TODO: fix redundancy here, utils used to have a common camera model attribute for all fcns when it was a class, should 
  // just pass into every relevant fcn
  std::shared_ptr<CameraModel> camera_model =
      CameraModel::Create(inputs_.intrinsics_path);
  utils::ReadCameraModel(inputs_.intrinsics_path);

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
  LOG_INFO("Camera data loaded successfully");


  LOG_INFO("Densifying points");
  image_buffer_.DensifyPoints(camera_points_CAMFRAME_,
                              params_.cam_density_index);

  LOG_INFO("Loading CAD model data");
  // read cad model points
  if (!image_buffer_.ReadPoints(inputs_.cad_path, cad_points_CADFRAME_)) {
    LOG_ERROR("Cannot read CAD file at: %s", inputs_.cad_path.c_str());
    return false;
  }

  LOG_INFO("Densifying points");
  image_buffer_.DensifyPoints(cad_points_CADFRAME_, params_.cad_density_index);
  LOG_INFO("CAD data loaded successfully");

  LOG_INFO("Getting CAD centroid");
  cad_centroid_ = utils::GetCloudCentroid(cad_points_CADFRAME_);
  utils::OriginCloudxy(cad_points_CADFRAME_, cad_centroid_);
  LOG_INFO("Done loading CAD dimension data");

  // attempt to read defect data
  LOG_INFO("Loading defect data");
  if (!image_buffer_.ReadPointsPNG(inputs_.defect_path, defect_points_CAMFRAME_)) {
    LOG_WARN("Cannot read defect file at: %s", inputs_.defect_path.c_str());
  }


  return true;
}

bool CadImageMarkup::Solve() {
  Eigen::Matrix4d T_WORLD_CAMERA_init;
  LoadInitialPose(inputs_.initial_pose_path, T_WORLD_CAMERA_init);


  //cad_points_WORLDFRAME_ = utils::TransformCloud(cad_points_CADFRAME_,T_WORLD_CAMERA_init);
  // these two frames are effectively coincident for the rest of the solution
  cad_points_WORLDFRAME_ = cad_points_CADFRAME_;


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

  // Generate output 

  return true;
}

//////////////////////////////////////////////////////////////////////
// TODO CAM: Double check this

void CadImageMarkup::LoadInitialPose(const std::string& path,
                                     Eigen::Matrix4d& T_WORLD_CAMERA) {

  if (path.empty()) {
    LOG_INFO(
        "No initial pose path provided. Assuming the image was collected about "
        "3 m from the structure, and taken perpendicularly.");
    T_WORLD_CAMERA = Eigen::Matrix4d::Identity();
    T_WORLD_CAMERA(2,3) = 7; // cad model is assumed to be 3 m ahead of camera in z
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