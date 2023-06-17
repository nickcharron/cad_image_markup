#include <cad_image_markup/CadImageMarkup.h>

#include <X11/Xlib.h>
#include <filesystem>

namespace cad_image_markup {

void CadImageMarkup::Inputs::Print() {
  std::cout << "\nCadImageMarkup Inputs:\n"
            << "image_label_path: " << image_label_path << "\n"
            << "image_path: " << image_path << "\n"
            << "cad_label_path: " << cad_label_path << "\n"
            << "cad_image_path: " << cad_image_path << "\n"
            << "defect_path: " << defect_path << "\n"
            << "intrinsics_path: " << intrinsics_path << "\n"
            << "config_path: " << solution_config_path << "\n"
            << "ceres_config_path: " << ceres_config_path << "\n"
            << "initial_pose_path: " << initial_pose_path << "\n\n";
}

CadImageMarkup::CadImageMarkup(const Inputs& inputs) : inputs_(inputs) {}

bool CadImageMarkup::Run() {
  LOG_INFO("MARKUP:  Run Starting");

  if (!Setup()) { return false; }

  LOG_INFO("MARKUP: Setup Complete");

  inputs_.Print();

  if (!LoadData()) { return false; }

  LOG_INFO("MARKUP: Load Data Complete");

  if (!Solve()) { return false; }

  LOG_INFO("MARKUP: Solution Complete");

  return true;
}

bool CadImageMarkup::Setup() {
  LOG_INFO("MARKUP: Setting up problem");
  camera_points_CAMFRAME_ = std::make_shared<PointCloud>();
  cad_points_CADFRAME_ = std::make_shared<PointCloud>();
  defect_points_CAMFRAME_ = std::make_shared<PointCloud>();

  if (!params_.LoadSolutionParamsFromJSON(inputs_.solution_config_path)) {
    LOG_ERROR("MARKUP: MARKUP: Could not load solution params. Exiting ...");
    return false;
  }

  camera_model_ = CameraModel::Create(inputs_.intrinsics_path);

  solver_ = std::make_unique<Solver>(camera_model_, params_,
                                     inputs_.ceres_config_path);

  // Initialize multithreading for visualizer
  XInitThreads();

  return true;
}

bool CadImageMarkup::LoadData() {
  LOG_INFO("MARKUP: Loading camera data");
  // read image points
  if (!utils::ReadPoints(inputs_.image_label_path,
                                camera_points_CAMFRAME_)) {
    LOG_ERROR("MARKUP: Cannot read image file at: %s",
              inputs_.image_label_path.c_str());
    return false;
  }

  if(camera_points_CAMFRAME_->size() < params_.min_num_points_to_densify) {
    LOG_INFO("MARKUP: Densifying feature points");
    utils::DensifyPoints(camera_points_CAMFRAME_,
                              params_.cam_density_index);
  }

  // read cad model points
  LOG_INFO("MARKUP: Loading CAD model data");

  if (!utils::ReadPoints(inputs_.cad_label_path, cad_points_CADFRAME_)) {
    LOG_ERROR("MARKUP: Cannot read CAD file at: %s",
              inputs_.cad_label_path.c_str());
    return false;
  }

  if(cad_points_CADFRAME_->size() < params_.min_num_points_to_densify) {
    LOG_INFO("MARKUP: Densifying cad points");
    utils::DensifyPoints(cad_points_CADFRAME_,
                                params_.cad_density_index);
  }

  LOG_INFO("MARKUP: Adjusting CAD data");
  cad_centroid_ = utils::GetCloudCentroid(cad_points_CADFRAME_);
  utils::OriginCloudxy(cad_points_CADFRAME_, cad_centroid_);
  utils::ScaleCloud(cad_points_CADFRAME_, params_.cad_cloud_scale);

  LOG_INFO("MARKUP: Done loading CAD dimension data");

  // attempt to read defect data
  LOG_INFO("MARKUP: Loading defect data");

  if (!utils::ReadPointsPNG(inputs_.defect_path, defect_points_CAMFRAME_,
                                   params_.defect_color)) {
    LOG_WARN("MARKUP: Cannot read defect file at: %s",
             inputs_.defect_path.c_str());
  }

  return LoadInitialPose();
}

bool CadImageMarkup::Solve() {
  cad_points_WORLDFRAME_ = cad_points_CADFRAME_;

  LOG_INFO("MARKUP: Running solver");
  solution_converged_ =
      solver_->Solve(cad_points_WORLDFRAME_, camera_points_CAMFRAME_,
                     T_WORLD_CAMERA_init_, params_.visualize);

  if (!solution_converged_) {
    LOG_ERROR("Solver failed, exiting.");
    return false;
  }
  LOG_INFO("MARKUP: Solver successful.");
  LOG_INFO("MARKUP: Final T_WORLD_CAMERA:");
  std::cout << solver_->GetT_WORLD_CAMERA() << std::endl;
  return true;
}

Eigen::Matrix4d CadImageMarkup::GetFinalT_World_Camera() const {
  if (!solution_converged_) {
    LOG_ERROR("No valid solution available, cannot return T_World_Camera.");
    return {};
  }

  return solver_->GetT_WORLD_CAMERA();
}

bool CadImageMarkup::SaveResults(const std::string& output_directory) const {
  Eigen::Matrix4d T_WORLD_CAMERA = solver_->GetT_WORLD_CAMERA();

  // Generate output
  PointCloud::Ptr cad_points_CAMFRAME = std::make_shared<PointCloud>();
  LOG_INFO("MARKUP: transforming cad points to camera frame...");
  pcl::transformPointCloud(*cad_points_CADFRAME_, *cad_points_CAMFRAME, T_WORLD_CAMERA);

  LOG_INFO("MARKUP: back projecting defect points into cad plane");
  PointCloud::Ptr defect_points_CADFRAME = utils::BackProject(
      T_WORLD_CAMERA, defect_points_CAMFRAME_, camera_model_);

  Eigen::Matrix4d T_CAMERA_WORLD = utils::InvertTransformMatrix(T_WORLD_CAMERA);
  pcl::transformPointCloud(*defect_points_CADFRAME, *defect_points_CADFRAME, T_CAMERA_WORLD);

  // [NOTE] Add an offset here if the target drawing was cropped from the
  // labelled drawing
  pcl::PointXYZ centroid_offset(-cad_centroid_.x, -cad_centroid_.y, 0);
  centroid_offset.x -= 0;
  centroid_offset.y -= 0;

  pcl::transformPointCloud(*cad_points_CAMFRAME, *cad_points_CAMFRAME, T_CAMERA_WORLD);
  utils::ScaleCloud(cad_points_CAMFRAME, 1.0 / params_.cad_cloud_scale);
  utils::OriginCloudxy(cad_points_CAMFRAME, centroid_offset);

  // Scale defect points before writing to CAD drawing
  utils::ScaleCloud(defect_points_CADFRAME, 1.0 / params_.cad_cloud_scale);
  utils::OriginCloudxy(defect_points_CADFRAME, centroid_offset);

  std::string date_and_time = cad_image_markup::utils::ConvertTimeToDate(
      std::chrono::system_clock::now());
  std::filesystem::path output_dir_stamped =
      std::filesystem::path(output_directory) / date_and_time;
  std::filesystem::create_directory(output_dir_stamped);
  std::filesystem::path output_cad =
      output_dir_stamped / std::filesystem::path("cad_with_defects.png");
  std::filesystem::path output_img =
      output_dir_stamped / std::filesystem::path("img_with_edges.png");

  image_buffer_.WriteToImage(defect_points_CADFRAME, inputs_.cad_image_path,
                             output_cad.string(), 255, 0, 0);

  image_buffer_.WriteToImage(cad_points_CAMFRAME, output_cad.string(),
                             output_cad.string(), 0, 255, 255);

  // copy input data for easy comparison
  std::string extension_cad =
      std::filesystem::path(inputs_.cad_image_path).extension();
  std::filesystem::path output_cad_orig =
      output_dir_stamped / std::filesystem::path("cad_original" + extension_cad);
  std::filesystem::copy(inputs_.cad_image_path, output_cad_orig);

  std::string extension_img =
      std::filesystem::path(inputs_.cad_image_path).extension();
  std::filesystem::path output_img_orig =
      output_dir_stamped / std::filesystem::path("img_original" + extension_img);
  std::filesystem::copy(inputs_.image_path, output_img_orig);

  return true;
}

// [NOTE]: pose calculated with respect to camera focal point with Z pointing
// perpendicularly out from the frame and x any y along image dimensions
// pose vector is trans_x, trans_y, trans_z, rot_x, rot_y, rot_z
bool CadImageMarkup::LoadInitialPose() {
  if (inputs_.initial_pose_path.empty()) {
    LOG_INFO("MARKUP: No initial pose path provided. Assuming the image was "
             "collected about "
             "3 m from the structure, and taken perpendicularly.");
    T_WORLD_CAMERA_init_ = Eigen::Matrix4d::Identity();
    T_WORLD_CAMERA_init_(2, 3) =
        3; // cad model is assumed to be 3 m ahead of camera in z by default
    return true;
  }

  if (!std::filesystem::exists(inputs_.initial_pose_path)) {
    LOG_ERROR("MARKUP: Invalid path to initial pose file: %s",
              inputs_.initial_pose_path.c_str());
    return false;
  }
  LOG_INFO("MARKUP: Loading initial pose file from: %s",
           inputs_.initial_pose_path.c_str());

  // load file
  nlohmann::json J;
  std::ifstream file(inputs_.initial_pose_path);
  file >> J;

  if (!J.contains("pose")) {
    LOG_ERROR("Invalid pose file,, missing field: pose");
    return false;
  }

  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(utils::DegToRad(J["pose"][3]),
                        Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(utils::DegToRad(J["pose"][4]),
                        Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(utils::DegToRad(J["pose"][5]),
                        Eigen::Vector3d::UnitZ());

  T_WORLD_CAMERA_init_ = Eigen::Matrix4d::Identity();
  T_WORLD_CAMERA_init_.block(0, 0, 3, 3) = R;
  T_WORLD_CAMERA_init_(0, 3) = J["pose"][0];
  T_WORLD_CAMERA_init_(1, 3) = J["pose"][1];
  T_WORLD_CAMERA_init_(2, 3) = J["pose"][2];
  return true;
}

void CadImageMarkup::SetInitialPose(const Eigen::Matrix4d& T_WORLD_CAM) {
  T_WORLD_CAMERA_init_ = T_WORLD_CAM;
}

} // namespace cad_image_markup