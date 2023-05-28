#include <cad_image_markup/CadImageMarkup.h>

#include <X11/Xlib.h>
#include <boost/filesystem.hpp>

namespace cad_image_markup {

void CadImageMarkup::Inputs::Print() {
  std::cout << "\nCadImageMarkup Inputs:\n"
            << "cad_path: " << cad_path << "\n"
            << "cad_image_path: " << cad_image_path << "\n"
            << "canny_edge_cad_path: " << canny_edge_cad_path << "\n"
            << "image_path: " << image_path << "\n"
            << "canny_edge_image_path: " << canny_edge_image_path << "\n"
            << "defect_path: " << defect_path << "\n"
            << "intrinsics_path: " << intrinsics_path << "\n"
            << "config_path: " << config_path << "\n"
            << "ceres_config_path: " << ceres_config_path << "\n"
            << "initial_pose_path: " << initial_pose_path << "\n"
            << "output_image_path: " << output_image_path << "\n\n";
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
  camera_points_CAMFRAME_raw_ = std::make_shared<PointCloud>();
  camera_points_CAMFRAME_ = std::make_shared<PointCloud>();
  cad_points_CADFRAME_ = std::make_shared<PointCloud>();
  defect_points_CAMFRAME_ = std::make_shared<PointCloud>();

  if (!params_.LoadFromJson(inputs_.config_path)) {
    LOG_ERROR("MARKUP: MARKUP: Could not load params. Exiting ...");
    return false;
  }

  camera_model_ = CameraModel::Create(inputs_.intrinsics_path);
  utils::ReadCameraModel(inputs_.intrinsics_path);

  solver_ = std::make_unique<Solver>(camera_model_, params_,
                                     inputs_.ceres_config_path);

  // Initialize multithreading for visualizer
  XInitThreads();

  return true;
}

bool CadImageMarkup::LoadData() {
  LOG_INFO("MARKUP: Loading camera data");
  // read image points
  if (params_.feature_label_type == "MANUAL") {
    if (!image_buffer_.ReadPoints(inputs_.image_path,
                                  camera_points_CAMFRAME_raw_)) {
      LOG_ERROR("MARKUP: Cannot read image file at: %s",
                inputs_.image_path.c_str());
      return false;
    }

    LOG_INFO("MARKUP: Densifying feature points");
    image_buffer_.DensifyPoints(camera_points_CAMFRAME_raw_,
                                params_.cam_density_index);

  } else if (params_.feature_label_type == "AUTOMATIC") {
    // run Canny edge detection on input image
    if (!image_buffer_.CannyEdgeDetect(
            inputs_.image_path, inputs_.canny_edge_image_path,
            params_.cannny_low_threshold_image, params_.canny_ratio_image,
            params_.canny_kernel_size_image)) {
      LOG_ERROR("MARKUP: Canny Edge Detection Failed");
      return false;
    }
    if (!image_buffer_.ReadPointsPNG(inputs_.canny_edge_image_path,
                                     camera_points_CAMFRAME_raw_, "white",
                                     10)) {
      LOG_WARN("MARKUP: Cannot read canny edge image file at: %s",
               inputs_.canny_edge_image_path.c_str());
    }
  } else {
    LOG_ERROR("MARKUP: Invalid feature_label_type value provided.");
  }

  // Downsample image points if configured
  if (params_.downsample_image_cloud)
    camera_points_CAMFRAME_ = utils::DownSampleCloud(
        camera_points_CAMFRAME_raw_, params_.downsample_grid_size);
  else
    camera_points_CAMFRAME_ = camera_points_CAMFRAME_raw_;

  // read cad model points
  LOG_INFO("MARKUP: Loading CAD model data");

  if (params_.feature_label_type == "MANUAL") {
    if (!image_buffer_.ReadPoints(inputs_.cad_path, cad_points_CADFRAME_)) {
      LOG_ERROR("MARKUP: Cannot read CAD file at: %s",
                inputs_.cad_path.c_str());
      return false;
    }

    LOG_INFO("MARKUP: Densifying cad points");
    image_buffer_.DensifyPoints(cad_points_CADFRAME_,
                                params_.cad_density_index);

  } else if (params_.feature_label_type == "AUTOMATIC") {
    // run Canny edge detection on input cad image
    if (!image_buffer_.CannyEdgeDetect(
            inputs_.cad_image_path, inputs_.canny_edge_cad_path,
            params_.cannny_low_threshold_cad, params_.canny_ratio_cad,
            params_.canny_kernel_size_cad)) {
      LOG_ERROR("MARKUP: Canny Edge Detection Failed");
      return false;
    }

    if (!image_buffer_.ReadPointsPNG(inputs_.canny_edge_cad_path,
                                     cad_points_CADFRAME_, "white", 3)) {
      LOG_WARN("MARKUP: Cannot read canny edge cad file at: %s",
               inputs_.canny_edge_cad_path.c_str());
    }

  } else
    LOG_ERROR("MARKUP: Invalid feature_label_type value provided.");

  LOG_INFO("MARKUP: Adjusting CAD data");
  cad_centroid_ = utils::GetCloudCentroid(cad_points_CADFRAME_);
  utils::OriginCloudxy(cad_points_CADFRAME_, cad_centroid_);
  utils::ScaleCloud(cad_points_CADFRAME_, params_.cad_cloud_scale);
  LOG_INFO("MARKUP: Done loading CAD dimension data");

  // attempt to read defect data
  LOG_INFO("MARKUP: Loading defect data");
  if (!image_buffer_.ReadPointsPNG(inputs_.defect_path, defect_points_CAMFRAME_,
                                   params_.defect_color)) {
    LOG_WARN("MARKUP: Cannot read defect file at: %s",
             inputs_.defect_path.c_str());
  }

  LoadInitialPose(inputs_.initial_pose_path, T_WORLD_CAMERA_init);

  return true;
}

bool CadImageMarkup::Solve() {
  cad_points_WORLDFRAME_ = cad_points_CADFRAME_;

  LOG_INFO("MARKUP: Running solver");
  bool converged =
      solver_->Solve(cad_points_WORLDFRAME_, camera_points_CAMFRAME_,
                     T_WORLD_CAMERA_init, params_.visualize);

  if (!converged) {
    LOG_ERROR("Solver failed, exiting.");
    return false;
  }
  LOG_INFO("MARKUP: Solver successful.");

  Eigen::Matrix4d T_WORLD_CAMERA = solver_->GetT_WORLD_CAMERA();
  std::cout << "T_WORLD_CAMERA: \n" << T_WORLD_CAMERA << "\n";

  // Generate output
  PointCloud::Ptr cad_points_CAMFRAME = std::make_shared<PointCloud>();
  cad_points_CAMFRAME =
      utils::TransformCloud(cad_points_CADFRAME_, T_WORLD_CAMERA);

  pcl::ModelCoefficients::Ptr cad_plane_CAMFRAME =
      utils::GetCloudPlane(cad_points_CAMFRAME);

  LOG_INFO("MARKUP: Got CAD plane in the camera frame");

  PointCloud::Ptr defect_points_CADFRAME = utils::BackProject(
      T_WORLD_CAMERA, defect_points_CAMFRAME_, camera_model_);
  LOG_INFO("MARKUP: Back projected defect points into cad plane");

  Eigen::Matrix4d T_CAMERA_WORLD = utils::InvertTransformMatrix(T_WORLD_CAMERA);
  utils::TransformCloudUpdate(defect_points_CADFRAME, T_CAMERA_WORLD);

  // [NOTE] Add an offset here if the target drawing was cropped from the
  // labelled drawing
  pcl::PointXYZ centroid_offset(-cad_centroid_.x, -cad_centroid_.y, 0);
  centroid_offset.x -= params_.cad_crop_offset_x;
  centroid_offset.y -= params_.cad_crop_offset_y;

  utils::TransformCloudUpdate(cad_points_CAMFRAME, T_CAMERA_WORLD);
  utils::ScaleCloud(cad_points_CAMFRAME, 1.0 / params_.cad_cloud_scale);
  utils::OriginCloudxy(cad_points_CAMFRAME, centroid_offset);

  // Scale defect points before writing to CAD drawing
  utils::ScaleCloud(defect_points_CADFRAME, 1.0 / params_.cad_cloud_scale);
  utils::OriginCloudxy(defect_points_CADFRAME, centroid_offset);

  image_buffer_.WriteToImage(defect_points_CADFRAME, inputs_.cad_image_path,
                             inputs_.output_image_path, 255, 0, 0);

  image_buffer_.WriteToImage(cad_points_CAMFRAME, inputs_.output_image_path,
                             inputs_.output_image_path, 50, 200, 100);

  return true;
}

// [NOTE]: pose calculated with respect to camera focal point with Z pointing
// perpendicularly out from the frame and x any y along image dimensions
// pose vector is trans_x, trans_y, trans_z, rot_x, rot_y, rot_z
void CadImageMarkup::LoadInitialPose(const std::string& path,
                                     Eigen::Matrix4d& T_WORLD_CAMERA) {
  if (path.empty()) {
    LOG_INFO("MARKUP: No initial pose path provided. Assuming the image was "
             "collected about "
             "3 m from the structure, and taken perpendicularly.");
    T_WORLD_CAMERA = Eigen::Matrix4d::Identity();
    T_WORLD_CAMERA(2, 3) =
        3; // cad model is assumed to be 3 m ahead of camera in z by default
    return;
  }

  if (!boost::filesystem::exists(path)) {
    LOG_ERROR("MARKUP: Invalid path to initial pose file: %s", path.c_str());
    return;
  }

  LOG_INFO("MARKUP: Loading initial pose file from: %s", path.c_str());

  // load file
  nlohmann::json J;
  std::ifstream file(path);
  file >> J;

  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(utils::DegToRad(J["pose"][3]),
                        Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(utils::DegToRad(J["pose"][4]),
                        Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(utils::DegToRad(J["pose"][5]),
                        Eigen::Vector3d::UnitZ());

  T_WORLD_CAMERA = Eigen::Matrix4d::Identity();
  T_WORLD_CAMERA.block(0, 0, 3, 3) = R;
  T_WORLD_CAMERA(0, 3) = J["pose"][0];
  T_WORLD_CAMERA(1, 3) = J["pose"][1];
  T_WORLD_CAMERA(2, 3) = J["pose"][2];
}

void CadImageMarkup::SetInitialPose(Eigen::Matrix4d& initial_pose) {
  T_WORLD_CAMERA_init = initial_pose;
}

} // namespace cad_image_markup