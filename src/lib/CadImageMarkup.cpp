#include <cad_image_markup/CadImageMarkup.h>

#include <boost/filesystem.hpp>

namespace cad_image_markup {

CadImageMarkup::CadImageMarkup(const Inputs& inputs) : inputs_(inputs) {}

bool CadImageMarkup::Run() {

  LOG_INFO("MARKUP:  Run Starting");

  if (!Setup()) {
    return false;
  }

  LOG_INFO("MARKUP: Setup Complete");

  if (!LoadData()) {
    return false;
  }

  LOG_INFO("MARKUP: Load Data Complete");

  if (!Solve()) {
    return false;
  }

  LOG_INFO("MARKUP: Solution Complete");

  return true;
}

bool CadImageMarkup::Setup() {
  LOG_INFO("Setting up problem");
  camera_points_CAMFRAME_ = boost::make_shared<PointCloud>();
  cad_points_CADFRAME_ = boost::make_shared<PointCloud>();
  defect_points_CAMFRAME_ = boost::make_shared<PointCloud>();

  if (!params_.LoadFromJson(inputs_.config_path)) {
    LOG_ERROR("Could not load params. Exiting ...");
    return false;
  }

  camera_model_ = CameraModel::Create(inputs_.intrinsics_path);
  utils::ReadCameraModel(inputs_.intrinsics_path);

  solver_ = std::make_unique<Solver>(camera_model_, params_,
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

  
  // read cad model points
  LOG_INFO("Loading CAD model data");
  if (!image_buffer_.ReadPoints(inputs_.cad_path, cad_points_CADFRAME_)) {
    LOG_ERROR("Cannot read CAD file at: %s", inputs_.cad_path.c_str());
    return false;
  }

  LOG_INFO("Densifying points");
  image_buffer_.DensifyPoints(cad_points_CADFRAME_, params_.cad_density_index);
  LOG_INFO("CAD data loaded successfully");

  LOG_INFO("Adjusting CAD data");
  cad_centroid_ = utils::GetCloudCentroid(cad_points_CADFRAME_);
  utils::OriginCloudxy(cad_points_CADFRAME_, cad_centroid_);
  utils::ScaleCloud(cad_points_CADFRAME_,params_.cad_cloud_scale);
  LOG_INFO("Done loading CAD dimension data");

  // attempt to read defect data
  LOG_INFO("Loading defect data");
  if (!image_buffer_.ReadPointsPNG(inputs_.defect_path, defect_points_CAMFRAME_,params_.defect_color)) {
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

  // STILL [TODO]!
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
  PointCloud::Ptr cad_points_CAMFRAME = boost::make_shared<PointCloud>();
  cad_points_CAMFRAME = utils::TransformCloud(cad_points_CADFRAME_, T_WORLD_CAMERA);

  pcl::ModelCoefficients::Ptr cad_plane_CAMFRAME = utils::GetCloudPlane(cad_points_CAMFRAME);

  LOG_INFO("Got CAD plane in the camera frame");

  PointCloud::Ptr defect_points_CADFRAME = utils::BackProject(defect_points_CAMFRAME_, cad_points_CAMFRAME, cad_plane_CAMFRAME, camera_model_);
  LOG_INFO("Back projected defect points into cad plane");

  Eigen::Matrix4d T_CAMERA_WORLD = utils::InvertTransformMatrix(T_WORLD_CAMERA);
  utils::TransformCloudUpdate(defect_points_CADFRAME, T_CAMERA_WORLD);

  // [NOTE] Add an offset here if the target drawing was cropped from the labelled drawing
  pcl::PointXYZ centroid_offset(-cad_centroid_.x, -cad_centroid_.y, 0);
  centroid_offset.x -= params_.cad_crop_offset_x;
  centroid_offset.y -= params_.cad_crop_offset_y;

  utils::TransformCloudUpdate(cad_points_CAMFRAME, T_CAMERA_WORLD);
  utils::ScaleCloud(cad_points_CAMFRAME,1.0/params_.cad_cloud_scale);
  utils::OriginCloudxy(cad_points_CAMFRAME, centroid_offset);
 

  // Scale defect points before writing to CAD drawing
  utils::ScaleCloud(defect_points_CADFRAME,1.0/params_.cad_cloud_scale);
  utils::OriginCloudxy(defect_points_CADFRAME, centroid_offset);

  image_buffer_.WriteToImage(defect_points_CADFRAME,inputs_.cad_image_path, 
                                                    inputs_.output_image_path, 
                                                    255, 0, 0);

  image_buffer_.WriteToImage(cad_points_CAMFRAME,inputs_.output_image_path, 
                                                 inputs_.output_image_path, 
                                                   50, 200, 100);


  return true;
}


// [NOTE]: pose calculated with respect to camera focal point with Z pointing 
// perpendicularly out from the frame and x any y along image dimensions 
void CadImageMarkup::LoadInitialPose(const std::string& path,
                                     Eigen::Matrix4d& T_WORLD_CAMERA) {

  if (path.empty()) {
    LOG_INFO(
        "No initial pose path provided. Assuming the image was collected about "
        "3 m from the structure, and taken perpendicularly.");
    T_WORLD_CAMERA = Eigen::Matrix4d::Identity();
    T_WORLD_CAMERA(2,3) = 3; // cad model is assumed to be 3 m ahead of camera in z
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