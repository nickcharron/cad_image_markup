#include <cad_image_markup/Utils.h>

namespace cad_image_markup {

std::shared_ptr<beam_calibration::CameraModel> Util::GetCameraModel() {
  return camera_model_;
}

void Util::ReadCameraModel(std::string intrinsics_file_path) {
  camera_model_ = beam_calibration::CameraModel::Create(intrinsics_file_path);
}

void Util::SetCameraID(uint8_t cam_ID) { camera_model_->SetCameraID(cam_ID); }

void Util::OffsetCloudxy(PointCloud::Ptr cloud) {
  if (!center_image_called_) {
    printf(
        "FAILED to restore image offset - originCloudxy not previously called "
        "by this utility");
    return;
  }

  // restore offset to all points
  for (uint16_t point_index = 0; point_index < cloud->size(); point_index++) {
    cloud->at(point_index).x += static_cast<int>(image_offset_x_);
    cloud->at(point_index).y += static_cast<int>(image_offset_y_);
  }
}

void Util::OriginCloudxy(PointCloud::Ptr cloud) {
  uint16_t num_points = cloud->size();

  // determine central x and y values
  float max_x = 0, max_y = 0, min_x = 2048, min_y = 2048;
  for (uint16_t point_index = 0; point_index < num_points; point_index++) {
    if (cloud->at(point_index).x > max_x) max_x = cloud->at(point_index).x;
    if (cloud->at(point_index).y > max_y) max_y = cloud->at(point_index).y;

    if (cloud->at(point_index).x < min_x) min_x = cloud->at(point_index).x;
    if (cloud->at(point_index).y < min_y) min_y = cloud->at(point_index).y;
  }

  float center_x = min_x + (max_x - min_x) / 2;
  float center_y = min_y + (max_y - min_y) / 2;

  image_offset_x_ = center_x;
  image_offset_y_ = center_y;

  // shift all points back to center on origin
  for (uint16_t point_index = 0; point_index < num_points; point_index++) {
    cloud->at(point_index).x -= (int)center_x;
    cloud->at(point_index).y -= (int)center_y;
  }

  center_image_called_ = true;
}

PointCloud::Ptr Util::ProjectCloud(PointCloud::Ptr cloud) {
  PointCloud::Ptr proj_cloud = std::make_shared<PointCloud>();
  for (uint16_t i = 0; i < cloud->size(); i++) {
    Eigen::Vector3d point(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
    std::optional<Eigen::Vector2d> pixel_projected;
    pixel_projected = camera_model_->ProjectPointPrecise(point);
    if (pixel_projected.has_value()) {
      pcl::PointXYZ proj_point(pixel_projected.value()(0),
                               pixel_projected.value()(1), 0);
      proj_cloud->push_back(proj_point);
    }
  }
  return proj_cloud;
}

void Util::CorrEst(PointCloud::ConstPtr CAD_cloud,
                   PointCloud::ConstPtr camera_cloud, Eigen::Matrix4d& T,
                   pcl::CorrespondencesPtr corrs, bool align_centroids, double max_corr_distance) {
  PointCloud::Ptr proj_cloud = std::make_shared<PointCloud>();
  PointCloud::Ptr trans_cloud = std::make_shared<PointCloud>();

  // transform the CAD cloud points to the camera frame
  trans_cloud = this->TransformCloud(CAD_cloud, T);

  // project the transformed points to the camera plane
  proj_cloud = this->ProjectCloud(trans_cloud);

  // merge centroids for correspondence estimation (projected -> camera)
  if (align_centroids) {
    pcl::PointXYZ camera_centroid = Util::GetCloudCentroid(camera_cloud);
    pcl::PointXYZ proj_centroid = Util::GetCloudCentroid(proj_cloud);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = camera_centroid.x - proj_centroid.x;
    T(1, 3) = camera_centroid.y - proj_centroid.y;
    T(2, 3) = camera_centroid.z - proj_centroid.z;
    pcl::transformPointCloud(*proj_cloud, proj_cloud, T);
  }

  // get correspondences
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>
      corr_est;
  corr_est.setInputSource(camera_cloud);
  corr_est.setInputTarget(proj_cloud);
  corr_est.determineCorrespondences(*corrs, max_corr_distance);
}

namespace utils {

Eigen::Matrix4d Util::QuaternionAndTranslationToTransformMatrix(
    const std::vector<double>& pose) {
  Eigen::Quaterniond q{pose[0], pose[1], pose[2], pose[3]};
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block(0, 0, 3, 3) = q.toRotationMatrix();
  T(0, 3) = pose_[4];
  T(1, 3) = pose_[5];
  T(2, 3) = pose_[6];
  return T;
}

void GetCloudScale(PointCloud::ConstPtr cloud, double max_x_dim,
                   double max_y_dim, float& x_scale, float& y_scale) {
  // get max cloud dimensions in x and y
  float max_x = 0;
  float max_y = 0;
  float min_x = cloud->at(0).x;
  float min_y = cloud->at(0).y;
  for (uint16_t point_index = 0; point_index < cloud->size(); point_index++) {
    if (cloud->at(point_index).x > max_x) max_x = cloud->at(point_index).x;
    if (cloud->at(point_index).y > max_y) max_y = cloud->at(point_index).y;
    if (cloud->at(point_index).x < min_x) min_x = cloud->at(point_index).x;
    if (cloud->at(point_index).y < min_y) min_y = cloud->at(point_index).y;
  }

  // CAD unit/pixel
  x_scale = max_x_dim / (max_x - min_x);
  y_scale = max_y_dim / (max_y - min_y);
}

void ScaleCloud(PointCloud::Ptr cloud, float scale) {
  for (uint16_t i = 0; i < cloud->size(); i++) {
    cloud->at(i).x *= scale;
    cloud->at(i).y *= scale;
    cloud->at(i).z *= scale;
  }
}

PointCloud::Ptr ScaleCloud(PointCloud::ConstPtr cloud, float scale) {
  PointCloud::Ptr scaled_cloud = std::make_shared<PointCloud>();
  for (uint16_t i = 0; i < cloud->size(); i++) {
    pcl::PointXYZ to_add;
    to_add.x = cloud->at(i).x * scale;
    to_add.y = cloud->at(i).y * scale;
    to_add.z = cloud->at(i).z * scale;
    scaled_cloud->push_back(to_add);
  }
  return scaled_cloud;
}

void ScaleCloud(PointCloud::Ptr cloud, float x_scale, float y_scale) {
  for (uint16_t i = 0; i < cloud->size(); i++) {
    cloud->at(i).x *= x_scale;
    cloud->at(i).y *= y_scale;
  }
}

//////////////////////////////////////////////////////////////////////
// CHANGE THESE FUNCTIONS

void LoadInitialPose(std::string file_name, Eigen::Matrix4d& T_,
                     bool inverted) {
  // load file
  nlohmann::json J;
  std::ifstream file(file_name);
  file >> J;

  // initial pose
  double w_initial_pose[6];  // x, y, z, alpha, beta, gamma
  double c_initial_pose[6];  // x, y, z, alpha, beta, gamma

  w_initial_pose[0] = J["pose"][0];
  w_initial_pose[1] = J["pose"][1];
  w_initial_pose[2] = J["pose"][2];
  w_initial_pose[3] = J["pose"][3];
  w_initial_pose[4] = J["pose"][4];
  w_initial_pose[5] = J["pose"][5];

  // remap translations and rotations
  RemapWorldtoCameraCoords(w_initial_pose, c_initial_pose);

  if (inverted == false) {
    // load forward transform
    Eigen::VectorXd perturbation(6, 1);
    perturbation << -c_initial_pose[3], 0, 0, 0, 0, 0;
    T = PerturbTransformDegM(T, perturbation);
    perturbation << 0, -c_initial_pose[4], 0, 0, 0, 0;
    T = PerturbTransformDegM(T, perturbation);
    perturbation << 0, 0, -c_initial_pose[5], 0, 0, 0;
    T = PerturbTransformDegM(T, perturbation);
    perturbation << 0, 0, 0, -c_initial_pose[0], -c_initial_pose[1],
        -c_initial_pose[2];
    T = PerturbTransformDegM(T, perturbation);
  } else {
    // load inverse transform
    Eigen::VectorXd perturbation(6, 1);
    perturbation << c_initial_pose[3], 0, 0, 0, 0, 0;
    T = PerturbTransformDegM(T, perturbation);
    perturbation << 0, c_initial_pose[4], 0, 0, 0, 0;
    T = PerturbTransformDegM(T, perturbation);
    perturbation << 0, 0, c_initial_pose[5], 0, 0, 0;
    T = PerturbTransformDegM(T, perturbation);
    perturbation << 0, 0, 0, c_initial_pose[0], c_initial_pose[1],
        c_initial_pose[2];
    T = PerturbTransformDegM(T, perturbation);
  }
}

void Util::TransformPose(std::string file_name, Eigen::Matrix4d& T,
                         bool inverted) {
  // load file
  nlohmann::json J;
  std::ifstream file(file_name);
  file >> J;

  // initial pose
  double w_pose[6];  // x, y, z, alpha, beta, gamma
  double c_pose[6];  // x, y, z, alpha, beta, gamma

  w_pose[0] = J["pose"][0];
  w_pose[1] = J["pose"][1];
  w_pose[2] = J["pose"][2];
  w_pose[3] = J["pose"][3];
  w_pose[4] = J["pose"][4];
  w_pose[5] = J["pose"][5];

  // remap translations and rotations
  RemapWorldtoCameraCoords(w_pose, c_pose);

  // construct the matrix describing the transformation
  // from the world to the camera frame
  if (inverted) {
    Eigen::VectorXd perturbation(6, 1);
    perturbation << -c_pose[3], 0, 0, 0, 0, 0;
    T = PerturbTransformDegM(T, perturbation);
    perturbation << 0, -c_pose[4], 0, 0, 0, 0;
    T = PerturbTransformDegM(T, perturbation);
    perturbation << 0, 0, -c_pose[5], 0, 0, 0;
    T = PerturbTransformDegM(T, perturbation);
    perturbation << 0, 0, 0, -c_pose[0], -c_pose[1], -c_pose[2];
    T = PerturbTransformDegM(T, perturbation);
  } else {
    Eigen::VectorXd perturbation(6, 1);
    perturbation << c_pose[3], 0, 0, 0, 0, 0;
    T = PerturbTransformDegM(T, perturbation);
    perturbation << 0, c_pose[4], 0, 0, 0, 0;
    T = PerturbTransformDegM(T, perturbation);
    perturbation << 0, 0, c_pose[5], 0, 0, 0;
    T = PerturbTransformDegM(T, perturbation);
    perturbation << 0, 0, 0, c_pose[0], c_pose[1], c_pose[2];
    T = PerturbTransformDegM(T, perturbation);
  }
}

void Util::RemapWorldtoCameraCoords(const double (&world_transform_)[6],
                                    double (&camera_transform_)[6]) {
  // just a rotation
  camera_transform_[0] = -world_transform_[1];  // y -> -x
  camera_transform_[1] = -world_transform_[2];  // z -> -y
  camera_transform_[2] = world_transform_[0];   // x -> z
  camera_transform_[4] = world_transform_[5];   // beta -> alpha
  camera_transform_[5] = -world_transform_[6];  // gamma ->
  camera_transform_[6] = world_transform_[4];   // alpha -> gamma
}
//////////////////////////////////////////////////////////////////////

pcl::ModelCoefficients::Ptr GetCloudPlane(PointCloud::ConstPtr cloud) {
  pcl::ModelCoefficients::Ptr coefficients =
      std::make_shared<pcl::ModelCoefficients>();
  pcl::PointIndices::Ptr inliers = std::make_shared<pcl::PointIndice>();

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  return coefficients;
}

PointCloud::Ptr BackProject(
    PointCloud::ConstPtr image_cloud, PointCloud::ConstPtr cad_cloud,
    pcl::ModelCoefficients::ConstPtr target_plane,
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model) {
  PointCloud::Ptr back_projected_cloud = std::make_shared<PointCloud>();

  // get cad surface normal and point on the cad plane
  Eigen::Vector3d cad_normal(target_plane->values[0], target_plane->values[1],
                             target_plane->values[2]);
  Eigen::Vector3d cad_point(cad_cloud->at(0).x, cad_cloud->at(0).y,
                            cad_cloud->at(0).z);

  for (uint32_t i = 0; i < image_cloud->size(); i++) {
    Eigen::Vector3d image_point(0, 0, 0);
    Eigen::Vector2i image_pixel(image_cloud->at(i).x, image_cloud->at(i).y);
    Eigen::Vector3d ray_unit_vector =
        camera_model->BackProject(image_pixel).value().normalized();
    double prod1 = (image_point - cad_point).dot(cad_normal);

    double len = prod1 / (ray_unit_vector.dot(cad_normal));

    Eigen::Vector3d back_projected_point = image_point - ray_unit_vector * len;

    pcl::PointXYZ back_projected_cloud_point(back_projected_point[0],
                                             back_projected_point[1],
                                             back_projected_point[2]);

    back_projected_cloud->push_back(back_projected_cloud_point);
  }

  return back_projected_cloud;
}

pcl::PointXYZ GetCloudCentroid(PointCloud::ConstPtr cloud) {
  pcl::PointXYZ centroid;
  pcl::computeCentroid(*cloud, centroid);
  centroid.z = 0;
  return centroid;
}

void GetCorrespondences(pcl::CorrespondencesPtr corrs,
                        PointCloud::ConstPtr source_coud,
                        PointCloud::ConstPtr target_cloud, uint16_t max_dist_) {
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>
      corr_est;
  corr_est.setInputSource(source_coud_);
  corr_est.setInputTarget(target_cloud_);
  corr_est.determineCorrespondences(*corrs, max_dist);
}

Eigen::Matrix4d PerturbTransformRadM(const Eigen::Matrix4d& T,
                                     const Eigen::VectorXd& perturbation) {
  Eigen::Vector3d r_perturb = perturbation.block(0, 0, 3, 1);
  Eigen::Vector3d t_perturb = perturbation.block(3, 0, 3, 1);
  Eigen::Matrix3d R_in = T.block(0, 0, 3, 3);
  Eigen::Matrix3d R_out = LieAlgebraToR(r_perturb) * R_in;
  Eigen::Matrix4d T_out;
  T_out.setIdentity();
  T_out.block(0, 3, 3, 1) = T.block(0, 3, 3, 1) + t_perturb;
  T_out.block(0, 0, 3, 3) = R_out;
  return T_out;
}

Eigen::Matrix4d PerturbTransformDegM(const Eigen::Matrix4d& T,
                                     const Eigen::VectorXd& perturbation) {
  Eigen::VectorXd perturbation_rad(perturbation);
  perturbation_rad[0] = DegToRad(perturbation_rad[0]);
  perturbation_rad[1] = DegToRad(perturbation_rad[1]);
  perturbation_rad[2] = DegToRad(perturbation_rad[2]);
  return PerturbTransformRadM(T, perturbation_rad);
}

Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d& eps) {
  return SkewTransform(eps).exp();
}

Eigen::Matrix3d SkewTransform(const Eigen::Vector3d& V) {
  Eigen::Matrix3d M;
  M(0, 0) = 0;
  M(0, 1) = -V(2, 0);
  M(0, 2) = V(1, 0);
  M(1, 0) = V(2, 0);
  M(1, 1) = 0;
  M(1, 2) = -V(0, 0);
  M(2, 0) = -V(1, 0);
  M(2, 1) = V(0, 0);
  M(2, 2) = 0;
  return M;
}

double DegToRad(double d) { return d * (M_PI / 180); }

}  // namespace utils

}  // namespace cad_image_markup