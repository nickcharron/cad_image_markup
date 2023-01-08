#include <cad_image_markup/Utils.h>

namespace cad_image_markup {

namespace utils {

// Provide default definitions for namespace variables
double image_offset_x_ = 0;
double image_offset_y_ = 0;
bool center_image_called_ = false;
std::shared_ptr<cad_image_markup::CameraModel> camera_model_ = nullptr;

std::shared_ptr<cad_image_markup::CameraModel> GetCameraModel() {
  return camera_model_;
}

void ReadCameraModel(std::string intrinsics_file_path) {
  camera_model_ = cad_image_markup::CameraModel::Create(intrinsics_file_path);
}

void SetCameraID(uint8_t cam_ID) { camera_model_->SetCameraID(cam_ID); }

void OffsetCloudxy(PointCloud::Ptr cloud, Eigen::Vector2d offset) {

  // restore offset to all points
  for (uint16_t point_index = 0; point_index < cloud->size(); point_index++) {
    cloud->at(point_index).x += static_cast<int>(offset(0));
    cloud->at(point_index).y += static_cast<int>(offset(1));
  }
}

void OriginCloudxy(PointCloud::Ptr cloud, const pcl::PointXYZ& centroid) {
  uint16_t num_points = cloud->size();

  // shift all points back to center on origin
  for (uint16_t point_index = 0; point_index < num_points; point_index++) {
    cloud->at(point_index).x -= centroid.x;
    cloud->at(point_index).y -= centroid.y;
  }
}

PointCloud::Ptr ProjectCloud(PointCloud::Ptr cloud) {
  PointCloud::Ptr proj_cloud (new PointCloud);

  for (uint16_t i = 0; i < cloud->size(); i++) {
    Eigen::Vector3d point(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
    cad_image_markup::optional<Eigen::Vector2d> pixel_projected;

    pixel_projected = camera_model_->ProjectPointPrecise(point);
    if (pixel_projected.has_value()) {
      pcl::PointXYZ proj_point(pixel_projected.value()(0),
                               pixel_projected.value()(1), 0);
      proj_cloud->push_back(proj_point);
    }
  }

  return proj_cloud;
}

PointCloud::Ptr TransformCloud (
    PointCloud::ConstPtr cloud, const Eigen::Matrix4d &T) {

    PointCloud::Ptr trans_cloud (new PointCloud);
    
    for(uint16_t i=0; i < cloud->size(); i++) {
        Eigen::Vector4d point (cloud->at(i).x, cloud->at(i).y, cloud->at(i).z, 1);
        Eigen::Vector4d point_transformed = T*point; 
        pcl::PointXYZ pcl_point_transformed (point_transformed(0), 
            point_transformed(1), point_transformed(2));
        trans_cloud->push_back(pcl_point_transformed);
    }

    return trans_cloud;

}

void TransformCloudUpdate (PointCloud::Ptr cloud, 
                     const Eigen::Matrix4d &T) {
    
    for(uint16_t i=0; i < cloud->size(); i++) {
        Eigen::Vector4d point (cloud->at(i).x, cloud->at(i).y, 
            cloud->at(i).z, 1);
        Eigen::Vector4d point_transformed = T*point; 
        pcl::PointXYZ pcl_point_transformed (point_transformed(0), 
            point_transformed(1), point_transformed(2));
        cloud->at(i) = pcl_point_transformed;
    }

}

void GetCorrespondences(pcl::CorrespondencesPtr corrs_, 
                              pcl::PointCloud<pcl::PointXYZ>::ConstPtr source_coud_,
                              pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud_,
                              uint16_t max_dist_) {

    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
    corr_est.setInputSource(source_coud_);
    corr_est.setInputTarget(target_cloud_);
    corr_est.determineCorrespondences(*corrs_,max_dist_);

}

void CorrespondenceEstimate(PointCloud::ConstPtr cad_cloud,
                   PointCloud::ConstPtr camera_cloud, const Eigen::Matrix4d& T,
                   pcl::CorrespondencesPtr corrs, bool align_centroids,
                   double max_corr_distance, int num_corrs, std::string source) {

  //clear the previous correspondences 
  corrs->clear();

  // transform the CAD cloud points to the camera frame
  PointCloud::Ptr trans_cloud (new PointCloud);
  trans_cloud = TransformCloud(cad_cloud, T);


  // project the transformed points to the camera plane
  PointCloud::Ptr proj_cloud = ProjectCloud(trans_cloud);

  // merge centroids for correspondence estimation (projected -> camera)
  if (align_centroids) {
    pcl::PointXYZ camera_centroid = GetCloudCentroid(camera_cloud);
    pcl::PointXYZ proj_centroid = GetCloudCentroid(proj_cloud);

    Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
    T1(0, 3) = camera_centroid.x - proj_centroid.x;
    T1(1, 3) = camera_centroid.y - proj_centroid.y;
    T1(2, 3) = camera_centroid.z - proj_centroid.z;
    TransformCloudUpdate(proj_cloud, T1);
  }


  // Get correspondences (source camera)
  if (source == "camera") {
    // Build kd tree from projected cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> proj_kdtree;
    proj_kdtree.setInputCloud (proj_cloud);

    for(uint32_t i = 0; i<camera_cloud->size(); i++) {
      std::vector<int> target_neighbor_indices(num_corrs);
      std::vector<float> target_neighbor_distances(num_corrs);
      pcl::Correspondence corr1;
      pcl::Correspondence corr2;

      if (proj_kdtree.nearestKSearch (camera_cloud->at(i), num_corrs, 
          target_neighbor_indices, target_neighbor_distances) >= num_corrs
          /*&& target_neighbor_distances[0] <= max_corr_distance*/)
      {
        corr1.index_query = i;
        corr1.index_match = target_neighbor_indices[0];

        corrs->push_back(corr1);
        
        if (num_corrs == 2 && target_neighbor_distances[1] <= max_corr_distance)
        {
          corr1.index_query = i;
          corr1.index_match = target_neighbor_indices[1];

          corrs->push_back(corr2);
        }

        else if (num_corrs == 2 && target_neighbor_distances[1] > max_corr_distance) corrs->pop_back();

      }

    }
  }

  // Get correspondences (source CAD)
  if (source == "projected") {
    // Build kd tree from projected cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> camera_kdtree;
    camera_kdtree.setInputCloud (camera_cloud);

    for(uint32_t i = 0; i<proj_cloud->size(); i++) {
      std::vector<int> target_neighbor_indices(num_corrs);
      std::vector<float> target_neighbor_distances(num_corrs);
      pcl::Correspondence corr1;
      pcl::Correspondence corr2;

      if (camera_kdtree.nearestKSearch (proj_cloud->at(i), num_corrs, 
          target_neighbor_indices, target_neighbor_distances) >= num_corrs
          /*&& target_neighbor_distances[0] <= max_corr_distance*/)
      {
        corr1.index_query = i;
        corr1.index_match = target_neighbor_indices[0];

        corrs->push_back(corr1);
        
        if (num_corrs == 2 && target_neighbor_distances[1] <= max_corr_distance)
        {
          corr1.index_query = i;
          corr1.index_match = target_neighbor_indices[1];

          corrs->push_back(corr2);
        }

        else if (num_corrs == 2 && target_neighbor_distances[1] > max_corr_distance) corrs->pop_back();

      }

    }
  }
  
}

Eigen::Matrix4d QuaternionAndTranslationToTransformMatrix(
    const std::vector<double>& pose) {
  Eigen::Quaterniond q{pose[0], pose[1], pose[2], pose[3]};
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block(0, 0, 3, 3) = q.toRotationMatrix();
  T(0, 3) = pose[4];
  T(1, 3) = pose[5];
  T(2, 3) = pose[6];
  return T;
}

void TransformMatrixToQuaternionAndTranslation(const Eigen::Matrix4d& T,
                                               Eigen::Quaterniond& q,
                                               Eigen::Vector3d& p) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Quaterniond q_tmp(R);
  q = q_tmp;
  p = T.block<3, 1>(0, 3).transpose();
}

Eigen::Matrix4d InvertTransformMatrix(const Eigen::Matrix4d& T) {
  Eigen::Matrix4d inverse_transform = Eigen::Matrix4d::Identity();

  // transpose rotation matrix to get inverse rotation
  inverse_transform.block<3,3>(0,0) = (T.block<3,3>(0,0)).transpose();

  // the translations are:
  inverse_transform.block<3,1>(0,3) = -inverse_transform.block<3,3>(0,0)*T.block<3,1>(0,3);

  return inverse_transform;

}

void GetCloudScale(PointCloud::ConstPtr cloud, double max_x_dim,
                   double max_y_dim, double x_scale, double y_scale) {
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
  x_scale = (max_x_dim / (max_x - min_x));
  y_scale = (max_y_dim / (max_y - min_y));
}

void ScaleCloud(PointCloud::Ptr cloud, float scale) {
  for (uint16_t i = 0; i < cloud->size(); i++) {
    cloud->at(i).x *= scale;
    cloud->at(i).y *= scale;
    cloud->at(i).z *= scale;
  }
}

PointCloud::Ptr ScaleCloud(PointCloud::ConstPtr cloud, float scale) {
  PointCloud::Ptr scaled_cloud (new PointCloud);
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

pcl::ModelCoefficients::Ptr GetCloudPlane(PointCloud::ConstPtr cloud) {
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

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
    const std::shared_ptr<cad_image_markup::CameraModel>& camera_model) {
  PointCloud::Ptr back_projected_cloud (new PointCloud);

  // get cad surface normal and point on the cad plane
  Eigen::Vector3d cad_normal(target_plane->values[0], target_plane->values[1],
                             target_plane->values[2]);
  Eigen::Vector3d cad_point(cad_cloud->at(0).x, cad_cloud->at(0).y,
                            cad_cloud->at(0).z);

  for (uint32_t i = 0; i < image_cloud->size(); i++) {
    Eigen::Vector3d image_point(0, 0, 0);
    Eigen::Vector2i image_pixel(image_cloud->at(i).x, image_cloud->at(i).y);
    if (camera_model->BackProject(image_pixel).has_value()) {
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

  }

  return back_projected_cloud;
}

pcl::PointXYZ GetCloudCentroid(PointCloud::ConstPtr cloud) {
  pcl::PointXYZ centroid;
  pcl::computeCentroid(*cloud, centroid);
  centroid.z = 0;
  return centroid;
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