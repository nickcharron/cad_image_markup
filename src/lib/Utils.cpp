#include <cad_image_markup/Utils.h>

#include <Eigen/Dense>
#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <unsupported/Eigen/MatrixFunctions>

#include <cad_image_markup/Log.h>

namespace cad_image_markup { namespace utils {

void OriginCloudxy(PointCloud::Ptr cloud, const pcl::PointXYZ& centroid) {
  uint16_t num_points = cloud->size();

  // shift all points back to center on origin
  for (uint16_t point_index = 0; point_index < num_points; point_index++) {
    cloud->at(point_index).x -= centroid.x;
    cloud->at(point_index).y -= centroid.y;
  }
}

PointCloud::Ptr ProjectCloud(
    PointCloud::Ptr cloud,
    const std::shared_ptr<cad_image_markup::CameraModel>& camera_model) {
  PointCloud::Ptr proj_cloud(new PointCloud);

  for (uint16_t i = 0; i < cloud->size(); i++) {
    Eigen::Vector3d point(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
    cad_image_markup::optional<Eigen::Vector2d> pixel_projected;

    pixel_projected = camera_model->ProjectPointPrecise(point);
    if (pixel_projected.has_value()) {
      pcl::PointXYZ proj_point(pixel_projected.value()(0),
                               pixel_projected.value()(1), 0);
      proj_cloud->push_back(proj_point);
    }
  }

  return proj_cloud;
}

void CorrespondenceEstimate(
    PointCloud::ConstPtr cad_cloud, PointCloud::ConstPtr camera_cloud,
    const Eigen::Matrix4d& T, pcl::CorrespondencesPtr corrs,
    bool align_centroids, double max_corr_distance, int num_corrs,
    std::string source,
    const std::shared_ptr<cad_image_markup::CameraModel>& camera_model) {
  // clear the previous correspondences
  corrs->clear();

  // transform the CAD cloud points to the camera frame
  PointCloud::Ptr trans_cloud(new PointCloud);
  pcl::transformPointCloud(*cad_cloud, *trans_cloud, T);

  // project the transformed points to the camera plane
  PointCloud::Ptr proj_cloud = ProjectCloud(trans_cloud, camera_model);

  // merge centroids for correspondence estimation (projected -> camera)
  if (align_centroids) {
    pcl::PointXYZ camera_centroid = GetCloudCentroid(camera_cloud);
    pcl::PointXYZ proj_centroid = GetCloudCentroid(proj_cloud);

    Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
    T1(0, 3) = camera_centroid.x - proj_centroid.x;
    T1(1, 3) = camera_centroid.y - proj_centroid.y;
    T1(2, 3) = camera_centroid.z - proj_centroid.z;
    pcl::transformPointCloud(*proj_cloud, *proj_cloud, T1);
  }

  // Get correspondences (source camera)
  if (source == "camera") {
    // Build kd tree from projected cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> proj_kdtree;
    proj_kdtree.setInputCloud(proj_cloud);

    for (uint32_t i = 0; i < camera_cloud->size(); i++) {
      std::vector<int> target_neighbor_indices(num_corrs);
      std::vector<float> target_neighbor_distances(num_corrs);
      pcl::Correspondence corr1;
      pcl::Correspondence corr2;

      if (proj_kdtree.nearestKSearch(camera_cloud->at(i), num_corrs,
                                     target_neighbor_indices,
                                     target_neighbor_distances) >= num_corrs) {
        corr1.index_query = i;
        corr1.index_match = target_neighbor_indices[0];

        if (target_neighbor_distances[0] <= max_corr_distance)
          corrs->push_back(corr1);

        if (num_corrs == 2 &&
            target_neighbor_distances[1] <= max_corr_distance) {
          corr1.index_query = i;
          corr1.index_match = target_neighbor_indices[1];

          corrs->push_back(corr2);
        }

        else if (num_corrs == 2 &&
                 target_neighbor_distances[1] > max_corr_distance)
          corrs->pop_back();
      }
    }
  }

  // Get correspondences (source CAD)
  if (source == "projected") {
    // Build kd tree from projected cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> camera_kdtree;
    camera_kdtree.setInputCloud(camera_cloud);

    for (uint32_t i = 0; i < proj_cloud->size(); i++) {
      std::vector<int> target_neighbor_indices(num_corrs);
      std::vector<float> target_neighbor_distances(num_corrs);
      pcl::Correspondence corr1;
      pcl::Correspondence corr2;

      if (camera_kdtree.nearestKSearch(
              proj_cloud->at(i), num_corrs, target_neighbor_indices,
              target_neighbor_distances) >= num_corrs) {
        corr1.index_query = i;
        corr1.index_match = target_neighbor_indices[0];

        if (target_neighbor_distances[0] <= max_corr_distance)
          corrs->push_back(corr1);

        if (num_corrs == 2 &&
            target_neighbor_distances[1] <= max_corr_distance) {
          corr1.index_query = i;
          corr1.index_match = target_neighbor_indices[1];

          corrs->push_back(corr2);
        }

        else if (num_corrs == 2 &&
                 target_neighbor_distances[1] > max_corr_distance)
          corrs->pop_back();
      }
    }
  }
}

Eigen::Matrix4d
    QuaternionAndTranslationToTransformMatrix(const std::vector<double>& pose) {
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
  inverse_transform.block<3, 3>(0, 0) = (T.block<3, 3>(0, 0)).transpose();

  // the translations are:
  inverse_transform.block<3, 1>(0, 3) =
      -inverse_transform.block<3, 3>(0, 0) * T.block<3, 1>(0, 3);

  return inverse_transform;
}

void ScaleCloud(PointCloud::Ptr cloud, float scale) {
  for (uint16_t i = 0; i < cloud->size(); i++) {
    cloud->at(i).x *= scale;
    cloud->at(i).y *= scale;
    cloud->at(i).z *= scale;
  }
}

PointCloud::Ptr BackProject(
    Eigen::Matrix4d T_WORLD_CAMERA, PointCloud::ConstPtr image_cloud,
    const std::shared_ptr<cad_image_markup::CameraModel>& camera_model) {
  PointCloud::Ptr back_projected_cloud(new PointCloud);

  // get cad surface normal and point on the cad plane
  Eigen::Vector3d cad_normal_cad(0, 0, 1);
  Eigen::Vector3d cad_normal_image =
      T_WORLD_CAMERA.block(0, 0, 3, 3) * cad_normal_cad;

  for (uint32_t i = 0; i < image_cloud->size(); i++) {
    Eigen::Vector3d image_origin(0, 0, 0);
    Eigen::Vector2i image_pixel(image_cloud->at(i).x, image_cloud->at(i).y);

    if (camera_model->BackProject(image_pixel).has_value()) {
      Eigen::Vector3d ray_unit_vector =
          camera_model->BackProject(image_pixel).value().normalized();

      double nlength = (image_origin - T_WORLD_CAMERA.block(0, 3, 3, 1))
                           .dot(cad_normal_image);
      double scale = nlength / (ray_unit_vector.dot(cad_normal_image));

      Eigen::Vector3d back_projected_point =
          image_origin - ray_unit_vector * scale;

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

PointCloud::Ptr DownSampleCloud(PointCloud::ConstPtr cloud,
                                const double grid_size) {
  PointCloud::Ptr downsampled_cloud(new PointCloud);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(grid_size, grid_size,
                  0); // image grid is 2D, don't need a z-direction
  sor.filter(*downsampled_cloud);

  return downsampled_cloud;
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

double DegToRad(double d) {
  return d * (M_PI / 180);
}
double RadToDeg(double r) {
  return r * (180 / M_PI);
}

std::string ConvertTimeToDate(std::chrono::system_clock::time_point time) {
  using namespace std;
  using namespace std::chrono;
  system_clock::duration tp = time.time_since_epoch();
  time_t tt = system_clock::to_time_t(time);
  tm local_tm = *localtime(&tt);

  string outputTime =
      to_string(local_tm.tm_year + 1900) + "_" +
      to_string(local_tm.tm_mon + 1) + "_" + to_string(local_tm.tm_mday) + "_" +
      to_string(local_tm.tm_hour) + "_" + to_string(local_tm.tm_min) + "_" +
      to_string(local_tm.tm_sec);
  return outputTime;
}

}} // namespace cad_image_markup::utils