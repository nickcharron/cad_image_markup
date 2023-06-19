#include <cad_image_markup/Utils.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

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

void CannyEdgeDetectToCloud(const std::string& src_filename,
                                          PointCloud::Ptr target,
                                          const int lowThreshold, 
                                          const int ratio,
                                          const int kernel_size) {
  cv::Mat src, src_gray, dst, detected_edges;

  src = cv::imread(src_filename, cv::IMREAD_COLOR);

  dst.create(src.size(), src.type());

  // detect edges
  cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
  cv::blur(src_gray, detected_edges, cv::Size(5, 5));
  cv::Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * ratio,
            kernel_size);

  dst = cv::Scalar::all(0);
  src.copyTo(dst, detected_edges);

  // get all non-black pixels
  for (int i = 0; i < dst.rows; i++) {
    for (int j = 0; j < dst.cols; j++) {
      int pixel_val = (int)dst.at<u_char>(i, j);

      pcl::PointXYZ point_pcl(j, i, 0);

      if (pixel_val > 0)
        target->push_back(point_pcl);
    }
  }

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

void DensifyPoints(PointCloud::Ptr points, uint8_t density_index) {
  // add additional point between existing points according to scale
  // will help to converge solution
  uint16_t init_length = points->size();

  for (uint16_t point_index = 0; point_index < init_length; point_index++) {
    pcl::PointXYZ current_start_point = points->at(0);
    pcl::PointXYZ current_end_point = points->at(1);

    points->erase(points->begin());

    // determine angle between points
    float slope, theta;

    if ((current_end_point.x - current_start_point.x) == 0) {
      theta = M_PI / 2;
    } else if ((current_end_point.y - current_start_point.y) == 0) {
      theta = 0;
    } else {
      slope = (float)(current_end_point.y - current_start_point.y) /
              (float)(current_end_point.x - current_start_point.x);
      theta = std::atan(std::abs(slope));
    }

    // determine distance between points
    float dist = std::sqrt(
        std::pow(std::abs(current_end_point.x - current_start_point.x), 2) +
        std::pow(std::abs(current_end_point.y - current_start_point.y), 2));

    float inter_dist = 10;

    // number of points added between each reference point should be the same
    // for both images for 1:1 mapping (in final solution)
    float interval = inter_dist / (density_index + 1);

    // determine delta x and y values based on quadrant
    float dx, dy;

    // first quadrant:
    if ((current_end_point.y - current_start_point.y) >= 0 &&
        (current_end_point.x - current_start_point.x) >= 0) {
      dx = interval * (std::cos(theta));
      dy = interval * (std::sin(theta));
    }
    // second quadrant
    else if ((current_end_point.y - current_start_point.y) >= 0 &&
             (current_end_point.x - current_start_point.x) < 0) {
      dx = -interval * (std::cos(theta));
      dy = interval * (std::sin(theta));
    }
    // third quadrant
    else if ((current_end_point.y - current_start_point.y) < 0 &&
             (current_end_point.x - current_start_point.x) <= 0) {
      dx = -interval * (std::cos(theta));
      dy = -interval * (std::sin(theta));
    }
    // fourth quadrant
    else if ((current_end_point.y - current_start_point.y) < 0 &&
             (current_end_point.x - current_start_point.x) > 0) {
      dx = interval * (std::cos(theta));
      dy = -interval * (std::sin(theta));
    }

    // push the start point first to conserve the order of the vector
    points->push_back(current_start_point);

    // push the rest of the interpolated points, trending toward the current end
    // point
    float current_x_coord = current_start_point.x + dx;
    float current_y_coord = current_start_point.y + dy;

    float current_dist = std::sqrt(
        std::pow(std::abs(current_x_coord - current_start_point.x), 2) +
        std::pow(std::abs(current_y_coord - current_start_point.y), 2));

    while (current_dist < dist) {
      pcl::PointXYZ current_inter_point(current_x_coord, current_y_coord, 0);
      points->push_back(current_inter_point);

      current_x_coord += dx;
      current_y_coord += dy;

      current_dist = std::sqrt(
          std::pow(std::abs(current_x_coord - current_start_point.x), 2) +
          std::pow(std::abs(current_y_coord - current_start_point.y), 2));
    }
  }
}

bool ReadPointsPNG(const std::string& filename,
                                PointCloud::Ptr points, std::string color,
                                int rate) {
  if (!boost::filesystem::exists(filename)) return false;

  int threshold = 150;
  int whitethreshold = 25;

  int pixel_point_count = 0;

  cv::Mat img = cv::imread(filename, cv::IMREAD_COLOR);

  // get all pixels of specified color
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      std::vector<int> pixel_vals = {img.at<cv::Vec3b>(i, j)[2],
                                     img.at<cv::Vec3b>(i, j)[1],
                                     img.at<cv::Vec3b>(i, j)[0]};
      pcl::PointXYZ point_pcl(j, i, 0);

      if (color == "red") {
        if (pixel_vals[0] >= threshold && pixel_vals[1] < (255 - threshold) &&
            pixel_vals[2] < (255 - threshold)) {
          if (pixel_point_count % rate == 0) points->push_back(point_pcl);
          pixel_point_count++;
        }

      } else if (color == "green") {
        if (pixel_vals[0] < (255 - threshold) && pixel_vals[1] >= threshold &&
            pixel_vals[2] < (255 - threshold)) {
          if (pixel_point_count % rate == 0) points->push_back(point_pcl);
          pixel_point_count++;
        }

      } else if (color == "blue") {
        if (pixel_vals[0] < (255 - threshold) &&
            pixel_vals[1] < (255 - threshold) && pixel_vals[2] >= threshold) {
          if (pixel_point_count % rate == 0) points->push_back(point_pcl);
          pixel_point_count++;
        }

      } else if (color == "white") {
        if (pixel_vals[0] >= whitethreshold &&
            pixel_vals[1] >= whitethreshold &&
            pixel_vals[2] >= whitethreshold) {
          if (pixel_point_count % rate == 0) points->push_back(point_pcl);
          pixel_point_count++;
        }

      } else if (color == "black") {
        if (pixel_vals[0] < (250 - threshold) &&
            pixel_vals[1] < (250 - threshold) &&
            pixel_vals[2] < (250 - threshold)) {
          if (pixel_point_count % rate == 0) points->push_back(point_pcl);
          pixel_point_count++;
        }
      } else {
        LOG_WARN("Invalid color selected for defect detection");
        return false;
      }
    }
  }

  return true;
}

bool WriteToImage(const PointCloud::Ptr& points,
                  const std::string& src_file_name,
                  const std::string& target_file_name, uint8_t r,
                  uint8_t g, uint8_t b) {
  // [NOTE] opencv use BGR not RGB
  cv::Vec3b color;
  color[0] = b;
  color[1] = g;
  color[2] = r;

  if (!boost::filesystem::exists(src_file_name)) {
    LOG_ERROR("OUTPUT BUFFER: Invalid path to input image: %s",
              src_file_name.c_str());
    return false;
  }
  LOG_INFO("OUTPUT BUFFER: Reading image: %s", src_file_name.c_str());

  cv::Mat image;
  image = cv::imread(src_file_name, 1);
  for (uint32_t i = 0; i < points->size(); i++) {
    if (points->at(i).x >= 0 && points->at(i).x <= image.cols &&
        points->at(i).y >= 0 && points->at(i).y <= image.rows)
      image.at<cv::Vec3b>((int)points->at(i).y, (int)points->at(i).x) = color;
  }

  LOG_INFO("OUTPUT BUFFER: Saving image to: %s", target_file_name.c_str());
  bool written = cv::imwrite(target_file_name, image);
  if (!written) { LOG_ERROR("OUTPUT BUFFER: Unable to write image."); }
  return written;
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

bool ReadPoints(const std::string& filename, PointCloud::Ptr points) {
  points->clear();

  if (!boost::filesystem::exists(filename)) {
    LOG_ERROR("Invalid path to points file: %s", filename.c_str());
    return false;
  }
  LOG_INFO("Loading points file: %s", filename.c_str());

  nlohmann::json J;
  std::ifstream file(filename);
  file >> J;

  nlohmann::json J_shapes = J["shapes"];
  for (auto J_point : J_shapes[0]["points"]) {
    std::vector<float> point_vec;
    for (auto val : J_point) { point_vec.push_back(val.get<float>()); }
    if (point_vec.size() != 2) {
      LOG_ERROR("Invalid point in points file.");
      return false;
    }
    pcl::PointXYZ point_pcl(point_vec.at(0), point_vec.at(1), 0);
    points->push_back(point_pcl);
  }

  return true;
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