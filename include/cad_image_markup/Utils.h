#pragma once
#include <chrono>
#include <ctime>
#include <stdio.h>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cad_image_markup/Optional.h>
#include <cad_image_markup/Params.h>
#include <cad_image_markup/camera_models/CameraModel.h>

namespace cad_image_markup {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef Eigen::aligned_allocator<Eigen::Vector2d> AlignVec2d;
typedef std::chrono::steady_clock Clock;
typedef std::chrono::time_point<Clock> TimePoint;

namespace utils {

/**
 * @brief Method to center a cloud on the origin in the xy plane
 * cloud is centered by shifting its centroid to the origin
 * @param cloud cloud to be centered
 * @param centroid centroid of could
 */
void OriginCloudxy(PointCloud::Ptr cloud, const pcl::PointXYZ& centroid);

/**
 * @brief Method to use camera model to project a point cloud into the xy
 * plane
 * @param cloud point cloud to project
 * @param camera_model camera model used for projection
 * @return projected planar cloud in the xy plane
 */
PointCloud::Ptr ProjectCloud(
    PointCloud::Ptr cloud,
    const std::shared_ptr<cad_image_markup::CameraModel>& camera_model);

/**
 * @brief Method to get single correspondences between a CAD cloud projection
 * and an image cloud given transformation matrix
 * @param cad_cloud CAD structure cloud (centered, at correct scale,
 * untransformed)
 * @param camera_cloud camera image label cloud
 * @param T transformation matrix to apply to CAD cloud before projecting
 * (usually T_CS)
 * @param corrs nearest-neighbor correspondences between the CAD cloud
 * projection and the camera image cloud (image point to nearest structure
 * point)
 * @param max_corr_distance
 * @param num_corrs number of targets points for each source point(1 or 2)
 * @param align_centroids
 * @param source source of the correspondences, "projected" or "camera"
 * @param camera_model camera model used for projection
 */
void CorrespondenceEstimate(
    PointCloud::ConstPtr cad_cloud, PointCloud::ConstPtr camera_cloud,
    const Eigen::Matrix4d& T, pcl::CorrespondencesPtr corrs,
    bool align_centroids, double max_corr_distance, int num_corrs,
    std::string source,
    const std::shared_ptr<cad_image_markup::CameraModel>& camera_model);

/**
 * @brief Method to convert a vector of quaternions and translations to a
 * transformation matrix
 * @param pose vector of quaternions and translations (quaternions followed
 * by translations)
 * @return transformation matrix
 */
Eigen::Matrix4d
    QuaternionAndTranslationToTransformMatrix(const std::vector<double>& pose);

/**
 * @brief Method convert a transformation matrix to a quaternion and translation
 * @param T transformation matrix
 * @param q quaternion representing the rotation encapsulated in the
 * transformation matrix
 * @param p vector representing the translation encapsulated in the
 * transformation matrix
 * @return void
 */
void TransformMatrixToQuaternionAndTranslation(const Eigen::Matrix4d& T,
                                               Eigen::Quaterniond& q,
                                               Eigen::Vector3d& p);

Eigen::Matrix4d InvertTransformMatrix(const Eigen::Matrix4d& T);

/**
 * @brief Method to scale a cloud (in xyz)
 * @param cloud cloud to scale
 * @param scale scale to apply (updated cloud = original cloud * scale)
 */
void ScaleCloud(PointCloud::Ptr cloud, float scale);

/**
 * @brief Method to get the plane that best fits a cloud
 * @param T_WORLD_CAMERA transformation matrix from world (model) frame to
 * camera
 * @param image_cloud point cloud of pixel coordinates to back project
 * @param camera_model camera model used for the back projection
 * @return image cloud points back projected into the plane of the cad cloud
 */
PointCloud::Ptr BackProject(
    Eigen::Matrix4d T_WORLD_CAMERA, PointCloud::ConstPtr image_cloud,
    const std::shared_ptr<cad_image_markup::CameraModel>& camera_model);

/**
 * @brief Method to get the centroid of a cloud
 * @param cloud point cloud
 * @return point representing the centroid of the cloud
 */
pcl::PointXYZ GetCloudCentroid(PointCloud::ConstPtr cloud);

/**
 * @brief Method to perform Canny edge detection on the input images and CAD
 * drawings and store the result in a mat type
 * @param src_filename path to source file, supported types: .png
 * @param target pcl cloud datatype to write detected edges to
 * @param lowThreshold Canny low threshold
 * @param ratio Canny upper to lower threshold ratio
 * @param kernel_size Canny kernel size for internal Sobel convolution
 * operations
 */
void CannyEdgeDetectToCloud(const std::string& src_filename,
                    PointCloud::Ptr target,
                    const int lowThreshold = 50, const int ratio = 3,
                    const int kernel_size = 3);

/**
 * @brief Method to downsample the points in a cloud
 *        a grid filter is passed over the cloud and the points
 *        in each grid volume are replaced with their centroid in
 *        the output cloud
 * @param cloud point cloud
 * @param grid_size size of the grid to pass over the cloud, effectively in
 * pixels
 * @return downsampled point cloud
 */
PointCloud::Ptr DownSampleCloud(PointCloud::ConstPtr cloud,
                                const double grid_size);

/**
 * @brief Method to apply perturbations to a transform in radians
 * @param T unperturbed transform
 * @param perturbation perturbations (euler angles and translations) to
 * apply
 * @return perturbed transform
 */
Eigen::Matrix4d PerturbTransformRadM(const Eigen::Matrix4d& T,
                                     const Eigen::VectorXd& perturbation);

/**
 * @brief Method to apply perturbations to a transform in degrees
 * @param T unperturbed transform
 * @param perturbation perturbations (euler angles and translations) to
 * apply
 * @return perturbed transform
 */
Eigen::Matrix4d PerturbTransformDegM(const Eigen::Matrix4d& T,
                                     const Eigen::VectorXd& perturbation);

/**
 * @brief Method for reading labelled image feature data from a json file
 * @param filename_ absolute path to the json file to read data from
 * @param points pointcloud of points to fill in
 * @return read success
 */
bool ReadPoints(const std::string& filename, PointCloud::Ptr points);

Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d& eps);

Eigen::Matrix3d SkewTransform(const Eigen::Vector3d& V);

double DegToRad(double d);

double RadToDeg(double d);

std::string ConvertTimeToDate(std::chrono::system_clock::time_point time);



} // namespace utils

} // namespace cad_image_markup