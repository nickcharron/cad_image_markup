#pragma once

#include <stdio.h>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cad_image_markup/camera_models/CameraModel.h>
#include <cad_image_markup/Optional.h>
#include <cad_image_markup/Params.h>
#include <cad_image_markup/Log.h>

namespace cad_image_markup {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef Eigen::aligned_allocator<Eigen::Vector2d> AlignVec2d;

namespace utils {

extern double image_offset_x_;
extern double image_offset_y_;
extern bool center_image_called_;
extern std::shared_ptr<cad_image_markup::CameraModel> camera_model_;

/**
 * @brief Accessor method to retrieve camera model
 * @return camera model
 * CAM NOTE: not sure if this should return the utility camera model, or a copy
 * (probably the pointer since you might need to change it)
 */
std::shared_ptr<cad_image_markup::CameraModel> GetCameraModel();

/**
 * @brief Method to read the camera model used by the utility object from a
 * config file
 * @param intrinsics_file_path_ absolute path to the camera configuration file
 * @note There can exist only one utility camera model at a time
 */
void ReadCameraModel(std::string intrinsics_file_path);

/**
 * @brief Method to set the camera ID used by the camera model
 * this is currently only applicable to the ladybug camera model
 * @param cam_ID_ ID of the camera intrinsics set to use
 */
void SetCameraID(uint8_t cam_ID);

/**
 * @brief Method to offset a cloud in x and y
 * @param cloud cloud to be offset
 * @param offset x and y offset to ba applied to the cloud
 */
void OffsetCloudxy(PointCloud::Ptr cloud, Eigen::Vector2d offset);

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
 * @return projected planar cloud in the xy plane
 */
PointCloud::Ptr ProjectCloud(PointCloud::Ptr cloud);

/**
 * @brief Method to apply a transform to a point cloud, same behavior as pcl
 * transform function but Easier to use with rest of project
 * @param cloud_ original point cloud
 * @param T_ transformation matrix
 * @return transformed point cloud
 */
PointCloud::Ptr TransformCloud(PointCloud::ConstPtr cloud,
                               const Eigen::Matrix4d& T);

/**
 * @brief Method to apply a transform to a point cloud by updating the original
 * cloud
 * @param cloud_ point cloud to transform
 * @param T_ transformation matrix
 */
void TransformCloudUpdate(PointCloud::Ptr cloud, const Eigen::Matrix4d& T);

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
 */
void CorrespondenceEstimate(PointCloud::ConstPtr cad_cloud,
                            PointCloud::ConstPtr camera_cloud,
                            const Eigen::Matrix4d& T,
                            pcl::CorrespondencesPtr corrs, bool align_centroids,
                            double max_corr_distance, int num_corrs);

/**
 * @brief Method to convert a vector of quaternions and translations to a
 * transformation matrix
 * @param pose vector of quaternions and translations (quaternions followed
 * by translations)
 * @return transformation matrix
 */
Eigen::Matrix4d QuaternionAndTranslationToTransformMatrix(
    const std::vector<double>& pose);

/**
 * @brief Method to get the scale in x and y of a cloud with respect to the
 * original structure dimensions
 * @param cloud scaled point cloud
 * @param max_x_dim maximum x dimension of the real structure (likely from
 * CAD drawing)
 * @param max_y_dim maximum y dimension of the real structure (likely from
 * CAD drawing)
 * @param x_scale scale in x direction (CAD unit/pixel)
 * @param y_scale scale in y direction (CAD unit/pixel)
 */
void GetCloudScale(PointCloud::ConstPtr cloud, double max_x_dim,
                   double max_y_dim, float& x_scale, float& y_scale);

/**
 * @brief Method to scale a cloud (in xyz)
 * @param cloud cloud to scale
 * @param scale scale to apply (updated cloud = original cloud * scale)
 */
void ScaleCloud(PointCloud::Ptr cloud, float scale);

/**
 * @brief Method to scale a cloud (in xyz)
 * @param cloud cloud to scale
 * @param scale scale to apply (updated cloud = original cloud * scale)
 * @return scaled cloud
 */
PointCloud::Ptr ScaleCloud(PointCloud::ConstPtr cloud, float scale);

/**
 * @brief Method to scale a cloud in x and y with different scales in each
 * dimension
 * @param cloud cloud to scale
 * @param x_scale scale to apply in x (updated cloud = original cloud *
 * scale)
 * @param y_scale scale to apply in y (updated cloud = original cloud *
 * scale)
 * @todo not sure if we really need this one
 */
void ScaleCloud(PointCloud::Ptr cloud, float x_scale, float y_scale);

/**
 * @brief Method to get the plane that best fits a cloud
 * @todo CAM: Why do we even need this function? We only have 2D clouds meaning
 * we know the plane parameters because all points lie in that plane, so you can
 * just take any 3 points to calculate the plane params
 * @param cloud point cloud
 * @return pcl model coefficients object, planar equation coefficients are
 * given in form: [0] = a , [1] = b, [2] = c, [3] = d
 */
pcl::ModelCoefficients::Ptr GetCloudPlane(PointCloud::ConstPtr cloud);

/**
 * @brief Method to get the plane that best fits a cloud
 * @todo CAM add more details
 * @param image_cloud
 * @param cad_cloud
 * @param target_plane pcl model coefficients object, planar equation
 * coefficients are given in form: [0] = a , [1] = b, [2] = c, [3] = d
 * @return
 */
PointCloud::Ptr BackProject(
    PointCloud::ConstPtr image_cloud, PointCloud::ConstPtr cad_cloud,
    pcl::ModelCoefficients::ConstPtr target_plane,
    const std::shared_ptr<cad_image_markup::CameraModel>& camera_model);

pcl::PointXYZ GetCloudCentroid(PointCloud::ConstPtr cloud);

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

Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d& eps);

Eigen::Matrix3d SkewTransform(const Eigen::Vector3d& V);

double DegToRad(double d);

}  // namespace utils

}  // namespace cad_image_markup