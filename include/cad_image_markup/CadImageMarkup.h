#pragma once

#include <map>

#include <cad_image_markup/ImageBuffer.h>
#include <cad_image_markup/Solver.h>
#include <cad_image_markup/Params.h>
#include <cad_image_markup/Utils.h>

namespace cad_image_markup {

/**
 * @brief TODO
 */
class CadImageMarkup {
 public:
  /**
   * @brief Struct for containing all inputs needed for this class
   */
  struct Inputs {
    std::string cad_path;
    std::string image_path;
    std::string defect_path;
    std::string intrinsics_path;
    std::string config_path;
    std::string ceres_config_path;
    std::string initial_pose_path;  // T_WORLD_CAM
    std::string output_image_path;
  };

  /**
   * @brief constructor
   */
  CadImageMarkup(const Inputs& inputs);

  /**
   * @brief default deconstructor
   */
  ~CadImageMarkup() = default;

  /**
   * @brief TODO
   */
  bool Run();

 private:
  bool Setup();

  bool LoadData();

  bool Solve();

  /**
   * @brief Method to load initial poses
   * @param path absolute path to json file with initial pose
   * @param T_WORLD_CAMERA transformation matrix to which the read pose is
   * applied
   */
  void LoadInitialPose(const std::string& path,
                       Eigen::Matrix4d& T_WORLD_CAMERA);

  Inputs inputs_;
  Params params_;

  ImageBuffer image_buffer_;

  pcl::PointXYZ cad_centroid_;

  std::unique_ptr<Solver> solver_;

  PointCloud::Ptr camera_points_CAMFRAME_;
  PointCloud::Ptr defect_points_CAMFRAME_;
  PointCloud::Ptr cad_points_CADFRAME_;
  PointCloud::Ptr cad_points_WORLDFRAME_;
};

}  // namespace cad_image_markup