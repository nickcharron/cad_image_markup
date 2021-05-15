#pragma once

#include <cad_image_markup/ImageBuffer.h>
#include <cad_image_markup/Solver.h>
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
    std::string intrinsics_path;
    std::string config_path;
    std::string ceres_config_path;
    std::string initial_pose_path;  // T_WORLD_CAM
  };

  /**
   * @brief Struct for containing all parameters needed for this class
   */
  struct Params {
    /**
     * @brief Loads params from a json file
     * @param path full path to json
     * TODO (figure out how calling this sets the members of this structure)
     */
    bool LoadFromJson(const std::string& path);

    std::string ceres_params_path = inputs_.config_path;

    // should be set from params files
    double cad_cloud_scale; 

    bool output_results;

    double converged_differential_cost;
    double converged_absolute_cost;
    double converged_differential_translation;
    double converged_absolute_translation;
    double converged_differential_rotation;
    double converged_absolute_rotation;

    cad_density_index{2};
    cam_density_index{10};
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

  Inputs inputs_;
  Params params_;

  ImageBuffer ImageBuffer;
  
  std::unique_ptr<Solver> solver_;

  // TODO: do these need to be XYZ or can we use XY?
  // CAM NOTE: they could be XY for now, but they end up being converted 
  //           for correspondences and visualization, I just found it more 
  //           intuitive to carry them through as point clouds rather than 
  //           switch back and forth
  PointCloud::Ptr camera_points_CAMFRAME_;
  PointCloud::Ptr cad_points_CADFRAME_;
  PointCloud::Ptr cad_points_WORLDFRAME_;
};

}  // namespace cad_image_markup