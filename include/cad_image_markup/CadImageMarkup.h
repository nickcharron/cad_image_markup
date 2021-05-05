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
     */
    bool LoadFromJson(const std::string& path);

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

  // TODO CAM: Does the visualizer need to be here or can it be constructed in
  // the solver only?
  std::shared_ptr<Visualizer> solver_visualizer_;
  std::unique_ptr<Solver> solver_;

  // TODO: do these need to be XYZ or can we use XY?
  // Also, to make these more clear we could rename them:
  // camera_points_CAMFRAME, cad_points_CADFRAME, cad_points_WORLDFRAME
  PointCloud::Ptr input_camera_points_;
  PointCloud::Ptr input_cad_points_;
  PointCloud::Ptr input_cad_points_transformed_;
};

}  // namespace cad_image_markup