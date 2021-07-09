#pragma once

#include <cad_image_markup/ImageBuffer.h>
#include <cad_image_markup/Solver.h>
#include <cad_image_markup/Utils.h>

namespace cad_image_markup {

  /**
   * @brief Struct for containing all parameters needed for this class
   * @param cad_cloud_scale CAD drawing scale in pixels/working unit (meters, feet, etc.)
   * @param max_solution_iterations maximum number of iterations of the overall solution 
   *                                before the solver exits
   * @param visualize enables or disables solution visualization
   * @param output_results enables or disables the terminal output of the solution results 
   *                       at each iteration
   * @param correspondence_type correspondence type for point-to-point solution or point-
   *                            to-plane solution - options are "P2POINT" or "P2PLANE"
   * @param max_corr_distance maximum distance for the solver to generate a correspondence (in pixels)
   * @param align_centroids option to align target and source centroids during correspondence
   *                        estimation
   * @param convergence_type convergence based on problem loss or transform geometry - options
   *                         are "LOSS_CONVERGENCE" or "GEO_CONVERGENCE"
   * @param convergence_condition absolute or differential convergence condition, absolute 
   *                              convergence only available with "LOSS_CONVERGENCE" type - 
   *                              options are "ABS_CONVERGENCE" or "DIFF_CONVERGENCE"
   * @param converged_differential_cost converged differential cost between overall solution
   *                                    iterations
   * @param converged_absolute_cost absolute converged overall solution cost 
   * @param converged_differential_translation converged differential translation in solution
   *                                           transform in all directions between overall
   *                                           solution iterations in working units
   * @param converged_differential_rotation converged differential rotation in solution
   *                                         transform in all axis between overall solution
   *                                         iterations in degrees
   * @param cad_density_index number of points to interpolate between each point in input cad cloud
   * @param cam_density_index number of points to interpolate between each point in input camera cloud
   */
  struct Params {
    /**
     * @brief Loads params from a json file
     * @param path full path to json
     */
    bool LoadFromJson(const std::string& path);

    std::string ceres_params_path;

    // Solution options 
    double cad_cloud_scale; // pixels/unit
    int max_solution_iterations;
    bool visualize;
    bool output_results;
    int correspondence_type; // "P2POINT" or "P2PLANE" 
    bool align_centroids;
    double max_corr_distance;
    bool minimizer_progress_to_stdout;

    // Convergence options
    int convergence_type; // "LOSS_CONVERGENCE" or "GEO_CONVERGENCE"
    double convergence_condition; // "DIFF_CONVERGENCE" or "ABS_CONVERGENCE" 
    double converged_differential_cost;
    double converged_absolute_cost;
    double converged_differential_translation;
    double converged_differential_rotation;


    double cad_density_index{2};
    double cam_density_index{10};
  };

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
   * @brief constructor
   */
  CadImageMarkup(const Inputs& inputs, Params& params);

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

  ImageBuffer image_buffer_;

  pcl::PointXYZ cad_centroid_;
  
  std::unique_ptr<Solver> solver_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr camera_points_CAMFRAME_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cad_points_CADFRAME_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cad_points_WORLDFRAME_;
};

}  // namespace cad_image_markup