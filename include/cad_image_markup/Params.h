#pragma once

#include <stdio.h>
#include <string>
#include <map>

#include <cad_image_markup/nlohmann/json.h>
#include <cad_image_markup/Log.h>

namespace cad_image_markup {

// Enum class for different convergence types
enum class ConvergenceType { GEOMETRIC = 0, LOSS = 1};

// Map for storing string input
static std::map<std::string, ConvergenceType> ConvergenceTypeStringMap = {
    {"GEOMETRIC", ConvergenceType::GEOMETRIC}, {"LOSS", ConvergenceType::LOSS}};

// function for listing types of Descriptor available
inline std::string GetConvergenceTypes() {
  std::string types;
  for (auto it = ConvergenceTypeStringMap.begin();
       it != ConvergenceTypeStringMap.end(); it++) {
    types += it->first;
    types += ", ";
  }
  types.erase(types.end() - 2, types.end());
  return types;
}

// Enum class for different convergence conditions
enum class ConvergenceCondition { DIFFERENCE = 0, ABSOLUTE = 1};

// Map for storing string input
static std::map<std::string, ConvergenceCondition>
    ConvergenceConditionStringMap = {
        {"DIFFERENCE", ConvergenceCondition::DIFFERENCE},
        {"ABSOLUTE", ConvergenceCondition::ABSOLUTE}};

// function for listing types of Descriptor available
inline std::string GetConvergenceConditions() {
  std::string types;
  for (auto it = ConvergenceConditionStringMap.begin();
       it != ConvergenceConditionStringMap.end(); it++) {
    types += it->first;
    types += ", ";
  }
  types.erase(types.end() - 2, types.end());
  return types;
}

// Enum class for different convergence conditions
enum class CorrespondenceType { P2POINT = 0, P2LINE };

// Map for storing string input
static std::map<std::string, CorrespondenceType> CorrespondenceTypeStringMap = {
    {"P2POINT", CorrespondenceType::P2POINT},
    {"P2LINE", CorrespondenceType::P2LINE}};

// function for listing types of Descriptor available
inline std::string GetCorrespondenceTypes() {
  std::string types;
  for (auto it = CorrespondenceTypeStringMap.begin();
       it != CorrespondenceTypeStringMap.end(); it++) {
    types += it->first;
    types += ", ";
  }
  types.erase(types.end() - 2, types.end());
  return types;
}

/**
 * @brief Struct for containing all parameters needed for this package
 */
struct Params {
  // SOLUTION OPTIONS

  /** CAD drawing scale in working unit/pixel (meters, feet, etc.) */
  double cad_cloud_scale{0.01};
  double cad_cloud_scale_x{100};
  double cad_cloud_scale_y{100};

  /** Alternatively, can provide surface dimensions (in working unit) to calculate scale automatically */
  double max_x_dim{1.5}; 
  double max_y_dim{3};

  /** maximum number of correspondence iterations of the overall solution
   * before the solver exits */
  int max_solution_iterations{30};

  /** enables or disables solution visualization */
  bool visualize{true};

  /** enables or disables the terminal output of the solution results at each
   * iteration */
  bool output_results{true};

  /** correspondence type for point-to-point solution or point- to-plane
   * solution - options are CorrespondenceType::P2POINT or
   * CorrespondenceType::P2LINE */
  CorrespondenceType correspondence_type{CorrespondenceType::P2POINT};

  /** option to align target and source centroids during correspondence
   * estimation */
  bool align_centroids{true};

  /**  maximum distance for the solver to generate a correspondence (in
   * pixels)
   */
  double max_corr_distance{10};

  /**  option to decrease the maximum correspondence distance as the solution converges
   * useful to deal with noise in auto-detected image features 
   * provided a good initial estimate is given to the solver 
   */
  bool attenuate_corr_distance{false};

  /**  minimum correspondence distance to reach asymptotically if attenuation is selected
   */
  double corr_bound_low{5000};

  /**  rate parameter to control the speed of the maximum correspondence distance 
   * formula : dist_k+1 = (dist_k-dist_low_bound)*rate + dist_low_bound
   */
  double attenuation_rate{0.8};

  // CONVERGENCE OPTIONS

  /**  convergence based on problem loss or transform geometry - options are
   * "ConvergenceType::LOSS" or "ConvergenceType::GEOMETRIC"
   */
  ConvergenceType convergence_type{ConvergenceType::LOSS};

  /** absolute or differential convergence condition, absolute convergence
   * only available with ""ConvergenceType::LOSS" type - options are
   * "ConvergenceCondition::DIFFERENCE" or
   * "ConvergenceCondition::ABSOLUTE" */
  ConvergenceCondition convergence_condition{ConvergenceCondition::DIFFERENCE};

  /** converged cost between correspondence iterations */
  double converged_differential_cost{1};

  /** absolute cost criteria for convergence of correspondence iterations */
  double converged_absolute_cost{100};

  /** converged differential translation in solution transform in all
   * directions between correspondence iterations in working units (m)*/
  double converged_differential_translation{0.01};

  /** converged differential rotation in solution transform in all axis
   * between correspondence iterations in degrees */
  double converged_differential_rotation{1};

  /** number of points to interpolate between each point in input cad cloud */
  double cad_density_index{0.01};

  /** number of points to interpolate between each point in input camera cloud */
  double cam_density_index{0.02};

  /** source cloud for correspondences, "camera" or "projected" */
  std::string source_cloud{"camera"};

  // OUTPUT OPTIONS

  /** source cloud for correspondences, "camera" or "projected" */
  bool output_image{false};

  // MISC OPTIONS

  std::string defect_color{"red"};

  double cad_crop_offset_x{0};
  double cad_crop_offset_y{0};

  /** how the reference features are provided to the solver, options are: 
   * MANUAL: a .json file is provided with manually labeled features in the form of polygons
   * AUTOMATIC: only an image file is provided and the system performs Canny edge detection to 
   *            extract the features to match 
   */ 
  std::string feature_label_type{"MANUAL"};


  /** Canny algorithm parameters for automatic edge detection
   * lowThreshold:  Canny low threshold 
   * ratio:  Canny upper to lower threshold ratio
   * kernel_size: Canny kernel size for internal Sobel convolution operations
   */
  int cannny_low_threshold_cad, canny_ratio_cad, canny_kernel_size_cad; 
  int cannny_low_threshold_image, canny_ratio_image, canny_kernel_size_image; 

  /** Option to downsample image cloud, useful with automatic line detection */
  bool downsample_image_cloud{false};

  /** Size of grid filter to use to downsample image cloud, units are effectively in pixels */
  double downsample_grid_size{10};

  /**
   * @brief Loads params from a json file
   * @param path full path to json
   */
  inline bool LoadFromJson(const std::string& path) {
    if (path.empty()) {
      return true;
    }

    if (!boost::filesystem::exists(path)) {
      LOG_ERROR("Invalid path to config file: %s", path.c_str());
      return false;
    }

    LOG_INFO("Loading config file from: %s", path.c_str());

    nlohmann::json J;
    std::ifstream file(path);
    file >> J;

    nlohmann::json J_solution_options = J["solution_options"];

    cad_cloud_scale = J_solution_options["cad_cloud_scale"];
    max_solution_iterations = J_solution_options["max_solution_iterations"];
    visualize = J_solution_options["visualize"];
    output_results = J_solution_options["output_results"];
    align_centroids = J_solution_options["align_centroids"];
    max_corr_distance = J_solution_options["max_corr_distance"];
    output_results = J_solution_options["output_results"];
    attenuate_corr_distance = J_solution_options["attenuate_corr_distance"];
    corr_bound_low = J_solution_options["corr_bound_low"];
    attenuation_rate = J_solution_options["attenuation_rate"];

    nlohmann::json J_convergence_options = J["convergence_options"];

    converged_differential_cost =
        J_convergence_options["converged_differential_cost"];
    converged_absolute_cost = J_convergence_options["converged_absolute_cost"];
    converged_differential_translation =
        J_convergence_options["converged_differential_translation_m"];
    converged_differential_rotation =
        J_convergence_options["converged_differential_rotation_deg"];

    nlohmann::json J_output_options = J["output_options"];

    output_image = J_output_options["output_image"];

    nlohmann::json J_misc_options = J["misc_options"];

    defect_color = J_misc_options["defect_color"];

    feature_label_type = J_misc_options["feature_label_type"];

    cannny_low_threshold_cad = J_misc_options["cannny_low_threshold_cad"];
    canny_ratio_cad = J_misc_options["canny_ratio_cad"];
    canny_kernel_size_cad = J_misc_options["canny_kernel_size_cad"];
    cannny_low_threshold_image = J_misc_options["cannny_low_threshold_image"];
    canny_ratio_image = J_misc_options["canny_ratio_image"];
    canny_kernel_size_image = J_misc_options["canny_kernel_size_image"];

    downsample_image_cloud = J_misc_options["downsample_image_cloud"];
    downsample_grid_size = J_misc_options["downsample_grid_size"];

    // this shouldn't ever change, even when adding new types
    auto ct_iter = CorrespondenceTypeStringMap.find(
        J_solution_options["correspondence_type"]);
    if (ct_iter == CorrespondenceTypeStringMap.end()) {
      std::string options_str = GetCorrespondenceTypes();
      LOG_ERROR("Invalid correspondence_type param, options: %s. Exiting...",
                options_str.c_str());
      return false;
    } else {
      correspondence_type = ct_iter->second;
    }

    // this shouldn't ever change, even when adding new types
    auto cvt_iter = ConvergenceTypeStringMap.find(
        J_convergence_options["convergence_type"]);
    if (cvt_iter == ConvergenceTypeStringMap.end()) {
      std::string options_str = GetConvergenceTypes();
      LOG_ERROR("Invalid convergence_type param, options: %s. Exiting...",
                options_str.c_str());
      return false;
    } else {
      convergence_type = cvt_iter->second;
    }

    // this shouldn't ever change, even when adding new types
    auto cc_iter = ConvergenceConditionStringMap.find(
        J_convergence_options["convergence_condition"]);
    if (cc_iter == ConvergenceConditionStringMap.end()) {
      std::string options_str = GetConvergenceConditions();
      LOG_ERROR("Invalid convergence_condition param, options: %s. Exiting...",
                options_str.c_str());
      return false;
    } else {
      convergence_condition = cc_iter->second;
    }

    // validate CC and CT
    if (convergence_condition == ConvergenceCondition::ABSOLUTE &&
        convergence_type == ConvergenceType::GEOMETRIC) {
      LOG_ERROR(
          "Absolute convergence condition is not available for geometric "
          "convergence type. Exiting...");
      return false;
    }

    return true;
  }
};

}  // namespace cad_image_markup