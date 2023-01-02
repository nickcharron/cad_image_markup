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
  double max_corr_distance{1000};

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
  double cad_density_index{1};

  /** number of points to interpolate between each point in input camera cloud */
  double cam_density_index{2};

  /** source cloud for correspondences, "camera" or "projected" */
  std::string source_cloud{"camera"};

  // OUTPUT OPTIONS

  /** source cloud for correspondences, "camera" or "projected" */
  bool output_image{false};

  // MISC OPTIONS

  std::string defect_color{"red"};

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