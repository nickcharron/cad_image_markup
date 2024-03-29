#pragma once

#include <memory>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <cad_image_markup/Optional.h>

static bool outside_domain_default_{false};

namespace cad_image_markup {

/**
 * @brief Enum class for different types of intrinsic calibrations
 */
enum class CameraType { RADTAN = 0, KANNALABRANDT, DOUBLESPHERE };

/**
 * @brief Abstract class for camera models
 */
class CameraModel {
public:
  /**
   * @brief Default constructor
   */
  CameraModel() = default;

  /**
   * @brief Default destructor
   */
  virtual ~CameraModel() = default;

  /**
   * @brief Factory method to create camera models at runtime
   */
  static std::shared_ptr<CameraModel> Create(std::string& file_location);

  /**
   * @brief Method for projecting a point into an image plane (continous)
   * @param point 3d point to be projected [x,y,z]^T
   * @param outside_domain optional parameter, set if point is outside camera
   * model domain
   */
  virtual opt<Eigen::Vector2d>
      ProjectPointPrecise(const Eigen::Vector3d& point,
                          bool& outside_domain = outside_domain_default_) = 0;

  /**
   * @brief Method for projecting a point into an image plane
   * @param point 3d point to be projected [x,y,z]^T
   * @param outside_domain optional parameter, set if point is outside camera
   * model domain
   */
  virtual opt<Eigen::Vector2i>
      ProjectPoint(const Eigen::Vector3d& point,
                   bool& outside_domain = outside_domain_default_) = 0;

  /**
   * @brief Overload projection function for computing jacobian of projection
   * @param point 3d point to be projected [x,y,z]^T
   * @param J 2 x 3 projection jacobian.
   * For ProjectPoint: [u,v]^T = [P1(x, y, z), P2(x, y, z)]^T
   *                   J = | dP1/dx , dP1/dy, dP1/dz |
   *                       | dP2/dx , dP2/dy, dP2/dz |
   * @param outside_domain optional parameter, set if point is outside camera
   * model domain
   */
  virtual opt<Eigen::Vector2i>
      ProjectPoint(const Eigen::Vector3d& point, Eigen::MatrixXd& J,
                   bool& outside_domain = outside_domain_default_) = 0;

  /**
   * @brief Method back projecting
   * @return Returns an unnormalized bearing vector where z = 1 to a pixel in
   * the image, from camera center = [0,0,0]
   * @param point [u = col, v = row]
   */
  virtual opt<Eigen::Vector3d> BackProject(const Eigen::Vector2i& pixel) = 0;

  /**
   * @brief Method for setting the camera ID
   * @param id of the camera to use
   */
  virtual void SetCameraID(const unsigned int id);

  /**
   * @brief Method for adding the frame id
   * @param frame_id frame associated with the intrinsics calibration object
   */
  void SetFrameID(const std::string& id);

  /**
   * @brief Method for setting the date that the calibration was done
   * @param Calibration date
   */
  void SetCalibrationDate(const std::string& date);

  /**
   * @brief Method for adding the image dimensions
   * @param height and width of the image plane
   */
  void SetImageDims(const uint32_t height, const uint32_t width);

  /**
   * @brief Method for adding intrinsic values
   * @param intrinsics of the camera
   */
  void SetIntrinsics(const Eigen::VectorXd& instrinsics);

  /**
   * @brief Method for returning the frame id of a camera object
   * @return Returns frame id
   */
  const std::string GetFrameID() const;

  /**
   * @brief Method for retrieving the date that the calibration was done
   * @return Return calibration date
   */
  const std::string GetCalibrationDate() const;

  /**
   * @brief Method for getting the image height
   * @return image height
   */
  uint32_t GetHeight() const;

  /**
   * @brief Method for getting the image width
   * @return image width
   */
  uint32_t GetWidth() const;

  /**
   * @brief Method for retrieving the intrinsic values of the model
   * @return intrinsics of the camera
   */
  const Eigen::VectorXd GetIntrinsics() const;

  /**
   * @brief Method for retrieving the camera type
   * @return camera type
   */
  CameraType GetType() const;

  /**
   * @brief Method for checking if pixel is in image
   * @return Returns boolean
   * @param pixel
   */
  bool PixelInImage(const Eigen::Vector2i& pixel);

  /**
   * @brief Method for checking if pixel is in image
   * @return Returns boolean
   * @param pixel
   */
  bool PixelInImage(const Eigen::Vector2d& pixel);

  /**
   * @brief Method for writing camera model to a json file
   * @param file_path full path to json
   */
  void WriteJSON(const std::string& file_path,
                 const std::string& method = std::string());

protected:
  /**
   * @brief Method for loading calibration information from a json.
   * @param file_path full path to json
   */
  void LoadJSON(const std::string& file_path);

  /**
   * @brief Method for outputting all camera model types from intrinsics_types_
   */
  void OutputCameraTypes();

  CameraType type_; // THIS SHOULD BE SET IN EACH DERIVED CLASS CONSTRUCTOR
  std::string frame_id_{""};
  std::string calibration_date_{""};
  uint32_t image_height_{0};
  uint32_t image_width_{0};
  Eigen::VectorXd intrinsics_;

  // static bool outside_domain_default_ = false;

  unsigned int cam_id_ = 0;

  // Map for keeping required number of values in distortion vector
  std::map<CameraType, int> intrinsics_size_ = {{CameraType::RADTAN, 8},
                                                {CameraType::KANNALABRANDT, 8},
                                                {CameraType::DOUBLESPHERE, 6}};
  // Map for storing string input
  std::map<std::string, CameraType> intrinsics_types_ = {
      {"RADTAN", CameraType::RADTAN},
      {"KANNALABRANDT", CameraType::KANNALABRANDT},
      {"DOUBLESPHERE", CameraType::DOUBLESPHERE}};
};

} // namespace cad_image_markup
