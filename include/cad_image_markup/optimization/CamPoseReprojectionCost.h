#pragma once

#include <cstdio>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/rotation.h>

#include <cad_image_markup/Optional.h>
#include <cad_image_markup/camera_models/CameraModel.h>

/**
 * @brief Ceres cost functor for camera projection
 * performs projection of 3D point into camera frame
 * if the projection is outside of the image frame, the projected
 * pixel is set to the nearest edge point to its corresponding
 * detected pixel in the image
 */

struct CameraProjectionFunctor {
  CameraProjectionFunctor(
      const std::shared_ptr<cad_image_markup::CameraModel>& camera_model,
      const Eigen::Vector2d& pixel_detected)
      : camera_model_(camera_model), pixel_detected_(pixel_detected) {}

  bool operator()(const double* P, double* pixel) const {
    Eigen::Vector3d P_CAMERA_eig{P[0], P[1], P[2]};
    cad_image_markup::opt<Eigen::Vector2d> pixel_projected =
        camera_model_->ProjectPointPrecise(P_CAMERA_eig);

    // get image dimensions in case projection fails
    uint16_t height =
        camera_model_->GetHeight() != 0 ? camera_model_->GetHeight() : 5000;
    uint16_t width =
        camera_model_->GetWidth() != 0 ? camera_model_->GetWidth() : 5000;

    if (pixel_projected.has_value()) {
      pixel[0] = pixel_projected.value()[0];
      pixel[1] = pixel_projected.value()[1];
    } else {
      // if the projection failed, set the projected point to
      // be the nearest edge point to the detected point
      int near_u =
          (width - pixel_detected_[0]) < pixel_detected_[0] ? (width + 200) : -200;
      int dist_u = (width - pixel_detected_[0]) < pixel_detected_[0]
                       ? (width - pixel_detected_[0])
                       : pixel_detected_[0];
      int near_v =
          (height - pixel_detected_[1]) < pixel_detected_[1] ? (height + 200) : -200;
      int dist_v = (height - pixel_detected_[1]) < pixel_detected_[1]
                       ? (height - pixel_detected_[1])
                       : pixel_detected_[1];
      if (dist_u <= dist_v) {
        pixel[0] = near_u;
        pixel[1] = pixel_detected_[1];
      } else {
        pixel[0] = pixel_detected_[0];
        pixel[1] = near_v;
      }
    }

    return true;
  }

  std::shared_ptr<cad_image_markup::CameraModel> camera_model_;
  Eigen::Vector2d pixel_detected_;
};

/**
 * @brief Ceres reprojection cost function
 * sets the residual for the projection of a 3D point as determined by the
 * preceding functor and its corresponding detected pixel
 * if the projection of the 3D point is outside the domain of the camera model,
 * the cost function will return false
 * this will fail the ceres solution immediately on the first iteration or
 * after two consecutive false returns at any other point during the solution
 */

struct CeresReprojectionCostFunction {
  CeresReprojectionCostFunction(
      Eigen::Vector2d pixel_detected, Eigen::Vector3d P_STRUCT,
      std::shared_ptr<cad_image_markup::CameraModel> camera_model)
      : pixel_detected_(pixel_detected),
        P_STRUCT_(P_STRUCT),
        camera_model_(camera_model) {
    compute_projection.reset(new ceres::CostFunctionToFunctor<2, 3>(
        new ceres::NumericDiffCostFunction<CameraProjectionFunctor,
                                           ceres::CENTRAL, 2, 3>(
            new CameraProjectionFunctor(camera_model_, pixel_detected_))));
  }

  template <typename T>
  bool operator()(const T* const T_CR, T* residuals) const {
    T P_STRUCT[3];
    P_STRUCT[0] = P_STRUCT_.cast<T>()[0];
    P_STRUCT[1] = P_STRUCT_.cast<T>()[1];
    P_STRUCT[2] = P_STRUCT_.cast<T>()[2];

    // Rotate and translate point
    T P_CAMERA[3];
    ceres::QuaternionRotatePoint(T_CR, P_STRUCT, P_CAMERA);
    P_CAMERA[0] += T_CR[4];
    P_CAMERA[1] += T_CR[5];
    P_CAMERA[2] += T_CR[6];

    const T* P_CAMERA_const = &(P_CAMERA[0]);

    void* P_CAMERA_temp_x = &(P_CAMERA[0]);
    void* P_CAMERA_temp_y = &(P_CAMERA[1]);
    void* P_CAMERA_temp_z = &(P_CAMERA[2]);
    double* P_CAMERA_check_x{static_cast<double*>(P_CAMERA_temp_x)};
    double* P_CAMERA_check_y{static_cast<double*>(P_CAMERA_temp_y)};
    double* P_CAMERA_check_z{static_cast<double*>(P_CAMERA_temp_z)};

    T pixel_projected[2];
    (*compute_projection)(P_CAMERA_const, &(pixel_projected[0]));

    residuals[0] = pixel_detected_.cast<T>()[0] - pixel_projected[0];
    residuals[1] = pixel_detected_.cast<T>()[1] - pixel_projected[1];

    // Check if projection is outside the domain of the camera model
    Eigen::Vector3d P_CAMERA_eig_check{*P_CAMERA_check_x, *P_CAMERA_check_y,
                                       *P_CAMERA_check_z};
    bool outside_domain = false;
    cad_image_markup::opt<Eigen::Vector2d> pixel_projected_check =
        camera_model_->ProjectPointPrecise(P_CAMERA_eig_check, outside_domain);

    return !outside_domain;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(
      const Eigen::Vector2d pixel_detected, const Eigen::Vector3d P_STRUCT,
      const std::shared_ptr<cad_image_markup::CameraModel> camera_model) {
    return (
        new ceres::AutoDiffCostFunction<CeresReprojectionCostFunction, 2, 7>(
            new CeresReprojectionCostFunction(pixel_detected, P_STRUCT,
                                              camera_model)));
  }

  Eigen::Vector2d pixel_detected_;
  Eigen::Vector3d P_STRUCT_;
  std::shared_ptr<cad_image_markup::CameraModel> camera_model_;
  std::unique_ptr<ceres::CostFunctionToFunctor<2, 3>> compute_projection;

private:
  bool checkDomain(const double* P) {
    Eigen::Vector3d P_CAMERA_eig{P[0], P[1], P[2]};
    bool outside_domain = false;
    cad_image_markup::opt<Eigen::Vector2d> pixel_projected =
        camera_model_->ProjectPointPrecise(P_CAMERA_eig, outside_domain);
    return outside_domain;
  }
};
