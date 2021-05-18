#pragma once

#include <cstdio>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/numeric_diff_cost_function.h>
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
      const std::shared_ptr<cad_image_markup::camera_models::CameraModel>& camera_model,
      Eigen::Vector2d pixel_detected)
      : camera_model_(camera_model), pixel_detected_(pixel_detected) {}

  bool operator()(const double* P1, double* pixel1, const double* P2, double* pixel2) const {
    Eigen::Vector3d P1_CAMERA_eig{P1[0], P1[1], P1[2]}, 
    Eigen::Vector3d P2_CAMERA_eig{P2[0], P2[1], P2[2]};

    opt<Eigen::Vector2d> pixel_projected1 =
        camera_model_->ProjectPointPrecise(P1_CAMERA_eig);

    opt<Eigen::Vector2d> pixel_projected2 =
        camera_model_->ProjectPointPrecise(P2_CAMERA_eig);

    // get image dimensions in case projection fails
    uint16_t height =
        camera_model_->GetHeight() != 0 ? camera_model_->GetHeight() : 5000;
    uint16_t width =
        camera_model_->GetWidth() != 0 ? camera_model_->GetWidth() : 5000;

    // calculate the nearest edge point to the detected point
    // if the projection failed, set the projected point to
    // be the nearest edge point to the detected point
    int near_u =
        (width - pixel_detected_[0]) < pixel_detected_[0] ? width : 0;
    int dist_u = (width - pixel_detected_[0]) < pixel_detected_[0]
                      ? (width - pixel_detected_[0])
                      : pixel_detected_[0];
    int near_v =
        (height - pixel_detected_[1]) < pixel_detected_[1] ? height : 0;
    int dist_v = (height - pixel_detected_[1]) < pixel_detected_[1]
                      ? (height - pixel_detected_[1])
                      : pixel_detected_[1];

    if (pixel_projected1.has_value()) {
      pixel1[0] = pixel_projected1.value()[0];
      pixel1[1] = pixel_projected1.value()[1];
    } else {

      if (dist_u <= dist_v) {
        pixel1[0] = near_u;
        pixel1[1] = pixel_detected_[1];
      } else {
        pixel1[0] = pixel_detected_[0];
        pixel1[1] = near_v;
      }
    }

    if (pixel_projected2.has_value()) {
      pixel2[0] = pixel_projected2.value()[0];
      pixel2[1] = pixel_projected2.value()[1];
    } else {

      if (dist_u <= dist_v) {
        pixel2[0] = near_u;
        pixel2[1] = pixel_detected_[1];
      } else {
        pixel2[0] = pixel_detected_[0];
        pixel2[1] = near_v;
      }
    }

    return true;
  }

  std::shared_ptr<cad_image_markup::camera_models::CameraModel> camera_model_;
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
      Eigen::Vector2d pixel_detected, Eigen::Vector3d P_STRUCT1, Eigen::Vector3d P_STRUCT2
      std::shared_ptr<cad_image_markup::camera_models::CameraModel> camera_model)
      : pixel_detected_(pixel_detected),
        P_STRUCT1_(P_STRUCT1),
        P_STRUCT2_(P_STRUCT2),
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
    opt<Eigen::Vector2d> pixel_projected_check =
        camera_model_->ProjectPointPrecise(P_CAMERA_eig_check, outside_domain);

    // Need to handle outside domain failure differently for ladybug camera
    // model
    // since all points projecting out of frame provoke this failure
    if (camera_model_->GetType() == cad_image_markup::camera_models::CameraType::LADYBUG)
      return true;  // Returning outside_domain here would crash many
                    // viable solutions, error checking must be done in calling
                    // code
    else
      return !outside_domain;  // All other camera models have valid
                               // out-of-domain conditions that should be
                               // avoided
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(
      const Eigen::Vector2d pixel_detected, const Eigen::Vector3d P_STRUCT,
      const std::shared_ptr<cad_image_markup::camera_models::CameraModel> camera_model) {
    return (
        new ceres::AutoDiffCostFunction<CeresReprojectionCostFunction, 2, 7>(
            new CeresReprojectionCostFunction(pixel_detected, P_STRUCT,
                                              camera_model)));
  }

  Eigen::Vector2d pixel_detected_;
  Eigen::Vector3d P_STRUCT_;
  std::shared_ptr<cad_image_markup::camera_models::CameraModel> camera_model_;
  std::unique_ptr<ceres::CostFunctionToFunctor<2, 3>> compute_projection;

 private:
  bool checkDomain(const double* P) {
    Eigen::Vector3d P_CAMERA_eig{P[0], P[1], P[2]};
    bool outside_domain = false;
    opt<Eigen::Vector2d> pixel_projected =
        camera_model_->ProjectPointPrecise(P_CAMERA_eig, outside_domain);
    return outside_domain;
  }
};


/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////



/**
 * @brief Ceres cost functor for a point to line error where the line is defined
 * by two 3D points
 */
struct CeresPointToLineCostFunction {
  /**
   * @brief Constructor
   * @param P_TGT point in question
   * @param P_REF1 reference line point 1
   * @param P_REF2 reference line point 2
   */
  CeresPointToLineCostFunction(const Eigen::Vector3d P_TGT,
                               const Eigen::Vector3d P_REF1,
                               const Eigen::Vector3d P_REF2)
      : P_TGT_(P_TGT), P_REF1_(P_REF1), P_REF2_(P_REF2) {}

  // T_REF_TGT is [qw, qx, qy, qz, tx, ty, tz]
  template <typename T>
  bool operator()(const T* const T_REF_TGT, T* residuals) const {
    // cast member variables
    Eigen::Matrix<T,3,1> _P_TGT = P_TGT_.cast<T>();
    Eigen::Matrix<T,3,1> _P_REF1 = P_REF1_.cast<T>();
    Eigen::Matrix<T,3,1> _P_REF2 = P_REF2_.cast<T>();

    T P_TGT[3];
    P_TGT[0] = _P_TGT[0];
    P_TGT[1] = _P_TGT[1];
    P_TGT[2] = _P_TGT[2];

    // rotate and translate point
    T P_REF[3];
    ceres::QuaternionRotatePoint(T_REF_TGT, P_TGT, P_REF);
    P_REF[0] += T_REF_TGT[4];
    P_REF[1] += T_REF_TGT[5];
    P_REF[2] += T_REF_TGT[6];

    /*
     * e = distance from point to line
     *   = | (P_REF - P_REF1) x (P_REF - P_REF1) |
     *     ---------------------------------------
     *               | P_REF1 - P_REF2 |
     *
     *   = | d1 x d2 |
     *     -----------
     *       | d12 |
     *
     *   Where P_REF = T_REF_TGT * P_TGT
     *
     */
    T d1[3], d2[3], d12[3];
    d1[0] = P_REF[0] - _P_REF1[0];
    d1[1] = P_REF[1] - _P_REF1[1];
    d1[2] = P_REF[2] - _P_REF1[2];

    d2[0] = P_REF[0] - _P_REF2[0];
    d2[1] = P_REF[1] - _P_REF2[1];
    d2[2] = P_REF[2] - _P_REF2[2];

    d12[0] = _P_REF1[0] - _P_REF2[0];
    d12[1] = _P_REF1[1] - _P_REF2[1];
    d12[2] = _P_REF1[2] - _P_REF2[2];

    T cross[3];
    ceres::CrossProduct(d1, d2, cross);

    T norm =
        sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
    T norm12 = sqrt(d12[0] * d12[0] + d12[1] * d12[1] + d12[2] * d12[2]);

    residuals[0] = norm / norm12;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const Eigen::Vector3d P_TGT,
                                     const Eigen::Vector3d P_REF1,
                                     const Eigen::Vector3d P_REF2) {
    return (new ceres::AutoDiffCostFunction<CeresPointToLineCostFunction, 1, 7>(
        new CeresPointToLineCostFunction(P_TGT, P_REF1, P_REF2)));
  }

  Eigen::Vector3d P_TGT_;
  Eigen::Vector3d P_REF1_;
  Eigen::Vector3d P_REF2_;
};
