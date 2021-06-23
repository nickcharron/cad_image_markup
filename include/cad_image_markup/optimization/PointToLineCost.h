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

namespace point_to_line_cost { 

struct CameraProjectionFunctor {
  CameraProjectionFunctor(
      const std::shared_ptr<cad_image_markup::CameraModel>& camera_model,
      Eigen::Vector2d pixel_detected)
      : camera_model_(camera_model), pixel_detected_(pixel_detected) {}

  bool operator()(const double* P, double* pixel) const {

    // P [0..2] -> P1
    // P [3..5] -> P2
    // pixel [0,1] -> pixel1
    // pixel [2,3] -> pixel2


    Eigen::Vector3d P1_CAMERA_eig{P[0], P[1], P[2]}; 
    Eigen::Vector3d P2_CAMERA_eig{P[3], P[4], P[5]};

    cad_image_markup::opt<Eigen::Vector2d> pixel_projected1 =
        camera_model_->ProjectPointPrecise(P1_CAMERA_eig);

    cad_image_markup::opt<Eigen::Vector2d> pixel_projected2 =
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
      pixel[0] = pixel_projected1.value()[0];
      pixel[1] = pixel_projected1.value()[1];
    } else {

      if (dist_u <= dist_v) {
        pixel[0] = near_u;
        pixel[1] = pixel_detected_[1];
      } else {
        pixel[0] = pixel_detected_[0];
        pixel[1] = near_v;
      }
    }

    if (pixel_projected2.has_value()) {
      pixel[2] = pixel_projected2.value()[0];
      pixel[3] = pixel_projected2.value()[1];
    } else {

      if (dist_u <= dist_v) {
        pixel[2] = near_u;
        pixel[3] = pixel_detected_[1];
      } else {
        pixel[2] = pixel_detected_[0];
        pixel[2] = near_v;
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
      Eigen::Vector2d pixel_detected, Eigen::Vector3d P_STRUCT1, Eigen::Vector3d P_STRUCT2,
      std::shared_ptr<cad_image_markup::CameraModel> camera_model)
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

    T _P_REF1[3];
    _P_REF1[0] = P_STRUCT1_.cast<T>()[0];
    _P_REF1[1] = P_STRUCT1_.cast<T>()[1];
    _P_REF1[2] = P_STRUCT1_.cast<T>()[2];

    T _P_REF2[3];
    _P_REF2[0] = P_STRUCT2_.cast<T>()[0];
    _P_REF2[1] = P_STRUCT2_.cast<T>()[1];
    _P_REF2[2] = P_STRUCT2_.cast<T>()[2];


   //Eigen::Matrix<T,3,1> _P_REF1 = P_STRUCT1_.cast<T>();
   //Eigen::Matrix<T,3,1> _P_REF2 = P_STRUCT2_.cast<T>();

    // rotate and translate reference points into camera frame
    T P_CAMERA1[3];
    ceres::QuaternionRotatePoint(T_CR, _P_REF1, P_CAMERA1);
    P_CAMERA1[0] += T_CR[4];
    P_CAMERA1[1] += T_CR[5];
    P_CAMERA1[2] += T_CR[6];

    // rotate and translate reference points into camera frame
    T P_CAMERA2[3];
    ceres::QuaternionRotatePoint(T_CR, _P_REF2, P_CAMERA2);
    P_CAMERA2[0] += T_CR[4];
    P_CAMERA2[1] += T_CR[5];
    P_CAMERA2[2] += T_CR[6];

    // Project reference points in camera frame into camera plane
    const T* P_CAMERA_const1 = &(P_CAMERA1[0]);
    const T* P_CAMERA_const2 = &(P_CAMERA1[0]);

    void* P_CAMERA_temp1_x = &(P_CAMERA1[0]);
    void* P_CAMERA_temp1_y = &(P_CAMERA1[1]);
    void* P_CAMERA_temp1_z = &(P_CAMERA1[2]);
    double* P_CAMERA_check1_x{static_cast<double*>(P_CAMERA_temp1_x)};
    double* P_CAMERA_check1_y{static_cast<double*>(P_CAMERA_temp1_y)};
    double* P_CAMERA_check1_z{static_cast<double*>(P_CAMERA_temp1_z)};

    void* P_CAMERA_temp2_x = &(P_CAMERA2[0]);
    void* P_CAMERA_temp2_y = &(P_CAMERA2[1]);
    void* P_CAMERA_temp2_z = &(P_CAMERA2[2]);
    double* P_CAMERA_check2_x{static_cast<double*>(P_CAMERA_temp2_x)};
    double* P_CAMERA_check2_y{static_cast<double*>(P_CAMERA_temp2_y)};
    double* P_CAMERA_check2_z{static_cast<double*>(P_CAMERA_temp2_z)};

    T pixel_projected1[2];
    (*compute_projection)(P_CAMERA_const1, &(pixel_projected1[0]));

    T pixel_projected2[2];
    (*compute_projection)(P_CAMERA_const2, &(pixel_projected2[0]));

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
    d1[0] = pixel_detected_.cast<T>()[0] - pixel_projected1[0];
    d1[1] = pixel_detected_.cast<T>()[1] - pixel_projected1[1];
    d1[2] = (T)0;

    d2[0] = pixel_detected_.cast<T>()[0] - pixel_projected2[0];
    d2[1] = pixel_detected_.cast<T>()[0] - pixel_projected2[1];
    d2[2] = (T)0;


    d12[0] = pixel_projected1[0] - pixel_projected2[0];
    d12[1] = pixel_projected1[1] - pixel_projected2[1];
    d12[2] = (T)0;


    T cross[3];
    ceres::CrossProduct(d1, d2, cross);

    T norm =
        sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
    T norm12 = sqrt(d12[0] * d12[0] + d12[1] * d12[1] + d12[2] * d12[2]);

    residuals[0] = norm / norm12;

    // Check if either projection is outside the domain of the camera model
    Eigen::Vector3d P_CAMERA_eig_check1{*P_CAMERA_check1_x, *P_CAMERA_check1_y,
                                       *P_CAMERA_check1_z};

    Eigen::Vector3d P_CAMERA_eig_check2{*P_CAMERA_check2_x, *P_CAMERA_check2_y,
                                       *P_CAMERA_check2_z};
    
    bool outside_domain = false;
    cad_image_markup::opt<Eigen::Vector2d> pixel_projected_check1 =
        camera_model_->ProjectPointPrecise(P_CAMERA_eig_check1, outside_domain);

    cad_image_markup::opt<Eigen::Vector2d> pixel_projected_check2 =
        camera_model_->ProjectPointPrecise(P_CAMERA_eig_check2, outside_domain);

    return !outside_domain;

  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(
      const Eigen::Vector2d pixel_detected, const Eigen::Vector3d P_STRUCT1,
      const Eigen::Vector3d P_STRUCT2,
      const std::shared_ptr<cad_image_markup::CameraModel> camera_model) {
    return (
        new ceres::AutoDiffCostFunction<CeresReprojectionCostFunction, 2, 7>(
            new CeresReprojectionCostFunction(pixel_detected, P_STRUCT1, P_STRUCT2,
                                              camera_model)));
  }

  Eigen::Vector2d pixel_detected_;
  Eigen::Vector3d P_STRUCT1_;
  Eigen::Vector3d P_STRUCT2_;
  std::shared_ptr<cad_image_markup::CameraModel> camera_model_;
  std::unique_ptr<ceres::CostFunctionToFunctor<2, 3>> compute_projection;

};

} // point_to_line_cost