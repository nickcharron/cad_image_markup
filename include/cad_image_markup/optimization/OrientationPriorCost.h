#pragma once

#include <Eigen/Core>
#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>

/**
 * @brief Create a prior cost function on a 3D orientation variable (quaternion)
 *
 * The cost function is of the form:
 *
 *             ||                         ||^2
 *   cost(x) = || A * AngleAxis(b^-1 * q) ||
 *             ||                         ||
 *
 * where the matrix A and the vector b are fixed, and q is the variable being
 * measured, represented as a quaternion. The AngleAxis function converts a
 * quaternion into a 3-vector of the form theta*k, where k is the unit vector
 * axis of rotation and theta is the angle of rotation. The A matrix is applied
 * to the angle-axis 3-vector.
 *
 * In case the user is interested in implementing a cost function of the form
 *
 *   cost(X) = (X - mu)^T S^{-1} (X - mu)
 *
 * where, mu is a vector and S is a covariance matrix, then, A = S^{-1/2}, i.e
 * the matrix A is the square root information matrix (the inverse of the
 * covariance).
 */
struct OrientationPriorFunctor {
  OrientationPriorFunctor(const Eigen::Matrix3d& sqrt_inv_cov,
                          const Eigen::Vector4d& orientation_wxyz)
      : A_(sqrt_inv_cov), b_(orientation_wxyz) {}

  template <typename T>
  bool operator()(const T* const orientation_wxyz, T* residuals) const {
    // Compute the delta quaternion
    T variable[4] = {orientation_wxyz[0], orientation_wxyz[1],
                     orientation_wxyz[2], orientation_wxyz[3]};

    T observation_inverse[4] = {T(b_(0)), T(-b_(1)), T(-b_(2)), T(-b_(3))};

    T difference[4];
    ceres::QuaternionProduct(observation_inverse, variable, difference);
    ceres::QuaternionToAngleAxis(difference, residuals);

    // Scale the residuals by the square root information matrix to account for
    // the measurement uncertainty.
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals);
    residuals_map.applyOnTheLeft(A_.template cast<T>());

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const Eigen::Matrix3d& sqrt_inv_cov_weight,
                                     const Eigen::Vector4d& orientation_wxyz) {
    return (new ceres::AutoDiffCostFunction<OrientationPriorFunctor, 3, 7>(
        new OrientationPriorFunctor(sqrt_inv_cov_weight, orientation_wxyz)));
  }

  Eigen::Matrix3d A_; // The residual weighting matrix, most likely the square
                      // root information matrix
  Eigen::Vector4d b_; // The measured 3D orientation (quaternion) value
};
