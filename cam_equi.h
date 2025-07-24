/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CAM_EQUI_H
#define CAM_EQUI_H

#include "cam_base.h"

/**
 * @brief Fisheye / equidistant pinhole camera model class
 *
 * This class implements the equidistant (fisheye) distortion model.
 * It is equivalent to Kalibr's `pinhole-equi` model and OpenCV's `cv::fisheye` model.
 *
 * The distortion function is:
 * \f{align*}{ 
 * \theta_d &= \theta (1 + k_1 \theta^2 + k_2 \theta^4 + k_3 \theta^6 + k_4 \theta^8) \\
 * x_d &= x_n \frac{\theta_d}{r} \\
 * y_d &= y_n \frac{\theta_d}{r}
 * \f}
 * where \f$ r = \sqrt{x_n^2 + y_n^2} \f$ and \f$ \theta = atan(r) \f$.
 *
 * The camera intrinsics are \f$ [f_x, f_y, c_x, c_y] \f$ and distortion coefficients are \f$ [k_1, k_2, k_3, k_4] \f$.
 */
class CamEqui : public CamBase {

public:
  /**
   * @brief Constructor
   * @param width Width of the camera (raw pixels)
   * @param height Height of the camera (raw pixels)
   * @param camera_k Intrinsics of the camera [fx, fy, cx, cy]
   * @param camera_d Distortion coefficients [k1, k2, k3, k4]
   */
  CamEqui(int width, int height, const std::vector<double> &camera_k, const std::vector<double> &camera_d)
    : CamBase(width, height, camera_k, camera_d) {
    fx_ = camera_k[0];
    fy_ = camera_k[1];
    cx_ = camera_k[2];
    cy_ = camera_k[3];

    k1_ = camera_d[0];
    k2_ = camera_d[1];
    k3_ = camera_d[2];
    k4_ = camera_d[3];

#ifdef USE_OPENCV_IMPL
    // Camera matrix
    cv::Matx33d tempK;
    tempK(0, 0) = camera_k_[0];
    tempK(0, 1) = 0;
    tempK(0, 2) = camera_k_[2];
    tempK(1, 0) = 0;
    tempK(1, 1) = camera_k_[1];
    tempK(1, 2) = camera_k_[3];
    tempK(2, 0) = 0;
    tempK(2, 1) = 0;
    tempK(2, 2) = 1;
    camera_k_OPENCV = tempK;

    // Distortion parameters
    cv::Vec4d tempD;
    tempD(0) = camera_d_[0];
    tempD(1) = camera_d_[1];
    tempD(2) = camera_d_[2];
    tempD(3) = camera_d_[3];
    camera_d_OPENCV = tempD;
#endif
  }

  ~CamEqui() override = default;

  /**
   * @brief Undistorts a raw pixel coordinate into normalized camera coordinates.
   * This function iteratively inverts the fisheye distortion model.
   * @param uv_dist Raw (distorted) uv coordinate
   * @return Normalized (undistorted) 2D coordinate
   */
  Eigen::Vector2d undistort_d(const Eigen::Vector2d &uv_dist) override {
#ifdef USE_OPENCV_IMPL
    // Convert to opencv format
    cv::Mat mat(1, 1, CV_64FC2);
    mat.at<cv::Vec2d>(0,0) = cv::Vec2d(uv_dist(0), uv_dist(1));

    // Undistort it using the fisheye model
    cv::Mat mat_undistorted(1, 1, CV_64FC2);
    cv::fisheye::undistortPoints(mat, mat_undistorted, camera_k_OPENCV, camera_d_OPENCV);

    // Construct our return vector
    Eigen::Vector2d pt_out;
    pt_out(0) = mat_undistorted.at<cv::Vec2d>(0,0)[0];
    pt_out(1) = mat_undistorted.at<cv::Vec2d>(0,0)[1];
    return pt_out;
#else
    // Convert from pixel to distorted normalized coordinates
    Eigen::Vector2d pt_distorted_normalized;
    pt_distorted_normalized.x() = (uv_dist.x() - cx_) / fx_;
    pt_distorted_normalized.y() = (uv_dist.y() - cy_) / fy_;

    // Get distorted radius
    double rd = pt_distorted_normalized.norm();

    // Initial guess for undistorted radius, theta
    double theta = rd;

    // Iteratively solve for the undistorted radius `ru`
    for (int i = 0; i < 10; i++) {
      double theta2 = theta * theta;
      double theta4 = theta2 * theta2;
      double theta6 = theta4 * theta2;
      double theta8 = theta4 * theta4;
      double theta_d = theta * (1.0 + k1_ * theta2 + k2_ * theta4 + k3_ * theta6 + k4_ * theta8);
      
      // Update theta using Newton's method
      double error = theta_d - rd;
      double J = 1.0 + 3.0 * k1_ * theta2 + 5.0 * k2_ * theta4 + 7.0 * k3_ * theta6 + 9.0 * k4_ * theta8;
      theta -= error / J;

      if (std::abs(error) < 1e-9) {
        break;
      }
    }
    double ru = std::tan(theta);

    // Scale the point to get the final undistorted normalized coordinates
    double scale = (rd > 1e-9) ? ru / rd : 1.0;
    return pt_distorted_normalized * scale;
#endif
  }

  /**
   * @brief Distorts a normalized camera coordinate to a raw pixel coordinate.
   * @param uv_norm Normalized (undistorted) 2D coordinate
   * @return Raw (distorted) uv coordinate
   */
  Eigen::Vector2d distort_d(const Eigen::Vector2d &uv_norm) override {
    // Calculate the undistorted radius and theta
    double r = uv_norm.norm();
    if (r < 1e-9) {
        return Eigen::Vector2d(cx_, cy_);
    }
    double theta = std::atan(r);

    // Calculate distorted theta
    double theta2 = theta * theta;
    double theta4 = theta2 * theta2;
    double theta6 = theta4 * theta2;
    double theta8 = theta4 * theta4;
    double theta_d = theta * (1.0 + k1_ * theta2 + k2_ * theta4 + k3_ * theta6 + k4_ * theta8);

    // Calculate the distortion scaling factor
    double cdist = theta_d / r;

    // Apply distortion and project to pixel coordinates
    Eigen::Vector2d uv_dist;
    uv_dist.x() = fx_ * (cdist * uv_norm.x()) + cx_;
    uv_dist.y() = fy_ * (cdist * uv_norm.y()) + cy_;
    return uv_dist;
  }

private:
  double fx_, fy_, cx_, cy_;
  double k1_, k2_, k3_, k4_;

#ifdef USE_OPENCV_IMPL
  cv::Matx33d camera_k_OPENCV;
  cv::Vec4d camera_d_OPENCV;
#endif
};

#endif /* CAM_EQUI_H */
