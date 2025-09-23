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
 */
class CamEqui : public CamBase {

public:
    /**
     * @brief Constructor
     */
    CamEqui(int width, int height, const std::vector<double> &camera_k, const std::vector<double> &camera_d, double scale = 1)
        : CamBase(width, height, camera_k, camera_d, scale),
          fx_(camera_k[0] * scale), fy_(camera_k[1] * scale), cx_(camera_k[2] * scale), cy_(camera_k[3] * scale),
          k1_(camera_d[0]), k2_(camera_d[1]), k3_(camera_d[2]), k4_(camera_d[3])
    {
#if defined(HAVE_OPENCV)
        camera_k_OPENCV(0, 0) = fx_;
        camera_k_OPENCV(0, 1) = 0;
        camera_k_OPENCV(0, 2) = cx_;
        camera_k_OPENCV(1, 0) = 0;
        camera_k_OPENCV(1, 1) = fy_;
        camera_k_OPENCV(1, 2) = cy_;
        camera_k_OPENCV(2, 0) = 0;
        camera_k_OPENCV(2, 1) = 0;
        camera_k_OPENCV(2, 2) = 1;

        camera_d_OPENCV(0) = k1_;
        camera_d_OPENCV(1) = k2_;
        camera_d_OPENCV(2) = k3_;
        camera_d_OPENCV(3) = k4_;

        // Precompute undistortion maps for better performance
        cv::fisheye::initUndistortRectifyMap(camera_k_OPENCV, camera_d_OPENCV, cv::Mat(),
                                             camera_k_OPENCV, cv::Size(width_, height_),
                                             CV_32FC1, map1_, map2_);
#endif
    }

  ~CamEqui() override = default;

  /**
   * @brief Undistorts a raw pixel coordinate into normalized camera coordinates.
   */
  Eigen::Vector2d undistort_d(const Eigen::Vector2d &uv_dist) override {
#if defined(USE_OPENCV_IMPL) && defined(HAVE_OPENCV)
    cv::Mat mat(1, 1, CV_64FC2, const_cast<double*>(uv_dist.data()));
    cv::Mat mat_undistorted(1, 1, CV_64FC2);
    cv::fisheye::undistortPoints(mat, mat_undistorted, camera_k_OPENCV, camera_d_OPENCV);
    return Eigen::Vector2d(mat_undistorted.at<cv::Vec2d>(0,0)[0], mat_undistorted.at<cv::Vec2d>(0,0)[1]);
#else
    Eigen::Vector2d pt_distorted_normalized;
    pt_distorted_normalized.x() = (uv_dist.x() - cx_) / fx_;
    pt_distorted_normalized.y() = (uv_dist.y() - cy_) / fy_;

    double rd = pt_distorted_normalized.norm();
    double theta = rd;

    for (int i = 0; i < 10; i++) {
        double theta2 = theta * theta;
        double theta4 = theta2 * theta2;
        double theta6 = theta4 * theta2;
        double theta8 = theta4 * theta4;
        double theta_d = theta * (1.0 + k1_ * theta2 + k2_ * theta4 + k3_ * theta6 + k4_ * theta8);
        double error = theta_d - rd;
        double J = 1.0 + 3.0 * k1_ * theta2 + 5.0 * k2_ * theta4 + 7.0 * k3_ * theta6 + 9.0 * k4_ * theta8;
        theta -= error / J;
        if (std::abs(error) < 1e-9) break;
    }
    
    double ru = std::tan(theta);
    double scale = (rd > 1e-9) ? ru / rd : 1.0;
    return pt_distorted_normalized * scale;
#endif
  }

  /**
   * @brief Distorts a normalized camera coordinate to a raw pixel coordinate.
   */
  Eigen::Vector2d distort_d(const Eigen::Vector2d &uv_norm) override {
    double r = uv_norm.norm();
    if (r < 1e-9) return Eigen::Vector2d(cx_, cy_);
    
    double theta = std::atan(r);
    double theta2 = theta * theta;
    double theta4 = theta2 * theta2;
    double theta6 = theta4 * theta2;
    double theta8 = theta4 * theta4;
    double theta_d = theta * (1.0 + k1_ * theta2 + k2_ * theta4 + k3_ * theta6 + k4_ * theta8);
    double cdist = theta_d / r;

    return Eigen::Vector2d(fx_ * (cdist * uv_norm.x()) + cx_, fy_ * (cdist * uv_norm.y()) + cy_);
  }

#if defined(HAVE_OPENCV)
  /**
   * @brief Undistorts an entire image using OpenCV
   * @param distorted_image Input distorted image
   * @return Undistorted image
   */
  cv::Mat undistort_image(const cv::Mat& distorted_image) override {
    // Use precomputed undistortion maps for better performance
    cv::Mat undistorted_image;
    cv::remap(distorted_image, undistorted_image, map1_, map2_, cv::INTER_LINEAR);
    return undistorted_image;
  }
#endif

private:
  double fx_, fy_, cx_, cy_;
  double k1_, k2_, k3_, k4_;

#if defined(HAVE_OPENCV)
  cv::Matx33d camera_k_OPENCV;
  cv::Vec4d camera_d_OPENCV;
  cv::Mat map1_, map2_;  // Precomputed undistortion maps
#endif
};

#endif /* CAM_EQUI_H */
