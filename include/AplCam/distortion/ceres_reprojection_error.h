#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace Distortion {
  // Base class for AutoDiffCostFunction'able functors.   Provides
  // functions usable for all Ceres-based solvers.

  struct ReprojectionError {
    ReprojectionError(double obs_x, double obs_y, double world_x, double world_y )
    : observedX(obs_x), observedY(obs_y), worldX( world_x ), worldY( world_y ) {;}

    template <typename T>
    void txToCameraFrame( const T* const pose, T *p ) const
    {
      const T point[3] = { T( worldX ), T( worldY ), T( 0.0 ) };
      ceres::AngleAxisRotatePoint(pose, point, p);
      p[0] += pose[3];
      p[1] += pose[4];
      p[2] += pose[5];
    }

    template <typename T>
    void txQuaternionToCameraFrame( const T* const pose, T *p ) const
    {
      const T point[3] = { T( worldX ), T( worldY ), T( 0.0 ) };
      ceres::QuaternionRotatePoint(pose, point, p);
      p[0] += pose[4];
      p[1] += pose[5];
      p[2] += pose[6];
    }

    template <typename T>
    bool projectAndComputeError(const T* const camera,
      const T* const alpha,
      const T* const pp,
      T* residuals) const
      {
        const T &fx = camera[0];
        const T &fy = camera[1];
        const T &cx = camera[2];
        const T &cy = camera[3];
        const T &xpp = pp[0],
                &ypp = pp[1];

        T predictedX = fx*(xpp + alpha[0]*ypp) + cx;
        T predictedY = fy* ypp                 + cy;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predictedX - T(observedX);
        residuals[1] = predictedY - T(observedY);
        return true;
      }


      double observedX, observedY;
      double worldX, worldY;
    };

  }
