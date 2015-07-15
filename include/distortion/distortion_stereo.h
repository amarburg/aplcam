
#ifndef __DISTORTION_STEREO_H__
#define __DISTORTION_STEREO_H__

#include <opencv2/core/core.hpp>

#include "types.h"
#include "distortion_model.h"

#include "stereo_calibration.h"

namespace Distortion {

  using namespace AplCam;

  using cv::Size;
  using cv::OutputArray;

  using cv::Mat;
  using cv::Rect;

  double stereoCalibrate( ObjectPointsVecVec _objectPoints,
      ImagePointsVecVec _imagePoints1,
      ImagePointsVecVec _imagePoints2,
      PinholeCamera &cam1, PinholeCamera &cam2,
      Size imageSize, OutputArray _Rmat, OutputArray _Tmat,
      OutputArray _Emat, OutputArray _Fmat,
      cv::TermCriteria criteria,
      int flags );

  void stereoRectify( const PinholeCamera &cam1, const PinholeCamera &cam2,
                          const Size &imageSize, const Mat &_Rmat, const Mat &_Tmat,
                          Mat &_Rmat1, Mat &_Rmat2,
                          Mat &_Pmat1, Mat &_Pmat2,
                          Mat &_Qmat, int flags,
                          double alpha, const Size &newImageSize,
                          Rect &validPixROI1, Rect &validPixROI2 );

  bool triangulate( const PinholeCamera &cam1, const PinholeCamera &cam2,
                   const StereoCalibration &calib,
                   ImagePointsVec &_imagePoints1,
                   ImagePointsVec &_imagePoints2,
                   ObjectPointsVec &_worldPoints );

   //void stereoRectify( const PinholeCamera &cam1, const PinholeCamera &cam2,
   //                       const Size &imageSize, const Mat &_Rmat, const Mat &_Tmat,
   //                       Mat &_Rmat1, Mat &_Rmat2,
   //                       Mat &_Pmat1, Mat &_Pmat2,
   //                       Mat &_Qmat, int flags,
   //                       double alpha, const Size &newImageSize )
   //{ Rect roi1, roi2;
   //  stereoRectify( cam1, cam2, imageSize, _Rmat, _Tmat, _Rmat1, _Rmat2, _Pmat1, _Pmat2, _Qmat, flags, alpha, newImageSize, roi1, roi2 ); }


}

#endif
