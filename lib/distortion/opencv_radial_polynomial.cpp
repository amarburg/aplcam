//
// Refactoring the OpenCV distortion model, starting with the OpencvRadialPolynomial model.
// Based on a heavily hacked version of fisheye.cpp from OpenCV.

#include <math.h>

#include "AplCam/distortion/radial_polynomial.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <iostream>
using namespace std;


namespace Distortion {

  using namespace cv;
  using namespace std;


  //  // Static version uses a reasonable estimate based on image size
  //  OpencvRadialPolynomial OpencvRadialPolynomial::Calibrate(
  //      const ObjectPointsVecVec &objectPoints,
  //      const ImagePointsVecVec &imagePoints,
  //      const Size& image_size,
  //      vector< Vec3d > &rvecs,
  //      vector< Vec3d > &tvecs,
  //      int flags,
  //      cv::TermCriteria criteria)
  //  {
  //    OpencvRadialPolynomial fe( InitialDistortionEstimate(), InitialCameraEstimate( image_size ) );
  //    fe.calibrate( objectPoints, imagePoints, image_size, rvecs, tvecs, flags, criteria );
  //    return fe;
  //  }


  bool OpencvRadialPolynomial::doCalibrate(
    const ObjectPointsVecVec &objectPoints,
    const ImagePointsVecVec &imagePoints,
    const Size& imageSize,
    CalibrationResult &result,
    int flags,
    cv::TermCriteria criteria)
    {

      Mat camera( mat() );
      Mat dist( _distCoeffs );

      vector<Mat> _rvecs, _tvecs;

      ObjectPointsVecVec _objPts;
      ImagePointsVecVec _imgPts;

      for( size_t i = 0; i < objectPoints.size(); ++i ) {
        if( result.status[i] ) {
          _objPts.push_back( objectPoints[i] );
          _imgPts.push_back( imagePoints[i] );
        }
      }

      //for( int i = 0; i < objectPoints.size(); ++i )
      //  cout << i << " " << objectPoints[i].size() << " " << imagePoints[i].size() << endl;

      int before = getTickCount();
      result.rms = calibrateCamera( _objPts, _imgPts, imageSize, camera, dist, _rvecs, _tvecs, flags, criteria );
      result.totalTime = (getTickCount() - before)/getTickFrequency();



      setCamera( camera );

      // _distCoeffs will be variable length
      double *d = dist.ptr<double>(0);

      for( unsigned int i = 0; i < 4; ++i ) _distCoeffs[i] = d[i];
      if( (dist.rows*dist.cols) == 5 )  _distCoeffs[4] = d[4];
      if( (dist.rows*dist.cols) == 8 ) {
        _distCoeffs[5] = d[5];
        _distCoeffs[6] = d[6];
        _distCoeffs[7] = d[7];
      }

      for( size_t i =0, k = 0; i < objectPoints.size(); ++i ) {
        if( result.status[i] ) {
          result.rvecs[i] = _rvecs[k];
          result.tvecs[i] = _tvecs[k];
          ++k;
        }
      }

      LOG(INFO) << "Camera:" << camera;
      LOG(INFO) << "Distortion coeffs: " << _distCoeffs;

      result.good = true;
      return result.good;
    }


  }
