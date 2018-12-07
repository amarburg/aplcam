#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <vector>
#include <algorithm>

#include "AplCam/types.h"
#include "AplCam/calibration_result.h"

#include "AplCam/distortion/pinhole_camera.h"

namespace Distortion {

using namespace AplCam;

using std::vector;
using cv::Size;
using cv::Mat;

using cv::Vec2f;
using cv::Vec3f;
using cv::Point3f;
using cv::Point2f;

using cv::Vec2d;
using cv::Vec3d;
using cv::Vec4d;

using cv::Matx33d;

// TODO:  For later ... it's all done double precision for now.  Not necessary.

class DistortionModel : public PinholeCamera {

 public:

  DistortionModel( void )
      : PinholeCamera() {;}

  DistortionModel( const Matx33d &cam )
      : PinholeCamera( cam ) {;}

  DistortionModel( const Vec4d &coeffs )
      : PinholeCamera( coeffs )
  {;}

  virtual ~DistortionModel() {;}

  //---- Accessor functions ----

        //virtual Mat distortionCoeffs( void ) const { return Mat(); }

  //-- Undistortion functions --

  virtual void initUndistortRectifyMap( const Mat &R, const Mat &P,
                                       const cv::Size& size, int m1type, Mat &map1, Mat &map2 );

  void undistortImage( const Mat &distorted, Mat &undistorted,
                      const Mat &Knew, const Size& new_size);

  void undistortImage( const Mat &distorted, Mat &undistorted )
  { undistortImage( distorted, undistorted, mat(), distorted.size() ); }


    // This is actually meaningful in DistortionModel, the version in
    // PinholeCamera does nothing
   virtual ImagePointsVec undistortVec( const ImagePointsVec &pw ) const;

  //

  enum DistortionModelType_t { CALIBRATION_NONE,
                               ANGULAR_POLYNOMIAL,
                               RADIAL8_POLYNOMIAL,
                               CERES_RADIAL,
                               OPENCV_RADIAL };

  static std::string           ValidateModelName( const std::string &name );
  static DistortionModelType_t ParseDistortionModel( const std::string &arg );
  static DistortionModel      *MakeDistortionModel( const std::string &arg ) { return MakeDistortionModel( ParseDistortionModel( arg )); }
  static DistortionModel      *MakeDistortionModel( DistortionModelType_t type );


  virtual DistortionModel *estimateMeanCamera( vector< DistortionModel *> cameras ) = 0;





};

}
