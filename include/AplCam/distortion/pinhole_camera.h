
#ifndef __PINHOLE_CAMERA_H__
#define __PINHOLE_CAMERA_H__

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <vector>
#include <algorithm>

#include "AplCam/types.h"
#include "AplCam/calibration_result.h"

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

  // For later ... it's all done double precision for now.  Not necessary.

  enum {
    CALIB_HUBER_LOSS = cv::CALIB_ZERO_DISPARITY << 1
  };



  class Camera {
    public:


      virtual ~Camera() {;}

      virtual const std::string name( void ) const = 0;
      virtual cv::FileStorage &write( cv::FileStorage &out ) const = 0;

      // OpenCV-like API.  The other version returns a CalibrationResult
      // with more information.
      double calibrate( const ObjectPointsVecVec &objectPoints,
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          vector< Vec3d > &rvecs,
          vector< Vec3d > &tvecs,
          int flags = 0,
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );


      // This does the "prep work", then doCalibrate does the "dirty work" for each distortion model
      bool calibrate( const ObjectPointsVecVec &objectPoints,
          const ImagePointsVecVec &imagePoints,
          const Size& image_size,
          CalibrationResult &result,
          int flags = 0,
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

      virtual void projectPoints( const ObjectPointsVec &objectPoints,
          const Vec3d &_rvec, const Vec3d &_tvec, ImagePointsVec &imagePoints ) const = 0;


      virtual Mat coefficientsMat( void ) const = 0;


      void imageSizeHint( const cv::Size &sz ) { _imgSizeHint = sz; }
      void focalLengthHint( float f ) { _focalLengthHint = f; }

    protected:

      virtual bool doCalibrate( const ObjectPointsVecVec &objectPoints,
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          CalibrationResult &result,
          int flags = 0,
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  ) { return false; };

      // Private constructor
      Camera()
          : _imgSizeHint(1920,1080), _focalLengthHint(600) {;}

      cv::Size _imgSizeHint;
      float _focalLengthHint;
  };

  class PinholeCamera : public Camera {
    public:

      // Must be equal to OpenCV's to avoid nasty conversions
      //     enum{
      //       CALIB_FIX_SKEW              =
      //       CALIB_RECOMPUTE_EXTRINSIC   = 2,
      //       CALIB_CHECK_COND            = 4,
      //       CALIB_FIX_SKEW              = 8,
      //       CALIB_FIX_K1                = 16,
      //       CALIB_FIX_K2                = 32,
      //       CALIB_FIX_K3                = 64,
      //       CALIB_FIX_K4                = 128,
      //       CALIB_FIX_INTRINSIC         = 256
      //     };

      PinholeCamera( void );
      PinholeCamera( const Matx33d &k );
      PinholeCamera( const Mat &k );
      PinholeCamera( const Vec4d &coeffs );

      virtual ~PinholeCamera() {;}

      virtual const std::string name( void ) const { return "pinhole"; }

      //--- Accessor functions ----
      void setCamera( const Matx33d &k );
      void setCamera( double fx, double fy, double cx, double cy, double alpha = 0 );
      void setCamera( const double *c, double alpha );

      virtual Matx33d matx( void ) const;
      virtual Mat mat( void ) const;

      Vec2d  f( void ) const      { return Vec2d( _fx, _fy ); }
      double favg( void ) const   { return (_fx + _fy)/2.0; }
      double fx( void ) const     { return _fx; }
      double fy( void ) const     { return _fy; }
      Vec2d  c( void ) const       { return Vec2d(_cx,_cy); }
      double cx( void ) const     { return _cx; }
      double cy( void ) const     { return _cy; }
      double alpha( void) const   { return _alpha; }

      virtual Mat coefficientsMat( void ) const
      {
        Mat m = (cv::Mat_<double>(4,1) << _fx, _fy, _cx, _cy);
        return m;
      }

      //---- Serialize/Unserialize functions ----
      virtual cv::FileStorage &write( cv::FileStorage &out ) const;

      virtual void to_json( json &j ) const;



      Mat getOptimalNewCameraMatrix( const Size &imgSize, double alpha,
          const Size &newImgSize, cv::Rect &validPixROI, bool centerPrincipalPoint = false );

      Mat getOptimalNewCameraMatrix( const Size &imgSize, double alpha,
          const Size &newImgSize, bool centerPrincipalPoint = false );

      //--- OpenCV-ish functions ----

      virtual void projectPoint( const ObjectPoint &objPt, const Vec3d &rvec, const Vec3d &tvec, ImagePoint &imgPt ) const;

      virtual void projectPoints( const ObjectPointsVec &objectPoints,
          const Vec3d &_rvec, const Vec3d &_tvec, ImagePointsVec &imagePoints  ) const;

      // Basically a clone of cv::undistortPoints but doesn't
      // require the camera matrix or distortions.
      // Performs normalization, undistortion.
      // THen optionally re-normalization and rectification with R  & P
      virtual void undistortPoints( const ImagePointsVec &distorted,
          ImagePointsVec &undistorted,
          const Mat &R = cv::Mat::eye(3,3,CV_64F),
          const Mat &P = cv::Mat()) const;


      //        struct TxVecNormalizerUndistorter {
      //          TxVecNormalizerUndistorter( const PinholeCamera &cam ) : _cam(cam) {;}
      //          const PinholeCamera &_cam;
      //
      //          ImagePointsVec operator()( const ImagePointsVec &vec )
      //          { ImagePointsVec out = _cam.normalize( vec );
      //            return _cam.undistort(  out ); }
      //        };
      //        TxVecNormalizerUndistorter makeVecNormalizerUndistorter( void ) const { return TxVecNormalizerUndistorter( *this ); }

      //---- Normalize/image functions ----
      virtual ImagePoint image( const ImagePoint &pt ) const;
      virtual ImagePointsVec image( const ImagePointsVec &vec ) const;

      virtual ImagePoint normalize( const ImagePoint &pt ) const;
      virtual ImagePointsVec normalize( const ImagePointsVec &vec ) const;

      // Transformation functors and factory functions
      struct ImagePointFunctor {
        public:
          ImagePointFunctor( const PinholeCamera &cam ) : _cam(cam) {;}
          const PinholeCamera &_cam;

          virtual ImagePoint operator()( const ImagePoint &pt ) = 0;
      };

      struct NormalizeFunctor : public ImagePointFunctor {
        NormalizeFunctor( const PinholeCamera &cam ) : ImagePointFunctor(cam) {;}

        virtual ImagePoint operator()( const ImagePoint &pt )
        { return _cam.normalize( pt ); }
      };

      NormalizeFunctor makeNormalizer( void ) const { return NormalizeFunctor( *this ); }

      struct ImageFunctor : public ImagePointFunctor {
        ImageFunctor( const PinholeCamera &cam ) : ImagePointFunctor(cam) {;}

        virtual ImagePoint operator()( const ImagePoint &pt )
        { return _cam.image( pt ); }
      };

      ImageFunctor makeImager( void ) const { return ImageFunctor( *this ); }


      // --- Distortion functions ---
      //
      // Even though PinholeCamera technically doesn't Distort, this lets all of its
      // functions be "distortion aware".
      //
      // For the Pinhole camera, it's just the null distortion model.


      // Note these are strictly undistortion functions.
      // Input points __must__ be normalized
      virtual ImagePoint     undistort( const ImagePoint &pw ) const
      { return pw; }

      virtual ImagePointsVec undistortVec( const ImagePointsVec &pw ) const
      {
        ImagePointsVec out;
        for(size_t i = 0; i < pw.size(); ++i)
          out.push_back( undistort( pw[i] ));
        return out;
      }

      virtual ImagePoint distort( const ImagePoint &w ) const
      { return w; }

      virtual ImagePoint distort( const ObjectPoint &w ) const
      { return ImagePoint( w[0]/w[2], w[1]/w[2] ); }


      struct UndistortFunctor : public ImagePointFunctor {
        UndistortFunctor( const PinholeCamera &cam ) : ImagePointFunctor(cam) {;}

        virtual ImagePoint operator()( const ImagePoint &pt )
        { return _cam.undistort( pt ); }
      };

      UndistortFunctor makeUndistorter( void ) const { return UndistortFunctor( *this  ); }

      struct VecUndistortFunctor {
        VecUndistortFunctor( const PinholeCamera &cam )
          : _cam(cam)  {;}
        const PinholeCamera &_cam;

        ImagePointsVec operator()( const ImagePointsVec &vec )
        { return _cam.undistortVec( vec ); }
      };
      VecUndistortFunctor makeVecUndistorter( void ) const { return VecUndistortFunctor( *this ); }


      //--- Various combination functions ----
      // Could be optimized in derived classes
      virtual ImagePoint     normalizeUndistort( const ImagePoint &pw ) const
      { return undistort( normalize(pw) ); }

      virtual ImagePointsVec normalizeUndistort( const ImagePointsVec &pw ) const
      { return undistortVec( normalize(pw) ); }

      virtual ImagePoint     normalizeUndistortImage( const ImagePoint &pw ) const
      { return image( undistort( normalize(pw) ) ); }

      virtual ImagePointsVec normalizeUndistortImage( const ImagePointsVec &pw ) const
      { return image( undistortVec( normalize(pw) ) ); }

      virtual ImagePointsVecVec normalizeUndistortImage( const ImagePointsVecVec &pw ) const;

      virtual ImagePoint distortImage( const ObjectPoint &pw ) const
      { return image( distort( pw ) ); }


      void getRectangles( const Mat &R, const Mat &newCameraMatrix, const Size &imgSize,
          cv::Rect_<float>& inner, cv::Rect_<float>& outer ) const;


      //---- ReprojectionError ----
      double reprojectionError( const ObjectPointsVec &objPts,
          const Vec3d &rvec, const Vec3d &tvec,
          const ImagePointsVec &imgPts );

      double reprojectionError( const ObjectPointsVec &objPts,
          const Vec3d &rvec, const Vec3d &tvec,
          const ImagePointsVec &imgPts,
          ReprojErrorVec &reprojErrors );

      double reprojectionError( const ObjectPointsVecVec &obPtsj,
          const RotVec &rvecs, const TransVec &tvecs,
          const ImagePointsVecVec &imgPts,
          const vector<bool> mask = vector<bool>() );

      double reprojectionError( const ObjectPointsVecVec &obPtsj,
          const RotVec &rvecs, const TransVec &tvecs,
          const ImagePointsVecVec &imgPts,
          ReprojErrorVecVec &reprojErrors,
          const vector<bool> mask = vector<bool>() );




    protected:

      static Matx33d InitialCameraEstimate( const Size &image_size )
      {
        float fEstimate = std::max( image_size.width, image_size.height )/ CV_PI;
        return Matx33d( fEstimate, 0, image_size.width/2.0 - 0.5,
            0, fEstimate, image_size.height/2.0 - 0.5,
            0, 0, 1. );
      }

      double _fx, _fy, _alpha, _cx, _cy;
  };

}

#endif
