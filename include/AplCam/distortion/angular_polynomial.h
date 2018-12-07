
#pragma once

#include "AplCam/cv_types.h"
#include "AplCam/distortion/distortion_model.h"

namespace Distortion {

  using cv::Vec4d;
  using cv::Vec8d;


  // Inplements the angular distortion model of ...
  class AngularPolynomial : public DistortionModel {
    public:

      static const Vec4d ZeroDistortion;

      AngularPolynomial( void );
      AngularPolynomial( const Vec4d &distCoeffs );
      AngularPolynomial( const Vec8d &coeffs );
      AngularPolynomial( const Vec4d &distCoeffs, const Matx33d &cam );

      static const std::string Name( void ) { return "AngularPolynomial"; }
      virtual const std::string name( void ) const { return AngularPolynomial::Name(); }

//      void set(const cv::Vec2d& f, const cv::Vec2d& c, const double &alpha = 0, const cv::Vec4d& k = ZeroDistortion );
//      void set( const double *c, const double alpha );

      //virtual cv::Mat distortionCoeffs( void ) const;

      //Vec4d distCoeffs( void ) const;

      virtual Mat coefficientsMat( void ) const;

      //static AngularPolynomial Calibrate( const ObjectPointsVecVec &objectPoints,
      //    const ImagePointsVecVec &imagePoints, const Size& image_size,
      //    vector< Vec3d > &rvecs,
      //    vector< Vec3d > &tvecs,
      //    int flags = 0,
      //    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );


      virtual ImagePoint undistort( const ImagePoint &pw ) const;

      // As I'm doing a Ceres-based undistortion, it's cheaper
      // to handle a whole vector as a chunk rather than iterating
      virtual ImagePointsVec undistort( const ImagePointsVec &pw ) const;

      virtual ImagePoint distort( const ObjectPoint &w ) const;

      virtual DistortionModel *estimateMeanCamera( vector< DistortionModel *> cameras );



      // --- Serialize/unserialize functions ---
      virtual cv::FileStorage &write( cv::FileStorage &out ) const;
      static AngularPolynomial *Load( cv::FileStorage &in );

      virtual void to_json( json &j ) const;



    protected:

      virtual bool doCalibrate( const ObjectPointsVecVec &objectPoints,
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          CalibrationResult &result,
          int flags = 0,
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );


      //static Matx33d InitialCameraEstimate( const Size &image_size );

      cv::Vec4d _distCoeffs;

  };

}
