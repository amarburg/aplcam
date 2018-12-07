
#pragma once


#include <opencv2/core/core.hpp>

#include "AplCam/distortion/distortion_model.h"
#include "AplCam/cv_types.h"

namespace Distortion {

  using cv::Vec4d;
  using cv::Vec5d;
  using cv::Vec8d;
  using cv::Vec12d;

  // Radial model aka "The OpenCV model"
  class RadialPolynomial : public DistortionModel {
  public:

    RadialPolynomial( void );
    RadialPolynomial( const Vec8d &distCoeffs );
    RadialPolynomial( const Vec5d &distCoeffs );
    RadialPolynomial( const Vec4d &distCoeffs );
    RadialPolynomial( const Vec8d &distCoeffs, const Matx33d &cam );
    RadialPolynomial( const Vec5d &distCoeffs, const Matx33d &cam );
    RadialPolynomial( const Vec4d &distCoeffs, const Matx33d &cam );
    RadialPolynomial( const Vec12d &coeffs );

    static const std::string Name( void ) { return "RadialPolynomial"; }
    virtual const std::string name( void ) const { return RadialPolynomial::Name(); }

  //  virtual cv::Mat distortionCoeffs( void ) const
  //  {
  //    cv::Mat m = (cv::Mat_<double>(8,1) << _distCoeffs[0], _distCoeffs[1], _distCoeffs[2], _distCoeffs[3],
  //                 _distCoeffs[4], _distCoeffs[5], _distCoeffs[6], _distCoeffs[7]);
  //    return m;
  //  }

  virtual Mat coefficientsMat( void ) const;


    virtual void projectPoints( const ObjectPointsVec &objectPoints,
                               const Vec3d &_rvec, const Vec3d &_tvec, ImagePointsVec &imagePoints ) const;

    virtual ImagePoint undistort( const ImagePoint &pw ) const;
    virtual ImagePoint distort( const ObjectPoint &w ) const ;

    virtual DistortionModel *estimateMeanCamera( vector< DistortionModel *> cameras );

    //--- Serialize/Unserialize functions ----
    static RadialPolynomial *Load( cv::FileStorage &in );
    virtual cv::FileStorage &write( cv::FileStorage &out ) const;
    virtual void to_json( json &j ) const;



  protected:

    virtual bool doCalibrate( const ObjectPointsVecVec &objectPoints,
                             const ImagePointsVecVec &imagePoints, const Size& image_size,
                             CalibrationResult &result,
                             int flags = 0,
                             cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

    static const Vec8d InitialDistortionEstimate( void )
    { return Vec8d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

    cv::Vec8d _distCoeffs;

  };

  class OpencvRadialPolynomial : public RadialPolynomial {
   public:

    static const std::string Name( void ) { return "OpencvRadialPolynomial"; }
    virtual const std::string name( void ) const { return OpencvRadialPolynomial::Name(); }

   protected:

    virtual bool doCalibrate( const ObjectPointsVecVec &objectPoints,
                             const ImagePointsVecVec &imagePoints, const Size& image_size,
                             CalibrationResult &result,
                             int flags = 0,
                             cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );


  };

  class CeresRadialPolynomial : public RadialPolynomial {
   public:

    static const std::string Name( void ) { return "CeresRadialPolynomial"; }
    virtual const std::string name( void ) const { return CeresRadialPolynomial::Name(); }


   protected:

    virtual bool doCalibrate( const ObjectPointsVecVec &objectPoints,
                             const ImagePointsVecVec &imagePoints, const Size& image_size,
                             CalibrationResult &result,
                             int flags = 0,
                             cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

  };




}
