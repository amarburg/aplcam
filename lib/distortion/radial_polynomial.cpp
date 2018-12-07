//
// Refactoring the  distortion model, starting with the RadialPolynomial model.
// Based on a heavily hacked version of fisheye.cpp from .
//


#include "AplCam/distortion/radial_polynomial.h"

#include <math.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <libg3logger/g3logger.h>
#include <iostream>
using namespace std;


namespace Distortion {

  using namespace cv;
  using namespace std;

  RadialPolynomial::RadialPolynomial( void )
  : DistortionModel(),  _distCoeffs( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 )
  {;}

  RadialPolynomial::RadialPolynomial( const Vec4d &d )
  : DistortionModel(),  _distCoeffs( d[0], d[1], d[2], d[3], 0., 0., 0., 0. )
  {;}

  RadialPolynomial::RadialPolynomial( const Vec5d &d )
  : DistortionModel(),  _distCoeffs( d[0], d[1], d[2], d[3], d[4], 0., 0., 0. )
  {;}

  RadialPolynomial::RadialPolynomial( const Vec8d &distCoeffs )
  : DistortionModel(),  _distCoeffs( distCoeffs )
  {;}

  RadialPolynomial::RadialPolynomial( const Vec12d &coeffs)
  : DistortionModel( Vec4d( coeffs[0], coeffs[1], coeffs[2], coeffs[3] ) ),
  _distCoeffs( Vec8d( coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8],
    coeffs[9], coeffs[10], coeffs[11] ) )
    {;}


    RadialPolynomial::RadialPolynomial( const Vec4d &d, const Matx33d &cam )
    : DistortionModel( cam ),    _distCoeffs( d[0], d[1], d[2], d[3], 0., 0., 0., 0. )
    {;}

    RadialPolynomial::RadialPolynomial( const Vec5d &d, const Matx33d &cam )
    : DistortionModel( cam ),    _distCoeffs( d[0], d[1], d[2], d[3], d[4], 0., 0., 0. )
    {;}

    RadialPolynomial::RadialPolynomial( const Vec8d &distCoeffs, const Matx33d &cam )
    : DistortionModel( cam ),    _distCoeffs( distCoeffs )
    {;}

    //--- Accessor functions ----
    Mat RadialPolynomial::coefficientsMat( void ) const
    {
      Mat m = (cv::Mat_<double>(12,1) << _fx, _fy, _cx, _cy,
      _distCoeffs[0], _distCoeffs[1], _distCoeffs[2], _distCoeffs[3],
      _distCoeffs[4], _distCoeffs[5], _distCoeffs[6], _distCoeffs[7]);
      return m;
    }




    //  // Static version uses a reasonable estimate based on image size
    //  RadialPolynomial RadialPolynomial::Calibrate(
    //      const ObjectPointsVecVec &objectPoints,
    //      const ImagePointsVecVec &imagePoints,
    //      const Size& image_size,
    //      vector< Vec3d > &rvecs,
    //      vector< Vec3d > &tvecs,
    //      int flags,
    //      cv::TermCriteria criteria)
    //  {
    //    RadialPolynomial fe( InitialDistortionEstimate(), InitialCameraEstimate( image_size ) );
    //    fe.calibrate( objectPoints, imagePoints, image_size, rvecs, tvecs, flags, criteria );
    //    return fe;
    //  }


    bool RadialPolynomial::doCalibrate(
      const ObjectPointsVecVec &objectPoints,
      const ImagePointsVecVec &imagePoints,
      const Size& image_size,
      CalibrationResult &result,
      int flags,
      cv::TermCriteria criteria)
      {
        LOG(WARNING) << "RadialPolynomial can't calibrate.  Use CeresRadialPolynomial or OpencvRadialPolynomial.";
        return false;
      }

      void RadialPolynomial::projectPoints( const ObjectPointsVec &objectPoints,
        const Vec3d &rvec, const Vec3d &tvec, ImagePointsVec &imagePoints ) const
        {

          cv::projectPoints( objectPoints, rvec, tvec, mat(), Mat( _distCoeffs ), imagePoints );
        }


        ImagePoint RadialPolynomial::distort( const ObjectPoint &w ) const
        {
          double xp = w[0]/w[2], yp = w[1]/w[2];
          double r2 = xp*xp + yp*yp;
          double r4 = r2*r2;
          double r6 = r2*r4;

          const double &k1( _distCoeffs[0]), &k2(_distCoeffs[1]), &p1(_distCoeffs[2]),
          &p2(_distCoeffs[3]), &k3(_distCoeffs[4]), &k4(_distCoeffs[5]),
          &k5(_distCoeffs[6]), &k6(_distCoeffs[7]);

          double pp[2];
          pp[0] = xp * ( 1 + k1*r2 + k2*r4 + k3*r6 ) / ( 1 + k4*r2 + k5*r4 + k6*r6 ) + 2*p1*xp*yp + p2*(r2 + 2*xp*xp);
          pp[1] = yp * ( 1 + k1*r2 + k2*r4 + k3*r6 ) / ( 1 + k4*r2 + k5*r4 + k6*r6 ) + p1*(r2 + 2*yp*yp) + 2*p2*xp*yp;

          return ImagePoint( pp[0], pp[1] );


        }

        ImagePoint RadialPolynomial::undistort( const ImagePoint &pw ) const
        {
          vector<ImagePoint> inVec, outVec(1);
          inVec.push_back(pw);

          cv::undistortPoints( inVec, outVec, Mat::eye(3,3,CV_64F), Mat(_distCoeffs) );

          return outVec.front();
        }




        DistortionModel *RadialPolynomial::estimateMeanCamera( vector< DistortionModel *> cameras )
        {
          Mat mean( Mat::zeros(12,1,CV_64F) );
          for( size_t i = 0; i < cameras.size(); ++i ) {
            assert( cameras[i]->name() == name() );
            mean += cameras[i]->coefficientsMat();
          }

          mean /= cameras.size();

          Vec12d vecCoeff;
          mean.copyTo( vecCoeff );

          return new RadialPolynomial( vecCoeff );
        }

        //--- Serialize/Unserialize functions

        RadialPolynomial *RadialPolynomial::Load( cv::FileStorage &in )
        {
          Mat kmat, distmat;
          Vec8d dist;

          in["camera_matrix"] >> kmat;
          in["distortion_coefficients"] >> dist;

          Matx33d k;

          kmat.convertTo( k, CV_64F );
          //distmat.copyTo( dist, CV_64F );)

          return new RadialPolynomial( dist, k );

        }


        FileStorage &RadialPolynomial::write( FileStorage &out ) const
        {
          DistortionModel::write( out );
          out << "distortion_coefficients" << _distCoeffs;

          return out;
        }

        void RadialPolynomial::to_json( json &j ) const {
          PinholeCamera::to_json(j);
          j["distortion_coefficients"] = { _distCoeffs[0], _distCoeffs[1], _distCoeffs[2], _distCoeffs[3], _distCoeffs[4], _distCoeffs[5], _distCoeffs[6], _distCoeffs[7] };
        }



      }
