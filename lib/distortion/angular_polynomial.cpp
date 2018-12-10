//
// Refactoring the OpenCV distortion model, starting with the AngularPolynomial model.
// Based on a heavily hacked version of fisheye.cpp from OpenCV.

#include <math.h>

#include "AplCam/distortion/angular_polynomial.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
// Neuter GoogleLogging, which is included by Ceres
#undef LOG
#undef LOG_IF
#undef CHECK

#include <boost/thread.hpp>

#include <iostream>
#include <iomanip>
using namespace std;


#include <libg3logger/g3logger.h>
#include "AplCam/distortion/ceres_reprojection_error.h"

namespace Distortion {

  using namespace cv;
  using namespace std;

  // Polynomial expansion from (pg 16):
  // http://www.research.scea.com/research/pdfs/RGREENfastermath_GDC02.pdf
  //
  const Vec4d AngularPolynomial::ZeroDistortion = Vec4d( 0.334961658, 0.118066350, 0.092151584, 0 );

  AngularPolynomial::AngularPolynomial( void )
    : DistortionModel(), _distCoeffs( ZeroDistortion )
  {;}

  AngularPolynomial::AngularPolynomial( const Vec4d &distCoeffs )
    : DistortionModel(), _distCoeffs( distCoeffs )
  {;}

  AngularPolynomial::AngularPolynomial( const Vec8d &coeffs )
    : DistortionModel( Vec4d( coeffs[0], coeffs[1], coeffs[2], coeffs[3] )),
      _distCoeffs( Vec4d( coeffs[4], coeffs[5], coeffs[6], coeffs[7] ) )
  {;}

  AngularPolynomial::AngularPolynomial( const Vec4d &distCoeffs, const Matx33d &cam )
    : DistortionModel( cam ), _distCoeffs( distCoeffs )
  {;}

  /// --- Accessor functions ----
  cv::Mat AngularPolynomial::coefficientsMat( void ) const
  {
    Mat m = (cv::Mat_<double>(12,1) << _fx, _fy, _cx, _cy,
        _distCoeffs[0], _distCoeffs[1], _distCoeffs[2], _distCoeffs[3] );
    return m;
  }

  // Ceres functor for solving calibration problem
  //  Based on the Bundler solver used in their examples
  struct AngularDistortionReprojError  : public ReprojectionError {
    AngularDistortionReprojError(double obs_x, double obs_y, double world_x, double world_y )
      : ReprojectionError( obs_x, obs_y, world_x, world_y )
    {;}

    template <typename T>
      bool operator()(const T* const camera,
          const T* const alpha,
          const T* const dist,
          const T* const pose,
          T* residuals) const
      {
        // pose is a 7-vector
        //    4 quaternion
        //    3 translations
        //
        // alpha is a 1-vector (separated out so it can be set Constant or Variable)
        //
        // camera i s 8-vector
        //    2 focal length
        //    2 camera center
        //    4 distortion params
        //
        // pose[0,1,2] are an angle-axis rotation.
        //

        // Rotate the world point into the camera frame as p
        T p[3];
        txQuaternionToCameraFrame( pose, p );

        T theta = atan2( sqrt( p[0]*p[0] + p[1]*p[1] ), p[2]  );
        T psi   = atan2( p[1], p[0] );

        //        const T &fx = camera[0];
        //        const T &fy = camera[1];
        //        const T &cx = camera[2];
        //        const T &cy = camera[3];
        const T &k1 = dist[0];
        const T &k2 = dist[1];
        const T &k3 = dist[2];
        const T &k4 = dist[3];

        T theta2 = theta *theta;
        T theta4 = theta2*theta2;
        T theta6 = theta4*theta2;
        T theta8 = theta4*theta4;

        T thetaDist = theta * ( T(1) + k1*theta2 + k2 *theta4 + k3*theta6 + k4*theta8);

        T pp[2];
        pp[0] = thetaDist * cos( psi );
        pp[1] = thetaDist * sin( psi );

        return projectAndComputeError( camera, alpha, pp, residuals );
      }

  };

  // struct AngularDistortionFactory {
  //   AngularDistortionFactory( double *camera, double *alpha, double *dist, ceres::LossFunction *lossF = NULL )
  //     : camera_(camera), alpha_(alpha), dist_(dist), lossFunc_( lossF )
  //   {;}
  //
  //   void addCorrespondence( ceres::Problem &problem, const ObjectPoint &obj, const ImagePoint &img, double *qpose )
  //   {
  //
  //
  //   }
  //
  //   double *camera_, *alpha_, *dist_;
  //   ceres::LossFunction *lossFunc_;
  // };



  bool AngularPolynomial::doCalibrate(
      const ObjectPointsVecVec &objectPoints,
      const ImagePointsVecVec &imagePoints,
      const Size& image_size,
      CalibrationResult &result,
      int flags,
      cv::TermCriteria criteria)
  {

    // If camera matrix is unset, get a default from the focal length and img size hints
    if( norm( matx(), Mat::eye(3,3,CV_64F) ) < 1e-9 ) {
      //setCamera( InitialCameraEstimate( _imageSizeHint ) );
      setCamera( _focalLengthHint, _focalLengthHint, _imgSizeHint.width/2.0, _imgSizeHint.height/2.0, 0 );

      LOG(INFO) << "Setting initial camera estimate with fx = " << _fx << ", fy = " << _fy << "; cx = " << _cx << ", cy = " << _cy;

    }

    int totalPoints = 0;
    int goodImages = 0;

    const int minPoints = 5;

    for( size_t i = 0; i < objectPoints.size(); ++i )  {
      if( objectPoints[i].size() > minPoints ) {
      //ImagePointsVec undistorted =  normalizeUndistortImage( imagePoints[i] );

      bool pnpRes = solvePnP( objectPoints[i], imagePoints[i], mat(), Mat::zeros(1,8,CV_64F), result.rvecs[i], result.tvecs[i], false, CV_ITERATIVE );


        // bool pnpRes = solvePnP( objectPoints[i], imagePoints[i], mat(), Mat::zeros(1,8,CV_64F),
        //      result.rvecs[i], result.tvecs[i], false, CV_ITERATIVE );

        if( !pnpRes ) {
          result.status[i] = false;
          continue;
        }

        result.status[i] = true;

        ++goodImages;
        totalPoints += objectPoints[i].size();
      }
    }

    if( goodImages < 10 ) {
      LOG(WARNING) << "Only got " << goodImages << ".  Not enough to calibrate!";
      return false;
    }

    LOG(INFO) << "From " << objectPoints.size() << " images, using " << totalPoints << " points from " << goodImages << " of the images" << endl;


    double camera[4] = { _fx, _fy, _cx, _cy };
    double alpha = _alpha;

    ceres::LossFunction *lossFunc =  NULL;
    if( flags & CALIB_HUBER_LOSS ) {
      LOG(INFO) << "Using Huber loss function";

      // Need to set parameter, which is in the units
      // of the residual (pixels, in this case)
      // It is squared internally
      lossFunc = new ceres::HuberLoss( 4.0 );
    }

    //AngularDistortionFactory factory(  camera, &alpha, (_distCoeffs.val), lossFunc );

    ceres::LocalParameterization *se3_param = new ceres::ProductParameterization(new ceres::QuaternionParameterization(), new ceres::IdentityParameterization(3));

    ceres::Problem problem;

    double *pose = new double[ goodImages * 7];
    for( size_t i = 0, idx = 0; i < objectPoints.size(); ++i ) {
      if( result.status[i] ) {

        double *p = &( pose[idx*7] );

        ceres::AngleAxisToQuaternion(result.rvecs[i].val, p );

        p[4] = result.tvecs[i][0];
        p[5] = result.tvecs[i][1];
        p[6] = result.tvecs[i][2];

        ++idx;

        for( size_t j = 0; j < imagePoints[i].size(); ++j ) {

          ceres::CostFunction *costFunction = (new ceres::AutoDiffCostFunction<AngularDistortionReprojError, 2, 4, 1, 4, 7>(
                new AngularDistortionReprojError( imagePoints[i][j][0], imagePoints[i][j][1], objectPoints[i][j][0], objectPoints[i][j][1] ) ) );

          problem.AddResidualBlock( costFunction, lossFunc, camera, &alpha, (_distCoeffs.val), p );

          //factory.addCorrespondence( problem, objectPoints[i][j], imagePoints[i][j], p );

          problem.SetParameterization( p, se3_param );

        }
      }
    }

    //if( flags & CALIB_FIX_SKEW )
    // Skew is always fixed in OpenCV 3.0.
    problem.SetParameterBlockConstant( &alpha );
    //problem.SetParameterBlockConstant( _distCoeffs.val );

    problem.SetParameterLowerBound( camera, 0, 1 );
    problem.SetParameterLowerBound( camera, 1, 1 );
    problem.SetParameterLowerBound( camera, 2, _imgSizeHint.width * 0.25 );
    problem.SetParameterUpperBound( camera, 2, _imgSizeHint.width * 0.75 );
    problem.SetParameterLowerBound( camera, 3, _imgSizeHint.height * 0.25 );
    problem.SetParameterUpperBound( camera, 3, _imgSizeHint.height * 0.75 );

    ceres::Solver::Options options;
    options.minimizer_type = ceres::TRUST_REGION; // ceres::LINE_SEARCH;
    if( options.minimizer_type == ceres::TRUST_REGION ) {
      options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
      options.use_nonmonotonic_steps = true;
    } else if( options.minimizer_type == ceres::LINE_SEARCH ) {
      options.line_search_direction_type = ceres::LBFGS;
    }

    options.linear_solver_type = ceres::DENSE_SCHUR; //ceres::SPARSE_NORMAL_CHOLESKY;  //ceres::DENSE_QR;
    options.max_num_iterations = criteria.maxCount;
    options.minimizer_progress_to_stdout = true;

    // This should be configurable by the end user
    options.num_threads = boost::thread::hardware_concurrency();

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << summary.FullReport();

    for( size_t i = 0, idx=0; i < objectPoints.size(); ++i ) {
      if( result.status[i] ) {
        ceres::QuaternionToAngleAxis( &(pose[idx*7]), result.rvecs[i].val );
        result.tvecs[i] = Vec3d( &(pose[idx*7+4]) );
        ++idx;
      }
    }

    delete[] pose;

    result.totalTime = summary.total_time_in_seconds;

    // N.b. the Ceres cost is 1/2 || f(x) ||^2
    //
    // Whereas we usually use the RMS reprojection error
    //
    // sqrt( 1/N  ||f(x)||^2
    //
    // So rms = sqrt( 2/N final_cost )
    //
    result.residual = summary.final_cost;
    result.numPoints = totalPoints;
    result.numImages = goodImages;
    result.good = summary.IsSolutionUsable();

    setCamera(camera, alpha);

    result.rms = reprojectionError( objectPoints, result.rvecs, result.tvecs, imagePoints, result.reprojErrors, result.status );

    cout << "Final camera: " << endl << matx() << endl;
    cout << "Final distortions: " << endl << _distCoeffs << endl;

    return true;
  }


  ImagePoint AngularPolynomial::distort( const ObjectPoint &w ) const
  {
    double theta = atan2( sqrt( w[0]*w[0] + w[1]*w[1] ), w[2] );
    double psi = atan2( w[1], w[0] );

    double theta2 = theta*theta,
           theta4 = theta2*theta2,
           theta6 = theta4*theta2,
           theta8 = theta4*theta4;

    double theta_d = theta * (1 + _distCoeffs[0]*theta2 + _distCoeffs[1]*theta4 + _distCoeffs[2]*theta6 + _distCoeffs[3]*theta8);

    return Vec2f( theta_d*cos( psi ), theta_d*sin(psi) );
  }


  // Use hammer to kill mosquito
  //  Based on the Bundler solver used in their examples
  struct UndistortReprojError {
    UndistortReprojError( double obs_x, double obs_y, const Vec4d &k )
      : observedX(obs_x), observedY(obs_y), _k( k ) {;}

    double observedX, observedY;
    const Vec4d _k;

    template <typename T>
      bool operator()(const T* const p,
          T* residuals) const
      {
        // point is a 2-vector
        //T p[2] = { point[0], point[1] };

        // Important to use atan2 if Z is always 1?
        T theta = atan( sqrt( p[0]*p[0] + p[1]*p[1] )  );
        T psi   = atan2( p[1], p[0] );

        T theta2 =  theta*theta;
        T theta4 =  theta2*theta2;
        T theta6 =  theta4*theta2;
        T theta8 =  theta4*theta4;

        T thetaDist = theta * ( T(1) + _k[0]*theta2 + _k[1]*theta4 + _k[2]*theta6 + _k[3]*theta8);

        T xdn = thetaDist * cos( psi ),
          ydn = thetaDist * sin( psi );

        // The error is the difference between the predicted and observed position.
        residuals[0] = xdn - T(observedX);
        residuals[1] = ydn - T(observedY);
        return true;
      }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x, const double observed_y,
        const Vec4d &k ) {
      return (new ceres::AutoDiffCostFunction<UndistortReprojError, 2, 2>(
            new UndistortReprojError(observed_x, observed_y, k )));
    }
  };


  ImagePointsVec AngularPolynomial::undistort( const ImagePointsVec &pw ) const
  {
    int Np = pw.size();
    double *p = new double[ Np*2 ];

    ceres::Problem problem;
    for( int i = 0; i < Np; ++i ) {
      p[ i*2 ] = pw[i][0];
      p[ i*2 + 1 ] = pw[i][1];

      ceres::CostFunction *costFunction = UndistortReprojError::Create( pw[i][0], pw[i][1], _distCoeffs );
      problem.AddResidualBlock( costFunction, NULL, &(p[i*2]) );
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    ImagePointsVec out( pw.size() );
    for( int i = 0; i < Np; ++i ) {
      out[i] = ImagePoint( p[i*2], p[i*2 + 1] );
    }

    delete[] p;

    return out;


  }

  ImagePoint AngularPolynomial::undistort( const ImagePoint &pw ) const
  {
    double p[2] = { pw[0], pw[1] };

    ceres::Problem problem;
    ceres::CostFunction *costFunction = UndistortReprojError::Create( pw[0], pw[1], _distCoeffs );
    problem.AddResidualBlock( costFunction, NULL, p );

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    ImagePoint out( p[0], p[1] );

    return out;
  }

  //---

  DistortionModel *AngularPolynomial::estimateMeanCamera( vector< DistortionModel *> cameras )
  {
    Mat mean( Mat::zeros(8,1,CV_64F) );
    for( size_t i = 0; i < cameras.size(); ++i ) {
      assert( cameras[i]->name() == name() );

      mean += cameras[i]->coefficientsMat();
    }

    mean /= cameras.size();

    Vec8d vecCoeff;
    mean.copyTo( vecCoeff );

    return new AngularPolynomial( vecCoeff );
  }


  //--- Serialize/unserialize functions ----
  FileStorage &AngularPolynomial::write( FileStorage &out ) const
  {
    DistortionModel::write( out );
    out << "distortion_coefficients" << _distCoeffs;

    return out;
  }

  void AngularPolynomial::to_json( json &j ) const {
    PinholeCamera::to_json(j);
    j["distortion_coefficients"] = { _distCoeffs[0], _distCoeffs[1], _distCoeffs[2], _distCoeffs[3] };
  }


  AngularPolynomial *AngularPolynomial::Load( cv::FileStorage &in )
  {
    Mat kmat;
    Vec4d dist;

    in["camera_matrix"] >> kmat;

    Matx33d k;
    kmat.convertTo( k, CV_64F );

    in["distortion_coefficients"] >> dist;

    return new AngularPolynomial( dist, k );

  }


}
