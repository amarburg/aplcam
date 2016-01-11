
#include <iostream>
using namespace std;

#include <opencv2/imgproc/imgproc.hpp>

#include "AplCam/distortion/distortion_model.h"
#include "AplCam/distortion/angular_polynomial.h"
#include "AplCam/distortion/radial_polynomial.h"

namespace Distortion {

  using namespace cv;
  using namespace std;


  void DistortionModel::undistortImage( const Mat &distorted, Mat &undistorted,
      const Mat &Knew, const Size& new_size)
  {
    Size size = new_size.area() != 0 ? new_size : distorted.size();

    Mat map1, map2;
    initUndistortRectifyMap(Mat(cv::Matx33d::eye()), Knew, size, CV_16SC2, map1, map2 );
    remap(distorted, undistorted, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
  }

  void DistortionModel::initUndistortRectifyMap( const Mat &R, const Mat &P,
      const cv::Size& size, int m1type, Mat &map1, Mat &map2 )
  {
    CV_Assert( m1type == CV_16SC2 || m1type == CV_32F || m1type <=0 );
    map1.create( size, m1type <= 0 ? CV_16SC2 : m1type );
    map2.create( size, map1.type() == CV_16SC2 ? CV_16UC1 : CV_32F );

 //   CV_Assert((P.depth() == CV_32F || P.depth() == CV_64F) && (R.depth() == CV_32F || R.depth() == CV_64F));
    CV_Assert(R.empty() || R.size() == Size(3, 3) || R.total() * R.channels() == 3);
    CV_Assert(P.empty() || P.size() == Size(3, 3) || P.size() == Size(4, 3));

    cv::Matx33d RR  = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
      cv::Vec3d rvec;
      R.convertTo(rvec, CV_64F);
      RR = Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == Size(3, 3))
      R.convertTo(RR, CV_64F);

    cv::Matx33d PP = cv::Matx33d::eye();
    if (!P.empty())
      P.colRange(0, 3).convertTo(PP, CV_64F);

    cv::Matx33d iR = (PP * RR).inv(cv::DECOMP_SVD);

    for( int i = 0; i < size.height; ++i)
    {
      float* m1f = map1.ptr<float>(i);
      float* m2f = map2.ptr<float>(i);
      short*  m1 = (short*)m1f;
      ushort* m2 = (ushort*)m2f;

      Vec3d world( i*iR(0, 1) + iR(0, 2),
          i*iR(1, 1) + iR(1, 2),
          i*iR(2, 1) + iR(2, 2) );

      for( int j = 0; j < size.width; ++j)
      {
        ImagePoint pt( distortImage( world ) );
        double u = pt[0], v = pt[1];

        if( m1type == CV_16SC2 )
        {
          int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
          int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
          m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
          m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
          m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
        }
        else if( m1type == CV_32FC1 )
        {
          m1f[j] = (float)u;
          m2f[j] = (float)v;
        }

        world += Vec3d( iR(0, 0), iR(1, 0), iR(2, 0) );
      }
    }
  }


  // This is actually meaningful in DistortionModel, the version in
  // PinholeCamera does nothing
  ImagePointsVec DistortionModel::undistortVec( const ImagePointsVec &pw ) const
  {
    ImagePointsVec out( pw.size() );
    std::transform( pw.begin(), pw.end(), out.begin(), makeUndistorter() );
    return out;
  }





  //--- Statics related to unserailizing cameras... ----

  DistortionModel::CalibrationType_t DistortionModel::ParseCalibrationType( const string &arg )
    {
      cout << arg << endl;
      if( arg == "angular" ) {
        return ANGULAR_POLYNOMIAL;
      } else if ( arg == "radial" ) {
        return CERES_RADIAL;
      } else if ( arg.compare("cv_radial") == 0 ) {
        return OPENCV_RADIAL;
      }

      return CALIBRATION_NONE;
    }



  DistortionModel *DistortionModel::MakeDistortionModel( CalibrationType_t type )
    {
      // Could/should farm this out to distortion..
      switch( type ) {
        case ANGULAR_POLYNOMIAL:
//          cout << "Using angular polynomial (Olson) calibration" << endl;
          return new Distortion::AngularPolynomial;
          break;

        case CERES_RADIAL:
          return new Distortion::CeresRadialPolynomial;
          break;

        case RADIAL8_POLYNOMIAL:
//          cout << "Using radial polynomial (normal OpenCV) 8-coefficient calibration" << endl;
//          return new Distortion::RadialPolynomial( CV_CALIB_RATIONAL_MODEL );
return NULL;
          break;

        case OPENCV_RADIAL:
//          cout << "Using radial polynomial (normal OpenCV) calibration" << endl;
          return new Distortion::OpencvRadialPolynomial;
          break;

        case CALIBRATION_NONE:
        default:
//          cout << "Not calibration model specified!" << endl;
          return NULL;
      }

      return NULL;
    }




}
