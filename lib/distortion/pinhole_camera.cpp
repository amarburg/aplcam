
#include <iostream>

#include <opencv2/core/affine.hpp>
#include <libg3logger/g3logger.h>

#include "AplCam/distortion/pinhole_camera.h"

namespace Distortion {

  using namespace std;
  using namespace cv;


  // Maintain the notation that the entry at (0,1) is alpha * fx
  PinholeCamera::PinholeCamera( void )
    : _fx(1.0), _fy(1.0), _alpha(0.0), _cx(0.0), _cy(0.0)
  {;}

  PinholeCamera::PinholeCamera( const Matx33d &k )
    : _fx( k(0,0) ), _fy( k(1,1) ), _alpha( k(0,1)/_fx ),
    _cx( k(0,2) ), _cy( k(1,2) )
  {;}

  PinholeCamera::PinholeCamera( const Vec4d &coeffs )
    : _fx( coeffs[0] ), _fy( coeffs[1] ), _alpha( 0.0 ),
    _cx( coeffs[2] ), _cy( coeffs[3] )
  {;}



  PinholeCamera::PinholeCamera( const Mat &k )
  { Matx33d kMatx;
    k.convertTo( kMatx, CV_64F );
    setCamera( kMatx );
  }


  // ---- Accessor functions ---

  void PinholeCamera::setCamera( const Matx33d &k )
  {
    setCamera( k(0,0), k(1,1), k(0,2), k(1,2), k(0,1)/k(0,0) );
  }

  void PinholeCamera::setCamera( double fx, double fy, double cx, double cy, double alpha )
  {
    _fx = fx;
    _fy = fy;
    _cx = cx;
    _cy = cy;
    _alpha = alpha;
  }

  void PinholeCamera::setCamera( const double *c, double alpha )
  {
    setCamera( c[0], c[1], c[2], c[3], alpha );
  }

  Matx33d PinholeCamera::matx( void ) const
  {
    return Matx33d( _fx, _alpha*_fx, _cx, 0., _fy, _cy, 0., 0., 1. );
  }

  Mat PinholeCamera::mat( void ) const
  {
    return Mat(matx());
  }

  // --- Image/normalize functions ----

  ImagePoint PinholeCamera::image( const ImagePoint &xd ) const
  {
    return  ImagePoint( _fx * ( xd[0] + _alpha*xd[1] ) + _cx,
                        _fy *   xd[1]                    + _cy );
  }

  ImagePoint PinholeCamera::normalize( const ImagePoint &pt ) const
  {
    return ImagePoint( 1.0 / _fx * (pt[0] - _cx ),
                       1.0 / _fy * (pt[1] - _cy ) );
  }

  //---- getOptimalNewCameraMatrix ----

  Mat PinholeCamera::getOptimalNewCameraMatrix( const Size &imgSize,
      double alpha, const Size &newImgSize,
      Rect &validPixROI, bool centerPrincipalPoint )
  {

    Rect_<float> inner, outer;
    Size theImgSize = (newImgSize.width*newImgSize.height != 0 ) ? newImgSize : imgSize;

    double M[3][3];
    Mat matM(3, 3, CV_64F, M);
    mat().copyTo( matM );

    if( centerPrincipalPoint )
    {
      double cx0 = M[0][2];
      double cy0 = M[1][2];
      double cx = (theImgSize.width-1)*0.5;
      double cy = (theImgSize.height-1)*0.5;

      getRectangles( Mat(), mat(), imgSize, inner, outer );
      double s0 = std::max(std::max(std::max((double)cx/(cx0 - inner.x), (double)cy/(cy0 - inner.y)),
            (double)cx/(inner.x + inner.width - cx0)),
          (double)cy/(inner.y + inner.height - cy0));
      double s1 = std::min(std::min(std::min((double)cx/(cx0 - outer.x), (double)cy/(cy0 - outer.y)),
            (double)cx/(outer.x + outer.width - cx0)),
          (double)cy/(outer.y + outer.height - cy0));
      double s = s0*(1 - alpha) + s1*alpha;

      M[0][0] *= s;
      M[1][1] *= s;
      M[0][2] = cx;
      M[1][2] = cy;

      inner = Rect_<float>((float)((inner.x - cx0)*s + cx),
          (float)((inner.y - cy0)*s + cy),
          (float)(inner.width*s),
          (float)(inner.height*s));

      Rect r( cvCeil(inner.x), cvCeil(inner.y), cvFloor(inner.width), cvFloor(inner.height));
      r &= Rect(0, 0, theImgSize.width, theImgSize.height);

      validPixROI = r;
    }
    else
    {
      // Get inscribed and circumscribed rectangles in normalized
      // (independent of camera matrix) coordinates
      getRectangles( Mat(), Mat(), imgSize, inner, outer );

      // Projection mapping inner rectangle to viewport
      double fx0 = (theImgSize.width  - 1) / inner.width;
      double fy0 = (theImgSize.height - 1) / inner.height;
      double cx0 = -fx0 * inner.x;
      double cy0 = -fy0 * inner.y;

      // Projection mapping outer rectangle to viewport
      double fx1 = (theImgSize.width  - 1) / outer.width;
      double fy1 = (theImgSize.height - 1) / outer.height;
      double cx1 = -fx1 * outer.x;
      double cy1 = -fy1 * outer.y;

      // Interpolate between the two optimal projections
      M[0][0] = fx0*(1 - alpha) + fx1*alpha;
      M[1][1] = fy0*(1 - alpha) + fy1*alpha;
      M[0][2] = cx0*(1 - alpha) + cx1*alpha;
      M[1][2] = cy0*(1 - alpha) + cy1*alpha;

      getRectangles(  Mat(), matM, imgSize, inner, outer );
      Rect r = inner;
      r &= cv::Rect(0, 0, theImgSize.width, theImgSize.height);
      validPixROI = r;
    }

    // Because the underlying data is actually M[][], it
    // goes undefined when the function exits
    Mat output;
    matM.copyTo( output );
    return output;
  }

  // As above, but discards ROI
  Mat PinholeCamera::getOptimalNewCameraMatrix( const Size &imgSize, double alpha,
                                const Size &newImgSize, bool centerPrincipalPoint )
  {
    cv::Rect validROI;
    return getOptimalNewCameraMatrix( imgSize, alpha, newImgSize, validROI, centerPrincipalPoint );
  }


  void PinholeCamera::getRectangles(
      const Mat &R, const Mat &newCameraMatrix, const Size &imgSize,
      Rect_<float>& inner, Rect_<float>& outer ) const
  {
    const int N = 9;
    int x, y, k;
    ImagePointsVec pts;

    for( y = k = 0; y < N; y++ )
      for( x = 0; x < N; x++ )
        pts.push_back( ImagePoint((float)x*imgSize.width/(N-1),
              (float)y*imgSize.height/(N-1)) );

    undistortPoints(pts, pts, R, newCameraMatrix);

    float iX0=-FLT_MAX, iX1=FLT_MAX, iY0=-FLT_MAX, iY1=FLT_MAX;
    float oX0=FLT_MAX, oX1=-FLT_MAX, oY0=FLT_MAX, oY1=-FLT_MAX;
    // find the inscribed rectangle.
    // the code will likely not work with extreme rotation matrices (R) (>45%)
    for( y = k = 0; y < N; y++ )
      for( x = 0; x < N; x++ )
      {
        ImagePoint &p = pts[ y*N + x ];
        oX0 = MIN(oX0, p[0]);
        oX1 = MAX(oX1, p[0]);
        oY0 = MIN(oY0, p[1]);
        oY1 = MAX(oY1, p[1]);

        if( x == 0 )
          iX0 = MAX(iX0, p[0]);
        if( x == N-1 )
          iX1 = MIN(iX1, p[0]);
        if( y == 0 )
          iY0 = MAX(iY0, p[1]);
        if( y == N-1 )
          iY1 = MIN(iY1, p[1]);
      }
    inner = Rect_<float>(iX0, iY0, iX1-iX0, iY1-iY0);
    outer = Rect_<float>(oX0, oY0, oX1-oX0, oY1-oY0);
  }


  struct ReprojectorFunctor {
    ReprojectorFunctor( const Matx33d &mat ) : _mat( mat ) {;}
    Matx33d _mat;

    ImagePoint operator()( const ImagePoint &pt )
    {
      Vec3d pth( pt[0], pt[1], 1.0 );
      Vec3d out = _mat * pth;
      return ImagePoint( out[0]/out[2], out[1]/out[2] );
    }
  };

  void PinholeCamera::undistortPoints( const ImagePointsVec &distorted,
      ImagePointsVec &undistorted,
      const Mat &R, const Mat &P) const
  {
    undistorted.resize(distorted.size());

    CV_Assert(P.empty() || P.size() == Size(3, 3) || P.size() == Size(4, 3));
    CV_Assert(R.empty() || R.size() == Size(3, 3) ); //|| R.total() * R.channels() == 3);

    // I believe the original supported axis-angle rotation vectors as well
    cv::Matx33d RR( Matx33d::eye() );
    if (!R.empty() && R.total() * R.channels() == 3)
    {
      cv::Vec3d rvec;
      R.convertTo(rvec, CV_64F);
      RR = cv::Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == Size(3, 3))
      R.convertTo(RR, CV_64F);

    if(!P.empty())
    {
      cv::Matx33d PP;
      P.colRange(0, 3).convertTo(PP, CV_64F);
      RR = PP * RR;
    }

    undistorted = normalizeUndistort( distorted );

    std::transform( undistorted.begin(), undistorted.end(), undistorted.begin(),
        ReprojectorFunctor( RR ) );


    //    for(size_t i = 0; i < distorted.size(); i++ )
    //    {
    //      Vec2d pw( unimage( distorted[i] ) );
    //      Vec2d pu( undistort( pw ) );
    //
    //      // reproject
    //      Vec3d pr = RR * Vec3d(pu[0], pu[1], 1.0); // rotated point optionally multiplied by new camera matrix
    //      Vec2d fi(pr[0]/pr[2], pr[1]/pr[2]);       // final
    //
    //      undistorted[i] = fi;
    //    }

  }

  void PinholeCamera::projectPoint( const ObjectPoint &objPt, const Vec3d &rvec, const Vec3d &tvec, ImagePoint &imgPt ) const
  {
   Affine3d aff(rvec, tvec);
  Vec3d Xworld( objPt );
      Vec3d Xcam( aff*Xworld );

      imgPt = image( distort( Xcam ) );
  }


  void PinholeCamera::projectPoints( const ObjectPointsVec &objectPoints,
      const Vec3d &rvec, const Vec3d &tvec, ImagePointsVec &imagePoints ) const
  {
    // will support only 3-channel data now for points
    imagePoints.resize(objectPoints.size());
    Affine3d aff(rvec, tvec);

    for(size_t i = 0; i < objectPoints.size(); ++i)
    {
      Vec3d Xworld( objectPoints[i] );
      Vec3d Xcam( aff*Xworld );

      Vec2d imaged = image( distort( Xcam ) );
      imagePoints[i] = imaged;
    }
  }

  //-- Image/normalize functions ----

  ImagePointsVec PinholeCamera::image( const ImagePointsVec &vec ) const
  {
    ImagePointsVec out( vec.size() );
    std::transform( vec.begin(), vec.end(), out.begin(), makeImager() );
    return out;
  }

  ImagePointsVec PinholeCamera::normalize( const ImagePointsVec &vec ) const
  {
    ImagePointsVec out;
    std::transform( vec.begin(), vec.end(), back_inserter( out ), makeNormalizer() );
    return out;
  }

  //--- Various combination functions ---

   ImagePointsVecVec PinholeCamera::normalizeUndistortImage( const ImagePointsVecVec &pw ) const
   {
     ImagePointsVecVec out;
     for( size_t i = 0; i < pw.size(); ++i )
       out.push_back( normalizeUndistortImage(pw[i]) );
     return out;
   }


  // Many, slightly different permutations on the same thing depending on the desired outcome.
  // Could be cleaned up to reduce DRY?
  //
  // Based on the assumption that the ReprojErrors* version use more storage
  // space that doesn't need to be used if the norm is being calculated just once

  double PinholeCamera::reprojectionError( const ObjectPointsVec &objPts,
      const Vec3d &rvec, const Vec3d &tvec,
      const ImagePointsVec &imgPts )
  {
    ImagePointsVec projPts;
    projectPoints( objPts, rvec, tvec, projPts );
    double err = cv::norm( projPts, imgPts, NORM_L2 );
    return sqrt( (err*err) / objPts.size() );
  }

  double PinholeCamera::reprojectionError( const ObjectPointsVec &objPts,
      const Vec3d &rvec, const Vec3d &tvec,
      const ImagePointsVec &imgPts,
      ReprojErrorVec &reproj )
  {
    ImagePointsVec projectedPoints;

    projectPoints( objPts, rvec, tvec, projectedPoints);
    std::transform( projectedPoints.begin(), projectedPoints.end(), imgPts.begin(), std::back_inserter(reproj),
                  [&]( const ImagePoint &projPt, const ImagePoint &imgPt ) -> ReprojError {
                    ReprojError rpj;
                    rpj.projPoint = projPt;
                    rpj.error = projPt - imgPt;
                    return rpj;
                  } );

    vector< Vec2f > errors;
    std::transform( reproj.begin(), reproj.end(), std::back_inserter(errors),
                    []( const ReprojError &rep ) -> Vec2f { return Vec2f( rep.error[0], rep.error[1]); } );

    double err = cv::norm( errors, NORM_L2 );
    return sqrt( (err*err) / objPts.size() );
  }



  double PinholeCamera::reprojectionError( const ObjectPointsVecVec &objPts,
                                             const RotVec &rvecs,
                                             const TransVec &tvecs,
                                             const ImagePointsVecVec &imgPts,
                                             const vector<bool> mask )
  {
    int numPoints = 0;
    double rms = 0.0;

    for( size_t j = 0; j < objPts.size(); ++j ) {
      if( !mask.empty() && mask[j] == false ) continue;

      numPoints += objPts[j].size();

      ImagePointsVec projPts;
      projectPoints( objPts[j], rvecs[j], tvecs[j], projPts );

      double err = cv::norm( projPts, imgPts[j], NORM_L2 );
      rms += err*err;
    }

    return sqrt(rms/numPoints);
  }

  double PinholeCamera::reprojectionError( const ObjectPointsVecVec &objPts,
                                             const RotVec &rvecs,
                                             const TransVec &tvecs,
                                             const ImagePointsVecVec &imgPts,
                                             ReprojErrorVecVec &reproj,
                                             const vector<bool> mask )
  {
    int numPoints = 0;
    double rms = 0.0;

    // TODO.  Reduce DRY with other reprojectionError functions

    reproj.resize( objPts.size() );
    for( size_t j = 0; j < objPts.size(); ++j ) {
      if( !mask.empty() && mask[j] == false ) continue;
      if( objPts[j].size() == 0 ) continue;

      numPoints += objPts[j].size();

      ImagePointsVec projectedPoints;
      projectPoints( objPts[j], rvecs[j], tvecs[j], projectedPoints );

      std::transform( projectedPoints.begin(), projectedPoints.end(), imgPts[j].begin(), std::back_inserter(reproj[j]),
                      [&]( const ImagePoint &projPt, const ImagePoint &imgPt ) -> ReprojError {
                        ReprojError rpj;
                        rpj.projPoint = projPt;
                        rpj.error = projPt - imgPt;
                        return rpj;
                      } );

      vector< Vec2f > errors;
      std::transform( reproj[j].begin(), reproj[j].end(), std::back_inserter(errors),
                      []( const ReprojError &rep ) -> Vec2f { return Vec2f( rep.error[0], rep.error[1]); } );

      double err = cv::norm( errors, NORM_L2 );
      rms += err*err;
    }

    return sqrt(rms/numPoints);
  }



 // --- Serialization/unserialization functions ----
 FileStorage &PinholeCamera::write( FileStorage &out ) const
 {
   out << "camera_model" << name();
   out << "camera_matrix" << mat();
   return out;
 }


  void PinholeCamera::to_json( json &j ) const {
    j["camera_model"] = name();
    j["camera_matrix"] = { mat().at<double>(0), mat().at<double>(1), mat().at<double>(2), mat().at<double>(3), mat().at<double>(4), mat().at<double>(5), mat().at<double>(6), mat().at<double>(7), mat().at<double>(8) };
   //out << "camera_model" << name();
   //out << "camera_matrix" << mat();
   //return out;
 }


};
