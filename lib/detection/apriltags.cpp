#ifndef USE_APRILTAGS
#warn "Compiling __FILE__ even though AprilTags support isn't selected"
#else

#include <opencv2/calib3d/calib3d.hpp>
#include <libg3logger/g3logger.h>

#include "AplCam/file_utils.h"

#include "AplCam/detection/detection.h"
#include "AplCam/detection/apriltags.h"
#include "AplCam/board/apriltags.h"


namespace AplCam {
  using namespace std;
  using namespace cv;

  //============================================================================
  //  AprilTagDetection
  //============================================================================

  AprilTagsDetection::AprilTagsDetection( vector< AprilTags::TagDetection > det, ObjectPointsVec worldLocations )
      : Detection(), _det(det)
    {
      assert( _det.size() == worldLocations.size() );
      for( size_t i = 0; i < _det.size(); ++i ) {
        add( worldLocations[i], Point2f( _det[i].cxy.first, _det[i].cxy.second ), _det[i].id );
      }
    }


  //
  // void AprilTagsDetection::calculateCorners( const AprilTagsBoard &board )
  // {
  //   // Go for a simple model here, assume all tags are unique on the board
  //
  //   points.clear();
  //   corners.clear();
  //   ids.clear();
  //
  //
  //   Point3f ptSpace = board.worldLocation( Point2i(0,0) ) - board.worldLocation(Point2i(1,1));
  //   pointSpacing =  ptSpace.x * ptSpace.x + ptSpace.y * ptSpace.y;
  //
  //   for( size_t i = 0; i < _det.size(); ++i ) {
  //
  //     Point2i loc;
  //     if( board.find( _det[i].id, loc ) ) {
  //       //cout << "Found  tag id " << _det[i].id << endl;
  //
  //       points.push_back( Point2f( _det[i].cxy.first, _det[i].cxy.second ) );
  //       corners.push_back( board.worldLocation( loc ) );
  //       ids.push_back( _det[i].id );
  //
  //     } else {
  //       LOG(ERROR) << "Couldn't find tag \'" << _det[i].id << "\'" << endl;
  //     }
  //   }
  // }


  typedef vector< std::array< size_t, 4 > > IdArray;

  static void buildComboArray( IdArray &arr, size_t maxVal )
  {
    assert( maxVal >= 4 );
    arr.resize( maxVal * (maxVal-1) * (maxVal-2) * (maxVal - 3) );

    size_t idx = 0;

    for( size_t i = 0; i < maxVal; ++i ) {
      for( size_t j = 0; j < maxVal; ++j ) {
        if( i == j ) continue;

        for( size_t k = 0; k < maxVal; ++k ) {
          if( k == i || k == j ) continue;

          for( size_t m = 0; m < maxVal; ++m ) {
            if( m == i || m == j || m ==k ) continue;

            //std::array<size_t,3> a(i,j,k);
            arr[ idx++ ] = std::array<size_t,4>{ {i,j,k,m} };
          }

        }
      }
    }
  }

  static bool isInlierPoint( const ImagePoint &img, const ObjectPoint &world, const Mat &H, float thresholdSq )
  {
    Vec3f wHom;
    cv::multiply( H, Vec3f( img[0], img[1], 1 ), wHom );
    Vec3f wPt( wHom[0]/wHom[2], wHom[1]/wHom[2], 0 );

    Vec3f diff = world - wPt;

    float normSq = diff[0]*diff[0] + diff[1]*diff[1];

    return ( normSq < thresholdSq ) ;
  }

  Detection::Validate_Return_Code AprilTagsDetection::validate( void )
  {
    size_t sz( size() );

    // Technically you only need 4 but assert you need 5 so you get 1-point robustness
    if( sz < 5 ) {
      return NOT_ENOUGH_POINTS;
    }

    IdArray combos;
    buildComboArray( combos, sz );

    int bestSupport = -1;
    Mat bestH;

    float threshold = 1.0; // TODO:  was Detection::pointSpacing, but that should lie in the Board, not in the detection
    LOG(INFO) << "Using inlier threshold " << threshold << endl;

    for( IdArray::iterator itr = combos.begin(); itr != combos.end(); ++itr ) {
      LOG(INFO) << "Trying: " << (*itr)[0] << " " << (*itr)[1] << " " << (*itr)[2] <<  " " << (*itr)[3];

      vector< Point2f > srcPts, dstPts;

      for( int i = 0; i < 4; ++i ) {
        ImagePoint &img( points[ (*itr)[i] ] );
        ObjectPoint &wld( corners[ (*itr)[i] ] );

        // Obviously this only works if the distortion is not too terrible
        srcPts.push_back( Point2f( img[0], img[1] ) );
        dstPts.push_back( Point2f( wld[0], wld[1] ) );
      }

      Mat h = findHomography( srcPts, dstPts, 0 );

      // Couldn't find H
      if( h.empty() ) { continue; }

      // Calculate support
      int support = 0;
      for( size_t i = 0; i < sz; ++ i ) {
        if( i == (*itr)[0] || i == (*itr)[1] || i == (*itr)[2] || i == (*itr)[3] ) continue;

        if( isInlierPoint( points[i], corners[i], h, threshold ) )
          support++;
      }

      if( support == (signed int)(sz-4) ) {
        LOG(INFO) << "Got perfect support.";
        return ALL_VALID;
      }

      if( support > bestSupport ) {
        bestSupport = support;
        bestH = h;
      }

    }

    if( bestSupport < 0 ) {
      LOG(INFO) << "Found no good configurations.";
      // No good configurations
      return NO_GOOD_CONFIGURATIONS;
    }


    LOG(INFO) << "Best support for " << bestSupport+4 << " / " << sz << " points";

    // Now actually drop outliers.  Awkward right now
    ObjectPointsVec oldWld( corners );
    ImagePointsVec oldImg( points );
    vector< int > oldIds;

    corners.clear();
    points.clear();
    ids.clear();

    for( size_t i = 0; i < oldWld.size(); ++i ) {
      if( isInlierPoint( oldImg[i], oldWld[i], bestH, threshold ) ) {
        corners.push_back( oldWld[i] );
        points.push_back( oldImg[i] );
        ids.push_back( oldIds[i] );
      }


    }

    return ALL_VALID;
  }



}

#endif
