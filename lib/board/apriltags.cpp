#ifndef USE_APRILTAGS
#warn "Compiling __FILE__ even though Apriltag support hasn't been configured."
#else

#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <libg3logger/g3logger.h>

#include "AprilTags/SubtagDetector.h"

#include "AplCam/board/apriltags.h"
#include "AplCam/detection/apriltags.h"

namespace AplCam {

  using namespace cv;

  using SubtagDetector = AprilTags::SubtagDetector;

  //===========================================================================
  //  AprilTagsBoard
  //===========================================================================

  static Mat drawAnnotatedSubtags( const Mat &gray, Detection *out )
  {
    Mat img;
    cvtColor( gray, img, COLOR_GRAY2BGR );

    for( unsigned int i = 0; i < out->size(); ++i ) {
      ImagePoint pt( out->points[i] );
      ObjectPoint wld( out->corners[i] );

      cv::circle( img, Point2f( pt[0], pt[1] ), 4, Scalar(0,0,255), 1 );

      // Draw the corner code as well
      char out[32];
      snprintf( out, 31, "%.1f,%.1f", wld[0], wld[1] );
      putText( img, out, Point2f( pt[0], pt[1] ), FONT_HERSHEY_SIMPLEX , 0.4, Scalar( 0,255,0 ) );
    }

    return img;
  }


  AprilTagsBoard::AprilTagsBoard( const Mat &ids, float squareSize, const std::string &name,  bool doSubtags  )
    : Board( APRILTAGS, ids.cols, ids.rows, squareSize, name ),
        _ids( ids ),
        _tagCode( AprilTags::tagCodes36h11 ),
        _subtagMinSize( doSubtags ? -1 : 100*100),
        _blackBorder( 1 ),
        _tagSize( -1, -1 )
  {
    ;
  }

  Detection *AprilTagsBoard::detectPattern( const cv::Mat &img )
  {
    AprilTags::TagDetector tagDetector( _tagCode );

    Mat gray;
    ensureGrayscale( img, gray );

    vector<AprilTags::TagDetection> detections = tagDetector.extractTags(gray);
    LOG(INFO) << "Detected " << detections.size() << " AprilTags";

    if( _subtagMinSize > 0.0 ) {

      Detection *detect = attemptSubtagDetection( gray, detections );
      if( detect != NULL ) return detect;
    }

    ObjectPointsVec worldLocations(detections.size());
    for( unsigned int i = 0; i < detections.size(); ++i ){
      worldLocations[i] = worldLocation( detections[i].id );
    }
    AprilTagsDetection *detect = new AprilTagsDetection( detections, worldLocations );
    return detect;
  }

  Detection *AprilTagsBoard::attemptSubtagDetection( const Mat &gray, vector<AprilTags::TagDetection> &detections )
  {
    if( _tagSize.width < 0.0 ) {
      LOG(WARNING) << "Tag size not specified in board file, cannot do subtag detection.";
      return NULL;
    }

    SubtagDetector subtag( AprilTags::tagCodes36h11 );
    subtag.saveDebugImages( true );
    subtag.setSigma( 1.5 );
    //vector<AprilTags::SubtagDetection> subtagDetections;

    Detection *out = new Detection;
    int id = 0;

    for( unsigned int i = 0; i < detections.size(); ++i ){
      float area = detections[i].totalArea();

      bool doSubtag = (area >= _subtagMinSize);
      LOG(INFO) << "Tag area of " << area << " pixels, " << (doSubtag ? "attempting" : "skipping") << " subtag detection";

      if( !doSubtag ) continue;

      vector<AprilTags::CornerDetection> corners( subtag.detectTagSubstructure( gray, detections[i] ));

      ObjectPoint tagCenter( worldLocation( detections[i].id ));

      for( unsigned int i = 0; i < corners.size(); ++i ) {

        // Position of corner in bit units from the center of the tag
        Point2f pt( corners[i].inTag );
        ObjectPoint onBoard( corners[i].inTag.x * _tagSize.width/2.0,
                             corners[i].inTag.y * _tagSize.height/2.0,
                             0.0);
        onBoard += tagCenter;

        // LOG(INFO) << "Point at " << corners[i].inTag.x << "," << corners[i].inTag.y << " a point " << pt << " world " << onBoard;

        out->add( onBoard, ImagePoint( corners[i].inImage), id++ );
      }

    }

    // Mat annotated( drawAnnotatedSubtags( gray, out ) );
    // imshow("annotated", annotated );
    // waitKey();

    //if( subtagDetections.size() > 0 ) return new AprilTagsSubtagDetection();
    if( out->size() == 0 ) {
      delete out;
      return NULL;
    }

    return out;
  }



  bool AprilTagsBoard::find( const int id, cv::Point2i &xy  ) const
  {
    for( int x = 0; x < width; ++x )
    for( int y = 0; y < height; ++y )
    if( _ids.at<int>(y,x) == id )
    {
      xy.x = x; xy.y = y;
      return true;
    }

    return false;
  }

  std::vector< int > AprilTagsBoard::ids( void )
  {
    vector< int > out;
    for( int x = 0; x < width; ++x )
    for( int y = 0; y < height; ++y )
    out.push_back( _ids.at< int >( y, x ) );

    return out;
  }

  ObjectPoint AprilTagsBoard::worldLocation( const int id ) const
  {
    // Point3f ptSpace = worldLocation( Point2i(0,0) ) - worldLocation(Point2i(1,1));
    // pointSpacing =  ptSpace.x * ptSpace.x + ptSpace.y * ptSpace.y;

    // for( size_t i = 0; i < _det.size(); ++i ) {

      Point2i loc;
      if( !find( id, loc ) ) {
      //   //cout << "Found  tag id " << _det[i].id << endl;
      //
      //   points.push_back( Point2f( _det[i].cxy.first, _det[i].cxy.second ) );
      //   corners.push_back( board.worldLocation( loc ) );
      //   ids.push_back( _det[i].id );
      //
      // } else {
        LOG(WARNING) << "Couldn't find tag ID \'" << id << "\'";
      }

      return Board::worldLocation(loc);

  }

  void AprilTagsBoard::setTagSize( float width, float height )
  {
      _tagSize.width = width;
      if( height < 0 )
        _tagSize.height = width;
      else
        _tagSize.height = height;
  }

  void AprilTagsBoard::setBlackBorder( unsigned int border )
  {
    _blackBorder = border;
  }


  AprilTagsBoard *AprilTagsBoard::Load( cv::FileStorage &fs, const string &name )
  {
    Mat ids;
    float squareSize;
    //
    // fs["width"] >> width;
    // fs["height"] >> height;
    fs["squareSize"] >> squareSize;

    fs["ids"] >> ids;

    AprilTagsBoard *board = new AprilTagsBoard( ids, squareSize, name );

    if( !fs["blackBorder"].empty() ) board->setBlackBorder( (int)fs["blackBorder"] );
    if( !fs["tagSize"].empty()) board->setTagSize( (float)fs["tagSize"] );

    return board;
  }


}

#endif
