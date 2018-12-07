
#include <iostream>
#include <fstream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <libg3logger/g3logger.h>

#include "AplCam/board/board.h"
#include "AplCam/board/apriltags.h"
#include "AplCam/board/circle.h"
#include "AplCam/board/trailer_hitch.h"

#include "AplCam/detection/detection.h"

namespace AplCam {

  using namespace std;
  using namespace cv;

  Detection *Board::detectPattern( const Mat &gray )
  {
    Detection *detect = new Detection();

    switch( pattern )
    {
      case CHESSBOARD:
        findChessboardCorners( gray, size(), detect->points,
                                              CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

        if( detect->good() ) cornerSubPix( gray, detect->points, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

        break;
      case CIRCLES_GRID:
        findCirclesGrid( gray, size(), detect->points );
        break;
      case ASYMMETRIC_CIRCLES_GRID:
        findCirclesGrid( gray, size(), detect->points, CALIB_CB_ASYMMETRIC_GRID );
        break;
      default:
        cerr << "Unknown pattern type" << endl;
        return NULL;
    }

    //detect->calculateCorners( *this );
    return detect;
  }

  void Board::ensureGrayscale( const Mat &img, Mat &gray )
  {

    // Automatically convert to gray --- somewhat ambivalent about this
    // but otherwise require use to know if detector takes color or gray images
    if( img.channels() != 1 ) {
      cvtColor( img, gray, CV_BGR2GRAY );
    } else {
      gray = img;
    }
  }


  Board *Board::load( const string &infile, const string &name )
  {
    FileStorage fs( infile, FileStorage::READ );
    if( ! fs.isOpened() ) {
      cout << "Couldn't open board file \"" << infile << "\"" << endl;
      exit(-1);
    }

    string type_s;
    int width, height;
    float squares;
    //Pattern type;

    fs["type"] >> type_s;
    fs["width"] >> width;
    fs["height"] >> height;
    fs["squareSize"] >> squares;

    Board *board = NULL;
    if( type_s.compare("chessboard" ) == 0 ) {
      LOG(INFO) << "Creating chessboard";
      board = new Board( CHESSBOARD, width, height, squares, name );
    } else if( type_s.compare("circle") == 0 ) {
      LOG(INFO) << "Creating circle board";
      board = new CircleBoard( name );
    } else if( type_s.compare("color_seg_circle") == 0 ) {
      LOG(INFO) << "Creating color segmentation circle board.";
      board = new ColorSegmentationCircleBoard( name );
    } else if( type_s.compare("apriltags_36h11" ) == 0) {
  #ifdef USE_APRILTAGS
      LOG(INFO) << "Creating Apriltags 36H11 board.";
      board = AprilTagsBoard::Load( fs, name );
  #else
      LOG(FATAL) << "Not compiled for Apriltags." << endl;
  #endif
    } else {
      LOG(WARNING) << "Don't know how to handle board type \"" << type_s << "\"" << endl;
    }

    board->loadCallback( fs );

    return board;
  }

  ObjectPoint Board::worldLocation( const cv::Point2i &xy ) const
  {
    Point2f idx( xy.x - (float)(size().width-1)/2.0, xy.y - (float)(size().height-1)/2.0 );
    //ObjectPoint halfSize( size().width * squareSize / 2.0, size().height * squareSize / 2.0, 0 );
    return ObjectPoint( idx.x * squareSize, idx.y * squareSize, 0.0 );
  }

  ObjectPointsVec Board::corners( void ) // const CornersReference ref )
  {
    ObjectPoint halfSize( squareSize * size().width / 2.0, squareSize * size().height / 2.0, 0 );

    ObjectPointsVec out;
    for( int x = 0; x < width; ++x )
      for( int y = 0; y < height; ++y )
        //if( ref == BOARD_UL )
        //  out.push_back( worldLocation( Point2i( x, y ) ) );
        //else
        out.push_back( worldLocation( Point2i(x,y) ) );

    return out;
  }

  std::vector< int > Board::ids( void )
  {
    return vector< int >();
  }

  void Board::extents( ObjectPointsVec &vec ) const
  {
    vec.resize(4);
    vec[0] = worldLocation( Point2i( 0,0 ) );
    vec[1] = worldLocation( Point2i( width-1,0 ) );
    vec[2] = worldLocation( Point2i( width-1,height-1 ) );
    vec[3] = worldLocation( Point2i( 0,height-1 ) );
  }

  void Board::draw( cv::Mat &img, Detection *detection ) const
  {
    // Generic function, draw all of the detection points w/o knowledge of the
    // board's structure

    detection->draw( img );
  }

  //===========
  std::shared_ptr<Board> BoardFactory::LoadJson( const std::string &filename ) {
    ifstream in( filename );
    json j;
    in >> j;

    std::string type = j["type"];

    if( type.compare("hitches") == 0 ) {
      TrailerHitch *board( new TrailerHitch( type ) );
      board->from_json(j);

      return shared_ptr<Board>(static_cast<Board *>(board));
    } else {
      LOG(WARNING) << "Unable to parse calibration type \"" << type << "\"";
      return nullptr;
    }
  }


}
