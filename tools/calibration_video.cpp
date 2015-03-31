#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <getopt.h>

#include <iostream>

#include "my_undistort.h"

#include "distortion_model.h"
#include "distortion_angular_polynomial.h"
#include "distortion_radial_polynomial.h"
using namespace Distortion;

#include "file_utils.h"
#include "board.h"
#include "detection.h"
#include "image.h"

using namespace cv;
using namespace std;

using kyotocabinet::HashDB;
using kyotocabinet::DB;

class CalibrationOpts {

  public:

    typedef enum { ANGULAR_POLYNOMIAL, RADIAL_POLYNOMIAL } CalibrationType_t;

    CalibrationOpts()
      : dataDir("data"),
      boardName(), cameraName(),
      seekTo( 0 ), intervalFrames( -1 ),
      calibFlags(0), randomize(-1),
      videoFile(),
      ignoreCache( false ), saveBoardPoses( false ), fixSkew( false ),
      calibType( ANGULAR_POLYNOMIAL )
  {;}

    bool validate( string &msg)
    {
      //if( boardName.empty() ) { msg = "Board name not set"; return false; }
      if( cameraName.empty() ) { msg = "Camera name not set"; return false; }

      return true;
    }

    string dataDir;
    string boardName;
    string cameraName;
    int seekTo, intervalFrames;
    int calibFlags, randomize;
    string videoFile, resultsFile;
    bool ignoreCache, retryUnregistered, saveBoardPoses, fixSkew;
    CalibrationType_t calibType;

    const string boardPath( void )
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string cachePath( void )
    { return dataDir + "/cache"; }

    const string imageCache( const Image &image )
    { return cachePath() + "/" + image.hash() + ".yml"; }

    const string tmpPath( const string &file )
    { return dataDir + "/tmp/" + file; }

    const string cameraPath( const string &filename )
    {
      string camDir(  dataDir + "/cameras/" + cameraName + "/" );
      if( !directory_exists( camDir ) ) mkdir_p( camDir );
      return camDir + filename;
    }



    //== Option parsing and help ==
    void help()
    {
      printf( "This is a camera calibration sample.\n"
          "Usage: calibration\n"
          "     -d <data directory>      # Specify top-level directory for board/camera/cache files.\n"
          "     --board,-b <board_name>    # Name of calibration pattern\n"
          "     --camera, -c <camera_name> # Name of camera\n"
          "     --ignore-cache, -i       # Ignore and overwrite files in cache\n"
          "     --retry-unregistered, -r   # Re-try to find the chessboard if the cache file is empty\n"
          "     --calibration-model, -m   # Set the distortion model to: angular, radial, radial8\n"
          "     --fix-skew, -k            # Fix skew (alpha) to 0\n"
          //     "     [-d <delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
          //     "                              # (used only for video capturing)\n"
          //     "     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
          //     "     [-op]                    # write detected feature points\n"
          //     "     [-oe]                    # write extrinsic parameters\n"
          //     "     [-zt]                    # assume zero tangential distortion\n"
          //     "     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
          //     "     [-p]                     # fix the principal point at the center\n"
          //     "     [-v]                     # flip the captured images around the horizontal axis\n"
          //     "     [-V]                     # use a video file, and not an image list, uses\n"
          //     "                              # [input_data] string for the video file name\n"
          //     "     [-su]                    # show undistorted images after calibration\n"
        "     [input_data]             # list of files to use\n"
        "\n" );
      //printf("\n%s",usage);
      //printf( "\n%s", liveCaptureHelp );
    }


    void parseOpts( int argc, char **argv )
    {
      static struct option long_options[] = {
        { "data-directory", true, NULL, 'd' },
        { "board", true, NULL, 'b' },
        { "camera", true, NULL, 'c' },
        { "calibation-model", true, NULL, 'm' },
        { "ignore-cache", false, NULL, 'R' },
        { "fix-skew", false, NULL, 'k'},
        { "retry-unregistered", false, NULL, 'r' },
        { "seek-to", true, NULL, 's' },
        { "interval-frames", true, NULL, 'i' },
        { "save-board-poses", no_argument, NULL, 'S' },
        { "random", required_argument, NULL, 'D' },
        { "results-file", required_argument, NULL, 'Z' },
        { "help", false, NULL, '?' },
        { 0, 0, 0, 0 }
      };


      if( argc < 2 )
      {
        help();
        exit(1);
      }

      int indexPtr;
      int optVal;
      string c;
      while( (optVal = getopt_long( argc, argv, "D:Z:RSrs:i:b:c:d:km:?", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'Z':
            resultsFile = optarg;
            break;
          case 'D':
            randomize = atoi( optarg );
            break;
          case 'd':
            dataDir = optarg;
            break;
          case 'b':
            boardName = optarg;
            break;
          case 'c':
            cameraName = optarg;
            break;
          case 'R':
            ignoreCache = true;
            break;
          case 'S':
            saveBoardPoses = true;
            break;
          case 's':
            seekTo = atoi( optarg );
            break;
          case 'i':
            intervalFrames = atoi( optarg );
            break;
          case 'k':
            calibFlags |= PinholeCamera::CALIB_FIX_SKEW;
            break;
          case 'm':
            c = optarg;
            if( c.compare("angular") == 0 ) {
              calibType = ANGULAR_POLYNOMIAL;
            } else if (c.compare("radial") == 0) {
              calibType = RADIAL_POLYNOMIAL;
            } else if (c.compare("radial8") == 0) {
              calibType = RADIAL_POLYNOMIAL;
              calibFlags |= CV_CALIB_RATIONAL_MODEL;
            } else {
              cerr <<  "Can't figure out the calibration model \"" <<  c << "\"";
              exit(-1);
            }
            break;
          case 'r':
            retryUnregistered = true;
            break;
          case '?': 
            help();
            break;
          default:
            exit(-1);

        }
      }

      videoFile = argv[ optind ];

      if( !file_exists( videoFile ) ) {
        cerr << "Can't open video file " << videoFile << endl;
        exit(-1);
      }

      string msg;
      if( !validate( msg ) ) {
        cout << "Error: " <<  msg << endl;
        exit(-1);
      }
    }


};


class ResultsFile {
  public:
    ResultsFile( const string &filename )
      : _db()
    {
      _isOpen = _db.open( filename );
    }

    kyotocabinet::BasicDB::Error error( void ) { return _db.error(); }
    bool isOpened( void ) const { return _isOpen; }

    HashDB _db;
    bool _isOpen;
};

class DetectionSet {
  public:

    typedef map< const int, Detection * > DetectionMap;

    DetectionSet() 
      : _detections(), _bitmask()
    {;}


    ~DetectionSet( void )
    {
      for( DetectionMap::iterator itr = _detections.begin(); itr != _detections.end(); ++itr ) delete itr->second;
    }

    //== Functions for initializing the DetectionSet ==
    //
    void addAll( DetectionDb &db )
    {
      DB::Cursor *cur = db.cursor();
      string key;
      while( cur->get_key( &key, false ) ) addDetection( db, stoi(key) );
      delete cur;
    }

    void addEveryNth( DetectionDb &db, int offset, int interval )
    {
      for( int i = offset; i < db.maxKey(); i += interval ) 
        addDetection( db,  i );
    }

    void addRandomSubset( DetectionDb &db, long int number )
    {
      vector< string > keys;

      DB::Cursor *cur = db.cursor();
      cur->jump();

      string key;
      while( cur->get_key( &key, false ) ) keys.push_back(key);
      delete cur;

      number = std::max( number, (long int)keys.size() );

      std::random_shuffle( keys.begin(), keys.end() );
      keys.resize( number );
      _bitmask.clear();
      _bitmask.resize( number, false );

      for( vector< string >::iterator itr = keys.begin(); itr != keys.end(); ++itr ) addDetection( db, stoi(key) );
    }

    void addDetection( DetectionDb &db, const int frame )
    { 
      Detection *detection = db.load( frame );
        if( detection ) {
          _detections.insert( make_pair( frame, detection ) );
          _bitmask[frame] = true;
        }
    }


    //==

    int imageObjectPoints( ImagePointsVecVec &imgPts, ObjectPointsVecVec &objPts )
    {
      int count = 0;
      for( DetectionMap::iterator itr = _detections.begin(); itr != _detections.end(); ++itr ) {
        Detection &det( *(itr->second) );
        if( det.corners.size() > 3 ) {
          imgPts.push_back( det.points );
          objPts.push_back( det.corners );
          count += det.corners.size();
        }
      }

      return count;
    }


    DetectionMap _detections;
    vector<bool> _bitmask;
};





static double computeReprojectionErrors(
    const DistortionModel *dist,
    const Distortion::ObjectPointsVecVec &objectPoints,
    const Distortion::ImagePointsVecVec &imagePoints,
    const Distortion::RotVec &rvecs, 
    const Distortion::TransVec &tvecs,
    vector<float>& perViewErrors )
{
  ImagePointsVec reprojImgPoints;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for( i = 0; i < (int)objectPoints.size(); i++ )
  {
    if( objectPoints[i].size() > 0 ) {
      dist->projectPoints( Mat( objectPoints[i] ), rvecs[i], tvecs[i], reprojImgPoints );

      err = norm(Mat(imagePoints[i]), Mat(reprojImgPoints), CV_L2);
      int n = (int)objectPoints[i].size();
      perViewErrors[i] = (float)std::sqrt(err*err/n);
      totalErr += err*err;
      totalPoints += n;
    }
  }

  return std::sqrt(totalErr/totalPoints);
}

static void saveCameraParams( const string& filename,
    Size imageSize, const Board &board,
    const vector< Image > &imagesUsed,
    float aspectRatio, int flags,
    const DistortionModel *model, 
    const vector<Vec3d>& rvecs, const vector<Vec3d>& tvecs,
    const vector<float>& reprojErrs,
    const Distortion::ImagePointsVecVec &imagePoints,
    double totalAvgErr )
{

  FileStorage out( filename, FileStorage::WRITE );

  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  char buf[1024];
  strftime( buf, sizeof(buf)-1, "%c", t2 );

  out << "calibration_time" << buf;

  if( !rvecs.empty() || !reprojErrs.empty() )
    out << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
  out << "image_width" << imageSize.width;
  out << "image_height" << imageSize.height;
  out << "board_name" << board.name;
  out << "board_width" << board.size().width;
  out << "board_height" << board.size().height;
  out << "square_size" << board.squareSize;

  if( flags & CV_CALIB_FIX_ASPECT_RATIO )
    out << "aspectRatio" << aspectRatio;

  if( flags != 0 )
  {
    sprintf( buf, "flags: %s%s%s%s",
        flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
        flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
        flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
        flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
    cvWriteComment( *out, buf, 0 );
  }

  out << "flags" << flags;

  model->write( out );

  out << "avg_reprojection_error" << totalAvgErr;
  if( !reprojErrs.empty() )
    out << "per_view_reprojection_errors" << Mat(reprojErrs);

  //  if( !rvecs.empty() && !tvecs.empty() )
  //  {
  //    CV_Assert(rvecs[0].type() == tvecs[0].type());
  //    Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
  //    for( int i = 0; i < (int)rvecs.size(); i++ )
  //    {
  //      Mat r = bigmat(Range(i, i+1), Range(0,3));
  //      Mat t = bigmat(Range(i, i+1), Range(3,6));
  //
  //      CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
  //      CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
  //      //*.t() is MatExpr (not Mat) so we can use assignment operator
  //      r = rvecs[i].t();
  //      t = tvecs[i].t();
  //    }
  //    cvWriteComment( *out, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
  //    out   << "extrinsic_parameters" << bigmat;
  //  }

  //  out << "images_used" << "[";
  //  for( vector<Image>::const_iterator img = imagesUsed.begin(); img < imagesUsed.end(); ++img ) {
  //    out << img->fileName();
  //  }
  //  out << "]";

  if( !imagePoints.empty() )
  {
    Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
    for( int i = 0; i < (int)imagePoints.size(); i++ )
    {
      Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
      Mat imgpti(imagePoints[i]);
      imgpti.copyTo(r);
    }
    out << "image_points" << imagePtMat;
  }
}


static string mkCameraFileName( const string &cameraName)
{
  char strtime[32], buffer[80];
  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  strftime( strtime, 32, "%y%m%d_%H%M%S", t2 );
  snprintf( buffer, 79, "%s_%s.yml", cameraName.c_str(), strtime );
  return  string( buffer );
}


int main( int argc, char** argv )
{

  CalibrationOpts opts;

  opts.parseOpts( argc, argv );

  Board *board = Board::load( opts.boardPath(), opts.boardName );

  Size imageSize;
  float aspectRatio = 1.f;
  bool writeExtrinsics = false, writePoints = false;

  Distortion::ImagePointsVecVec imagePoints;
  Distortion::ObjectPointsVecVec objectPoints;

  DetectionDb db;
  DetectionSet detSet;

  if( ! db.open( opts.cachePath(), opts.videoFile, 
        ( opts.saveBoardPoses == true ? true : false ) ) ) {
    cerr << "Open error: " << db.error().name() << endl;
    return -1;
  }

  string videoSource( opts.videoFile );
  VideoCapture vid( videoSource );
  if( !vid.isOpened() ) {
    cerr << "Couldn't open video source \"" << videoSource << "\"" << endl;
    return -1;
  }
  int vidLength = vid.get( CV_CAP_PROP_FRAME_COUNT );

  // Get image size
  imageSize = Size( vid.get( CV_CAP_PROP_FRAME_WIDTH ), vid.get(CV_CAP_PROP_FRAME_HEIGHT ) );

  vector< pair< string, Detection * > > detections;

  if( opts.randomize != 0 ) {
    detSet.addRandomSubset( db, opts.randomize );
  } else if( opts.intervalFrames > 0 ) {
    detSet.addEveryNth( db,  opts.seekTo, opts.intervalFrames );
  } else {
    detSet.addAll( db );
  }


  int count = detSet.imageObjectPoints( imagePoints, objectPoints );


  cout << "Using " << count << " points from " << imagePoints.size() << " images" << endl;

  if( imagePoints.size() < 3 ) {
    cerr << "Not enough images.  Stopping." << endl;
    exit(-1);
  }

  vector< Vec3d > rvecs, tvecs;

  int flags =  opts.calibFlags;

  DistortionModel *distModel = NULL;
  switch( opts.calibType ) {
    case CalibrationOpts::ANGULAR_POLYNOMIAL:
      distModel = new Distortion::AngularPolynomial;
      break;
    case CalibrationOpts::RADIAL_POLYNOMIAL:
      distModel = new Distortion::RadialPolynomial;
      break;
  }

  if( !distModel ) {
    cerr << "Something went wrong choosing a distortion model." << endl;
    exit(-1);
  }

  double rms = distModel->calibrate( objectPoints, imagePoints, 
      imageSize, rvecs, tvecs, flags );

  //  ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
  printf("RMS error reported by calibrateCamera: %g\n", rms);

  //  bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

  bool ok = true;

  vector<float> reprojErrs;
  double totalAvgErr = 0;
  totalAvgErr = computeReprojectionErrors(distModel, objectPoints, imagePoints, rvecs, tvecs, reprojErrs );

  if( ok ) {
    string cameraFile( opts.cameraPath(mkCameraFileName( opts.cameraName ) ) );
    cout << "Writing results to " << cameraFile << endl;

    vector<Image> imagesUsed;
    saveCameraParams( cameraFile, imageSize,
        *board, imagesUsed, aspectRatio,
        flags, distModel,
        writeExtrinsics ? rvecs : vector<Vec3d>(),
        writeExtrinsics ? tvecs : vector<Vec3d>(),
        writeExtrinsics ? reprojErrs : vector<float>(),
        writePoints ? imagePoints : Distortion::ImagePointsVecVec(),
        totalAvgErr );

    if( opts.saveBoardPoses ) {
      for( int i = 0; i < detections.size(); ++i ) {
        Detection *det = detections[i].second;
        det->rot = rvecs[i];
        det->trans = tvecs[i];

        if( ! db.update( detections[i].first, *det ) )
          cerr << "Trouble saving updated poses: " << db.error().name() << endl;
      }
    }
  }

  for( int i = 0; i < detections.size(); ++i ) {
    delete detections[i].second;
  }



  printf("%s. avg reprojection error = %.2f\n",
      ok ? "Calibration succeeded" : "Calibration failed",
      totalAvgErr);

  delete distModel;
  delete board;

  return 0;
}