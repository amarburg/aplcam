//#include "opencv2/core/core.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/highgui/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <getopt.h>

#include <iostream>

#ifdef USE_TBB
#include "tbb/tbb.h"
using namespace tbb;
#endif

#include "glog/logging.h"

#include "file_utils.h"
#include "board.h"
#include "detection.h"
#include "detection_set.h"
#include "image.h"

#include "distortion_model.h"
using namespace Distortion;

#include "calibration_db.h"
#include "calibration_opts_common.h"
#include "calibrator.h"
using namespace AplCam;

#include "video_splitters/video_splitter_opts.h"
#include "video_splitters/video_splitters.h"
using namespace AplCam::VideoSplitters;



using namespace cv;
using namespace std;

using kyotocabinet::HashDB;
using kyotocabinet::DB;

class CalibrationOpts : public AplCam::CalibrationOptsCommon {

  public:

    CalibrationOpts()
      : CalibrationOptsCommon(), 
      calibrationDb(),
      videoFile(),
      fixSkew( true ), overwriteDb( false )
  {;}

    string calibrationDb;
    string videoFile;

    bool fixSkew, overwriteDb;

    CalibrationType_t calibType;
//    SplitterType_t splitter;
//
//    IntervalSplitterOpts intervalSplitterOpts;
//    RandomSplitterOpts randomSplitterOpts;


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


    bool parseOpts( int argc, char **argv, string &msg )
    {
      stringstream msgstrm;

      static struct option long_options[] = {
        { "data-directory", true, NULL, 'd' },
        { "board", true, NULL, 'b' },
        { "camera", true, NULL, 'c' },
        { "calibation-model", true, NULL, 'm' },
        { "fix-skew", false, NULL, 'k'},
        { "calibration-db", required_argument, NULL, 'Z' },
        { "help", false, NULL, '?' },
        { 0, 0, 0, 0 }
      };


      if( argc < 2 )
      {
        help();
        return false;
      }

      int indexPtr;
      int optVal;
      string c;

      // The '+' option ensures it stops on the first non-conforming option. Required for the
      //   cmd opt1 opt2 opt3 verb verb_opt1 files ...
      // pattern I'm using
      while( (optVal = getopt_long( argc, argv, "+yZ:RSrb:c:d:km:?", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'Z':
            calibrationDb = optarg;
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
          case 'k':
            calibFlags |= PinholeCamera::CALIB_FIX_SKEW;
            break;
          case 'y':
            overwriteDb = true;
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
              return false;
            }
            break;
          case '?': 
            help();
            break;
          default:
            return false;

        }
      }

      if( optind >= argc ) {
        msg = "Must specify splitter type on command line";
        return false;
      }

      if( optind >= argc ) {
        msg = "Must specify video file on command line";
        return false;
      }

      videoFile = argv[ optind ];

      if( !file_exists( videoFile ) ) {
        cerr << "Can't open video file " << videoFile << endl;
        return false;
      }

      if( !validate( msg ) ) return false;

      return true;
    }


    virtual bool validate( string &msg )
    {
      if( !calibrationDb.empty() ) {
        if( !calibrationFile.empty() ) {
          msg = "Can't set both calibration file and calibration db";
          return false;
        }
      }

      if( !CalibrationOptsCommon::validate( msg ) ) return false;

      // The super will auto-fill calibrationFile if not set
      if( !calibrationDb.empty() ) calibrationFile.clear();

      return true;
    }

};

struct RandomKeyCounter {
  RandomKeyCounter( int i ) : _count(i) {;}
  int _count;

  bool operator()( const string &key )
  {
    int c;
    int found = sscanf( key.c_str(), "random(%d)", &c );
    if( found != 1 ) return false;

    return c == _count;
  }
};


struct CalibrateFunctor {
  public:
    CalibrateFunctor( const CalibrationOpts &opts, Size &imageSize, CalibrationDb &db, vector< DetectionSet * > &detSets )
      : _opts( opts ), _imageSize( imageSize ), _db( db ), _detSets( detSets )
    {;}

    const CalibrationOpts &_opts;
    const Size &_imageSize;
    CalibrationDb &_db;
    vector< DetectionSet *> _detSets;

#ifdef USE_TBB
    void operator()( const blocked_range<size_t> &r ) const {
      size_t end = r.end();
      for( size_t i = r.begin(); i != end; ++i ) {
#else
        void operator()(void ) const {
          size_t end = _detSets.size();
          for( size_t i = 0; i != end; ++i ) {
#endif

            Calibrator cal( _opts, *_detSets[i], _imageSize );
            cal.run();

            if( cal.result.success ) {
              cal.saveDb( _db );
            }



          }
        }
      };



int main( int argc, char** argv )
{

   google::InitGoogleLogging("video_calibration_permutation");

  CalibrationOpts opts;

  string optsMsg;
  if( !opts.parseOpts( argc, argv, optsMsg ) ) {
    cout << optsMsg << endl;
    exit(-1);
  }

  DetectionDb db;
  if( ! db.open( opts.cachePath(), opts.videoFile,  false ) ) {
    cerr << "Error opening db error: " << db.error().name() << endl;
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
  Size imageSize = Size( vid.get( CV_CAP_PROP_FRAME_WIDTH ), vid.get(CV_CAP_PROP_FRAME_HEIGHT ) );

  const int spacing = 100;
  const int minImages = 10;
  const int maxReps = 5;
  const int maxImages = vidLength;

  CalibrationDb calDb( opts.calibrationDb );
  if( !calDb.isOpened() ) {
    cerr << "Couldn't open database!" << endl;
    exit(-1);
  }

  vector<string> keys;
  calDb.findKeysStartingWith( "random", keys );

  // For simplicity, just configure in code.
  for( int i = minImages; i < maxImages; i+= spacing ) {
    int maxSets = std::min( maxReps, (vidLength-i) );


    // Parse out the keys with the correct length
    int existing = std::count_if( keys.begin(), keys.end(), RandomKeyCounter( i ) );

    size_t todo = maxSets - existing;


    cout << "For random set of " << i << " images, found " << existing << " sets, must do " << todo << endl;

    if( todo == 0 ) continue;

    vector< DetectionSet * > detSets;
    for( size_t j = 0; detSets.size() < todo; ++j ) {
      DetectionSet *detSet = new DetectionSet;
      RandomVideoSplitter( i ).generate( db, *detSet );

      if( calDb.has( detSet->name() ) ) continue;

      detSets.push_back( detSet );
    }


    // Now run each one
    CalibrateFunctor func( opts, imageSize, calDb, detSets );
#ifdef USE_TBB
    parallel_for( blocked_range<size_t>(0,detSets.size()), func );
#else
    func();
#endif

    // Delete all detection sets
    for( size_t j = 0; j < detSets.size(); ++j ) delete detSets[j];

  }

  return 0;
}