
#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <mutex>

#include <thrust/for_each.h>
#include <thrust/host_vector.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <kchashdb.h>

#include "board.h"
#include "detection.h"
#include "file_utils.h"
#include "trendnet_time_code.h"

using namespace std;
using namespace cv;
using kyotocabinet::HashDB;

struct BuildDbOpts {
  public:
    BuildDbOpts()
      : seekTo(-1), intervalFrames(-1), waitKey( 1 ), 
      intervalSeconds( -1 ),
      dataDir("data"),
      boardName(), 
      doBenchmark(),
      doRewrite( false ),
      doDisplay( false ), yes( false ),
      verb( NONE )
  {;}


    typedef enum {  NONE = -1} Verbs;

    int seekTo, intervalFrames, waitKey;
    float intervalSeconds;
    string dataDir;
    string boardName, doBenchmark;
    bool doRewrite, doDisplay, yes;
    Verbs verb;

    string inFile;

    const string boardPath( void )
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string tmpPath( const string &file )
    { return dataDir + "/tmp/" + file; }

    const string cachePath( const string &file = "" ) const
    { return dataDir + "/cache/" + file; }

    //== Option parsing and help ==

    string help( void )
    {
      stringstream strm;
      const int w = 20;

      strm <<  "This is a tool for extracting images from a video file." << endl;
      strm << setw( w ) <<  "--data-directory, -d" << "Set location of data directory." << endl;
      strm << setw( w ) << "--do-display, -x" << "Do display video" << endl;
      strm << setw( w ) << "--do-benchmark [file]" << "Save benchmarking information to [file]" << endl;
      strm << setw( w ) << "--do-rewrite" << "Extract features even if it already exists in database" << endl;
      //    "Usage: calibration\n"
      //    "     -d <data directory>      # Specify top-level directory for board/camera/cache files.\n"
      //    "     --board,-b <board_name>    # Name of calibration pattern\n"
      //    "     --camera, -c <camera_name> # Name of camera\n"
      //    "     --seek-to <frame>          # Seek to specified frame before starting\n"
      //    //     "     [-d <delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
      //    //     "                              # (used only for video capturing)\n"
      //    //     "     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
      //    //     "     [-op]                    # write detected feature points\n"
      //    //     "     [-oe]                    # write extrinsic parameters\n"
      //    //     "     [-zt]                    # assume zero tangential distortion\n"
      //    //     "     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
      //    //     "     [-p]                     # fix the principal point at the center\n"
      //    //     "     [-v]                     # flip the captured images around the horizontal axis\n"
      //    //     "     [-V]                     # use a video file, and not an image list, uses\n"
      //    //     "                              # [input_data] string for the video file name\n"
      //    //     "     [-su]                    # show undistorted images after calibration\n"
      //    "     [input_data]             # list of files to use\n"
      //    "\n" );
      //printf("\n%s",usage);
      //printf( "\n%s", liveCaptureHelp );

      return strm.str();
    }

    string unknownOption( int opt )
    {
      stringstream strm;
      strm << "Unknown option \"" << opt << "\"";
      return strm.str();
    }


    bool parseOpts( int argc, char **argv, string &msg )
    {
      static struct option long_options[] = {
        { "data-directory", true, NULL, 'd' },
        { "board", true, NULL, 'b' },
        { "seek-to", true, NULL, 's' },
        { "interval-frames", true, NULL, 'i' },
        { "interval-seconds", true, NULL, 'I' },
        { "do-rewrite", no_argument, NULL, 'R' },
        { "do-display", no_argument,  NULL, 'x' },
        { "do-benchmark", required_argument, NULL, 'K' },
        { "yes", no_argument, NULL, 'y' },
        { "help", false, NULL, '?' },
        { 0, 0, 0, 0 }
      };


      if( argc < 2 )
      {
        msg =  help();
        return false;
      }

      int indexPtr;
      int optVal;
      while( (optVal = getopt_long( argc, argv, "b:c:d:K:I:i:Rs:x?", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'd':
            dataDir = optarg;
            break;
          case 'b':
            boardName = optarg;
            break;
          case 'i':
            intervalFrames = atoi( optarg );
            break;
          case 'I':
            intervalSeconds = atof( optarg );
            break;
          case 'K':
            doBenchmark = optarg;
            break;
          case 'R':
            doRewrite = true;
            break;
          case 's':
            seekTo = atoi( optarg );
            break;
          case 'x':
            doDisplay = true;
            break;
          case 'y':
            yes = true;
            break;
          case '?': 
            msg = help();
            return false;
            break;
          default:
            msg = unknownOption( optopt );
            return false;
        }
      }

      if( optind == argc )
      {
        msg = help();
        return false;
      }

      inFile = argv[optind];

      if( intervalFrames > 0 && intervalSeconds > 0 ) {
        msg = "Can't specify both interval frames and interval seconds at the same time.";
        return false;
      }

      return validate( msg );
    }

    bool validate( string &msg )
    {
      return true;
    }

};




class BuildDbMain 
{
  public:
    BuildDbMain( BuildDbOpts &options )
      : opts( options ), _benchmark()
    {;}

    ~BuildDbMain( void )
    {
      if( _benchmark.is_open() ) _benchmark.close();
    }

    int run( void ) {
      if( opts.doDisplay ) namedWindow( "BuildDb" );

      return doBuildDb();
    }

    int doBuildDb( void )
    {
      board = Board::load( opts.boardPath(), opts.boardName );
      if( !board ) {
        cerr << "Couldn't open board from " << opts.boardPath() << endl;
        return -1;
      }

      string videoSource( opts.inFile );
      VideoCapture vid( videoSource );
      if( !vid.isOpened() ) {
        cerr << "Couldn't open video source \"" << videoSource << "\"" << endl;
        return -1;
      }

      mkdir_p( opts.cachePath() );
      if( ! db.open( opts.cachePath(), opts.inFile, true ) ) {
        cerr << "Error opening database file: " << db.error().name() << endl;
        return -1;
      }

      double vidLength = vid.get( CV_CAP_PROP_FRAME_COUNT );

      if( opts.intervalSeconds > 0 ) 
        opts.intervalFrames = opts.intervalSeconds * vid.get( CV_CAP_PROP_FPS );

      TimingVecType timingData;
      FrameVecType frames;
      Mat img;
      while( vid.read( img ) ) {
        int currentFrame = vid.get( CV_CAP_PROP_POS_FRAMES );
        cout << currentFrame << endl;
        if( !db.has( currentFrame ) || opts.doRewrite ) {
          frames.push_back( new Frame( currentFrame, img ) );
        }

        if( opts.intervalFrames > 1 ) {
          int destFrame = currentFrame + opts.intervalFrames - 1;
          if( destFrame < vidLength ) {
            vid.set( CV_CAP_PROP_POS_FRAMES, currentFrame + opts.intervalFrames - 1 );
          } else {
            // Done
            break;
          }
        }

        if( frames.size() > 5 ) processFrames( frames, timingData );
        
      }

      processFrames( frames, timingData );

      if( opts.doBenchmark.size() > 0 ) saveBenchmarks( timingData );

      return 0;
    }

    struct Frame {
      Frame( void ) 
        : frame(0), img(0,0,CV_64F) {;}

      Frame( const Frame &other ) 
        : frame( other.frame ), img() 
      { other.img.copyTo( img ); }

      Frame( int _f, Mat _m )
        : frame(_f), img() 
      { _m.copyTo( img ); }

      void operator=( const Frame &other )
      {
        frame = other.frame;
        other.img.copyTo( img );
      }

      int frame;
      Mat img;
    };


    typedef thrust::host_vector< Frame * > FrameVecType;
    typedef thrust::host_vector< pair< int, int64 > > TimingVecType;


    void processFrames( FrameVecType &frames, TimingVecType &timingData )
    {
          timingData.resize( timingData.size() + frames.size() );
          thrust::transform( frames.begin(), frames.end(), timingData.end(), AprilTagDetectorFunctor( db, board ) );
          for( int i = 0; i < frames.size(); ++i ) delete frames[i];
          frames.clear();
    }

    struct AprilTagDetectorFunctor {
      public:
        AprilTagDetectorFunctor( DetectionDb &db, Board *board )
          : _db(db), _board(board)
        {;}

        DetectionDb &_db;
        Board *_board;

        pair< int, int64 > operator()( const Frame *p )
        {
          Detection *detection = NULL;

          cout << "Extracting from " << p->frame << ". ";

          Mat grey;
          cvtColor( p->img, grey, CV_BGR2GRAY );
          vector<Point2f> pointbuf;

          int64 before = getTickCount();
          detection = _board->detectPattern( grey, pointbuf );
          int64 elapsed = getTickCount() - before;

          cout << p->frame << ": " << detection->size() << " features" << endl;

          _db.save( p->frame, *detection);

          delete detection;

          return make_pair( detection->size(), elapsed );
        }
    };


    void saveBenchmarks( const TimingVecType &timingData )
    {
      if( !_benchmark.is_open() ) _benchmark.open( opts.doBenchmark, ios_base::trunc );
      for( TimingVecType::const_iterator itr = timingData.begin();  itr != timingData.end(); ++itr )
        addBenchmark( (*itr).first, (*itr).second );
    }


    void addBenchmark( int numPoints, int64 ticks )
    {
      if( !_benchmark.is_open() ) _benchmark.open( opts.doBenchmark, ios_base::trunc );

      _benchmark << numPoints << ',' << ticks / getTickFrequency() << endl;
    }



  private:
    BuildDbOpts opts;

    DetectionDb db;
    Board *board;

    ofstream _benchmark;
};



int main( int argc, char **argv ) 
{

  BuildDbOpts opts;
  string msg;
  if( !opts.parseOpts( argc, argv, msg ) ) {
    cout << msg << endl;
    exit(-1);
  }

  BuildDbMain main( opts );

  exit( main.run() );

}

