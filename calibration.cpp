#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <getopt.h>
#include <sys/stat.h>

#include <iostream>

using namespace cv;
using namespace std;

const char * usage =
" \nexample command line for calibration from a live feed.\n"
"   calibration  -w 4 -h 5 -s 0.025 -o camera.yml -op -oe\n"
" \n"
" example command line for calibration from a list of stored images:\n"
"   imagelist_creator image_list.xml *.png\n"
"   calibration -w 4 -h 5 -s 0.025 -o camera.yml -op -oe image_list.xml\n"
" where image_list.xml is the standard OpenCV XML/YAML\n"
" use imagelist_creator to create the xml or yaml list\n"
" file consisting of the list of strings, e.g.:\n"
" \n"
"<?xml version=\"1.0\"?>\n"
"<opencv_storage>\n"
"<images>\n"
"view000.png\n"
"view001.png\n"
"<!-- view002.png -->\n"
"view003.png\n"
"view010.png\n"
"one_extra_view.jpg\n"
"</images>\n"
"</opencv_storage>\n";

enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };


static bool file_exists( const string &infile ) {
  struct stat buffer;   
  return (stat(infile.c_str(), &buffer) == 0 );
}

static bool directory_exists( const string &infile ) {
  struct stat buffer;   
  return (stat(infile.c_str(), &buffer) == 0  && (buffer.st_mode & S_IFDIR));
}

static void mkdir_p( const string &dir )
{
  char tmp[256];
  char *p = NULL;
  size_t len;

  snprintf(tmp, sizeof(tmp),"%s",dir.c_str() );
  len = strlen(tmp);

  bool finalSep = (tmp[len - 1] == '/');

  for(p = tmp + 1; *p; p++)
    if(*p == '/') {
      *p = 0;
      mkdir(tmp, S_IRWXU);
      *p = '/';
    }

  if( finalSep ) {
    tmp[len-1] = 0;
    mkdir(tmp, S_IRWXU);
    tmp[len-1] = '/';
  }
}

class CalibrationOpts {
  public:
    CalibrationOpts()
      : dataDir("data"),
      boardName(), 
      inFiles(),
      ignoreCache( false )
  {;}

    bool validate( string &msg)
    {
      if( boardName.empty() ) { msg = "Board name not set"; return false; }
      if( cameraName.empty() ) { msg = "Camea name not set"; return false; }

      return true;
    }

    string dataDir;
    string boardName;
    string cameraName;
    vector< string > inFiles;
    bool ignoreCache;

    const string boardPath( void )
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string cachePath( void )
    { return dataDir + "/cache"; }

    const string imageCache( const string &image )
    { return cachePath() + "/" + image + ".yml"; }

    const string tmpPath( const string &file )
    { return dataDir + "/tmp/" + file; }

    const string cameraPath( const string &filename )
    {
      string camDir(  dataDir + "/cameras/" + cameraName + "/" );
      if( !directory_exists( camDir ) ) mkdir_p( camDir );
      return camDir + filename;
    }

};



class Board {
  public:
    Board( Pattern pat, int w, int h, float squares )
      : pattern(pat), width(w), height(h), squareSize( squares )
    {;}

    string name;
    Pattern pattern;
    int width, height;
    float squareSize;

    Size size( void ) const { return Size( width,height ); }

  private:
};


class BoardFactory {
  public:
    static Board load( const string &infile )
    {
      FileStorage fs( infile, FileStorage::READ );
      if( ! fs.isOpened() ) {
        cout << "Couldn't open board file \"" << infile << "\"" << endl;
        exit(-1);
      }

      string type_s;
      int width, height;
      float squares;
      Pattern type;

      fs["type"] >> type_s;
      fs["width"] >> width;
      fs["height"] >> height;
      fs["squareSize"] >> squares;

      if( type_s.compare("chessboard" ) ) {
        type = CHESSBOARD;
      } else {
        cout << "Don't know how to handle board type \"" << type_s << "\"" << endl;
        exit(-1);
      }

      return Board( type, width, height, squares );
    }
  private:

};


class Image {
  public:
    Image( const string &filename, Mat &img )
      : _fileName( filename ), _img( img )
    {;}

    const string &fileName( void ) const { return _fileName; }
    const Mat    &img( void )      const { return _img; }
    vector< Point2f > points;

    bool loadCache( const string &cacheFile )
    {
      if( !file_exists( cacheFile ) ) {
        cout << "Unable to find cache file \"" << cacheFile << "\"" << endl;
        return false;
      }

      FileStorage fs( cacheFile, FileStorage::READ );

      // Load and validate data
      Mat pts;
      fs["points"] >> pts;

      // Should be able to do this in-place, but ...
      for( int i = 0; i < pts.rows; ++i ) {
points.push_back( Point2f(pts.at<float>(i,0), pts.at<float>(i,1) ) );
      }

      return true;
    }

    void writeCache( const string &cacheFile )
    {
      mkdir_p( cacheFile );

      FileStorage fs( cacheFile, FileStorage::WRITE );

      fs << "points" << Mat( points );

    }

    string basename( void )
    {
      size_t sep = _fileName.find_last_of( '/' );
      if( sep == string::npos )
        return _fileName;

        return String( _fileName, sep+1 );
    }

  private:

    string _fileName;
    Mat _img;
};


const char* liveCaptureHelp =
"When the live video from camera is used as input, the following hot-keys may be used:\n"
"  <ESC>, 'q' - quit the program\n"
"  'g' - start capturing images\n"
"  'u' - switch undistortion on/off\n";

static void help()
{
  printf( "This is a camera calibration sample.\n"
      "Usage: calibration\n"
      "     -d <data directory>      # Specify top-level directory for board/camera/cache files.\n"
      "     --board,-b <board_name>    # Name of calibration pattern\n"
      "     --camera, -c <camera_name> # Name of camera\n"
      "     --ignore-cache, -i       # Ignore and overwrite files in cache\n"
      "     [-d <delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
      "                              # (used only for video capturing)\n"
      "     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
      "     [-op]                    # write detected feature points\n"
      "     [-oe]                    # write extrinsic parameters\n"
      "     [-zt]                    # assume zero tangential distortion\n"
      "     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
      "     [-p]                     # fix the principal point at the center\n"
      "     [-v]                     # flip the captured images around the horizontal axis\n"
      "     [-V]                     # use a video file, and not an image list, uses\n"
      "                              # [input_data] string for the video file name\n"
      "     [-su]                    # show undistorted images after calibration\n"
      "     [input_data]             # input data, one of the following:\n"
      "                              #  - text file with a list of the images of the board\n"
      "                              #    the text file can be generated with imagelist_creator\n"
      "                              #  - name of video file with a video of the board\n"
      "                              # if input_data not specified, a live view from the camera is used\n"
      "\n" );
  printf("\n%s",usage);
  printf( "\n%s", liveCaptureHelp );
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

static double computeReprojectionErrors(
    const vector<vector<Point3f> >& objectPoints,
    const vector<vector<Point2f> >& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    vector<float>& perViewErrors )
{
  vector<Point2f> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for( i = 0; i < (int)objectPoints.size(); i++ )
  {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
        cameraMatrix, distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float)std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }

  return std::sqrt(totalErr/totalPoints);
}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
  corners.resize(0);

  switch(patternType)
  {
    case CHESSBOARD:
    case CIRCLES_GRID:
      for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
          corners.push_back(Point3f(float(j*squareSize),
                float(i*squareSize), 0));
      break;

    case ASYMMETRIC_CIRCLES_GRID:
      for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
          corners.push_back(Point3f(float((2*j + i % 2)*squareSize),
                float(i*squareSize), 0));
      break;

    default:
      CV_Error(CV_StsBadArg, "Unknown pattern type\n");
  }
}

static bool runCalibration( vector<vector<Point2f> > imagePoints,
    Size imageSize, Size boardSize, Pattern patternType,
    float squareSize, float aspectRatio,
    int flags, Mat& cameraMatrix, Mat& distCoeffs,
    vector<Mat>& rvecs, vector<Mat>& tvecs,
    vector<float>& reprojErrs,
    double& totalAvgErr)
{
  cameraMatrix = Mat::eye(3, 3, CV_64F);
  if( flags & CV_CALIB_FIX_ASPECT_RATIO )
    cameraMatrix.at<double>(0,0) = aspectRatio;

  distCoeffs = Mat::zeros(8, 1, CV_64F);

  vector<vector<Point3f> > objectPoints(1);
  calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

  objectPoints.resize(imagePoints.size(),objectPoints[0]);

  double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
      distCoeffs, rvecs, tvecs, flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
  ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
  printf("RMS error reported by calibrateCamera: %g\n", rms);

  bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

  totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
      rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

  return ok;
}


static void saveCameraParams( const string& filename,
    Size imageSize, const Board &board,
    const vector< Image > &imagesUsed,
    float aspectRatio, int flags,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const vector<float>& reprojErrs,
    const vector<vector<Point2f> >& imagePoints,
    double totalAvgErr )
{
  //FileStorage existing( filename, FileStorage::READ || FileStorage::MEMORY );

  //int idx = 0;
  //if( !existing.isOpened() || existing.root().type() == FileNode::NONE  ) {
  //cout << "Creating new file for camera" <<endl;
  //} else {

  //}

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

  out << "camera_matrix" << cameraMatrix;
  out << "distortion_coefficients" << distCoeffs;

  out << "avg_reprojection_error" << totalAvgErr;
  if( !reprojErrs.empty() )
    out << "per_view_reprojection_errors" << Mat(reprojErrs);

  if( !rvecs.empty() && !tvecs.empty() )
  {
    CV_Assert(rvecs[0].type() == tvecs[0].type());
    Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
    for( int i = 0; i < (int)rvecs.size(); i++ )
    {
      Mat r = bigmat(Range(i, i+1), Range(0,3));
      Mat t = bigmat(Range(i, i+1), Range(3,6));

      CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
      CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
      //*.t() is MatExpr (not Mat) so we can use assignment operator
      r = rvecs[i].t();
      t = tvecs[i].t();
    }
    cvWriteComment( *out, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
  out   << "extrinsic_parameters" << bigmat;
  }

  out << "images_used" << "[";
  for( vector<Image>::const_iterator img = imagesUsed.begin(); img < imagesUsed.end(); ++img ) {
    out << img->fileName();
  }
  out << "]";

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

//static bool readStringList( const string& filename, vector<string>& l )
//{
//  l.resize(0);
//  FileStorage fs(filename, FileStorage::READ);
//  if( !fs.isOpened() )
//    return false;
//  FileNode n = fs.getFirstTopLevelNode();
//  if( n.type() != FileNode::SEQ )
//    return false;
//  FileNodeIterator it = n.begin(), it_end = n.end();
//  for( ; it != it_end; ++it )
//    l.push_back((string)*it);
//  return true;
//}


static bool runAndSave(const string& outputFilename,
    const vector<vector<Point2f> >& imagePoints,
    Size imageSize, const Board &board,
    const vector<Image> &imagesUsed,
    float aspectRatio, int flags, Mat& cameraMatrix,
    Mat& distCoeffs, bool writeExtrinsics, bool writePoints )
{
  vector<Mat> rvecs, tvecs;
  vector<float> reprojErrs;
  double totalAvgErr = 0;

  bool ok = runCalibration(imagePoints, imageSize, board.size(), board.pattern, board.squareSize,
      aspectRatio, flags, cameraMatrix, distCoeffs,
      rvecs, tvecs, reprojErrs, totalAvgErr);
  printf("%s. avg reprojection error = %.2f\n",
      ok ? "Calibration succeeded" : "Calibration failed",
      totalAvgErr);

  if( ok )
    saveCameraParams( outputFilename, imageSize,
        board, imagesUsed, aspectRatio,
        flags, cameraMatrix, distCoeffs,
        writeExtrinsics ? rvecs : vector<Mat>(),
        writeExtrinsics ? tvecs : vector<Mat>(),
        writeExtrinsics ? reprojErrs : vector<float>(),
        writePoints ? imagePoints : vector<vector<Point2f> >(),
        totalAvgErr );
  return ok;
}



void parseOpts( int argc, char **argv, CalibrationOpts &opts )
{
  static struct option long_options[] = {
    { "data_directory", true, NULL, 'd' },
    { "board", true, NULL, 'b' },
    { "camera", true, NULL, 'c' },
    { "ignore-cache", false, NULL, 'i' },
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
  while( (optVal = getopt_long( argc, argv, "ib:c:d:?", long_options, &indexPtr )) != -1 ) {
    switch( optVal ) {
      case 'd':
        opts.dataDir = optarg;
        break;
      case 'b':
        opts.boardName = optarg;
        break;
      case 'c':
        opts.cameraName = optarg;
        break;
      case 'i':
        opts.ignoreCache = true;
        break;
      case '?': 
        help();
        break;
      default:
        exit(-1);

    }
  }

  if( optind == argc )
  {
    cout << "No input files specified." << endl;
    exit(-1);
  }

  for( int i = optind; i < argc; ++i ) {
    string infile( argv[i] );

    if( !file_exists( infile ) ) {
        cout << "Couldn't open input file \"" << infile << "\"" << endl;
        exit(-1);
      }

      opts.inFiles.push_back( infile );
  }

  //    for( i = 1; i < argc; i++ )
  //    {
  //        const char* s = argv[i];
  //        if( strcmp( s, "-w" ) == 0 )
  //        {
  //            if( sscanf( argv[++i], "%u", &boardSize.width ) != 1 || boardSize.width <= 0 )
  //                return fprintf( stderr, "Invalid board width\n" ), -1;
  //        }
  //        else if( strcmp( s, "-h" ) == 0 )
  //        {
  //            if( sscanf( argv[++i], "%u", &boardSize.height ) != 1 || boardSize.height <= 0 )
  //                return fprintf( stderr, "Invalid board height\n" ), -1;
  //        }
  //        else if( strcmp( s, "-pt" ) == 0 )
  //        {
  //            i++;
  //            if( !strcmp( argv[i], "circles" ) )
  //                pattern = CIRCLES_GRID;
  //            else if( !strcmp( argv[i], "acircles" ) )
  //                pattern = ASYMMETRIC_CIRCLES_GRID;
  //            else if( !strcmp( argv[i], "chessboard" ) )
  //                pattern = CHESSBOARD;
  //            else
  //                return fprintf( stderr, "Invalid pattern type: must be chessboard or circles\n" ), -1;
  //        }
  //        else if( strcmp( s, "-s" ) == 0 )
  //        {
  //            if( sscanf( argv[++i], "%f", &squareSize ) != 1 || squareSize <= 0 )
  //                return fprintf( stderr, "Invalid board square width\n" ), -1;
  //        }
  //        ..else if( strcmp( s, "-n" ) == 0 )
  //        {
  //            if( sscanf( argv[++i], "%u", &nframes ) != 1 || nframes <= 3 )
  //                return printf("Invalid number of images\n" ), -1;
  //        }
  //        else if( strcmp( s, "-a" ) == 0 )
  //        {
  //            if( sscanf( argv[++i], "%f", &aspectRatio ) != 1 || aspectRatio <= 0 )
  //                return printf("Invalid aspect ratio\n" ), -1;
  //            flags |= CV_CALIB_FIX_ASPECT_RATIO;
  //        }
  //        else if( strcmp( s, "-d" ) == 0 )
  //        {
  //            if( sscanf( argv[++i], "%u", &delay ) != 1 || delay <= 0 )
  //                return printf("Invalid delay\n" ), -1;
  //        }
  //        else if( strcmp( s, "-op" ) == 0 )
  //        {
  //            writePoints = true;
  //        }
  //        else if( strcmp( s, "-oe" ) == 0 )
  //        {
  //            writeExtrinsics = true;
  //        }
  //        else if( strcmp( s, "-zt" ) == 0 )
  //        {
  //            flags |= CV_CALIB_ZERO_TANGENT_DIST;
  //        }
  //        else if( strcmp( s, "-p" ) == 0 )
  //        {
  //            flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
  //        }
  //        else if( strcmp( s, "-v" ) == 0 )
  //        {
  //            flipVertical = true;
  //        }
  //        else if( strcmp( s, "-V" ) == 0 )
  //        {
  //            videofile = true;
  //        }
  //        else if( strcmp( s, "-o" ) == 0 )
  //        {
  //            outputFilename = argv[++i];
  //        }
  //        else if( strcmp( s, "-su" ) == 0 )
  //        {
  //            showUndistorted = true;
  //        }
  //        else if( s[0] != '-' )
  //        {
  //            if( isdigit(s[0]) )
  //                sscanf(s, "%d", &cameraId);
  //            else
  //                inputFilename = s;
  //        }
  //        else
  //            return fprintf( stderr, "Unknown option %s", s ), -1;
  //    }

  string msg;
  if( !opts.validate( msg ) ) {
    cout << "Error: " <<  msg << endl;
    exit(-1);
  }
}



int main( int argc, char** argv )
{

  CalibrationOpts opts;

  parseOpts( argc, argv, opts );

  Board board = BoardFactory::load( opts.boardPath() );
  board.name = opts.boardName;

  const char* outputFilename = "out_camera_data.yml";
  Size imageSize;
  float aspectRatio = 1.f;
  int flags = 0;
  Mat cameraMatrix, distCoeffs;
  bool writeExtrinsics = false, writePoints = false;

  //    Size boardSize, 
  //    float squareSize = 1.f, 
  //    const char* inputFilename = 0;
  //
  //    int i, nframes = 10;
  //    bool undistortImage = false;
  //    VideoCapture capture;
  //    bool flipVertical = false;
  //    bool showUndistorted = false;
  //    bool videofile = false;
  //    int delay = 1000;
  //    clock_t prevTimestamp = 0;
  //    int mode = DETECTION;
  //    int cameraId = 0;
  //    vector<string> imageList;
  //    Pattern pattern = CHESSBOARD;


  vector<vector<Point2f> > imagePoints;

  if( opts.inFiles.size() < 1 ) {
    cout << "No input files specified on command line." << endl;
    exit(-1);
  }

  vector<Image> imagesUsed;

  if( opts.ignoreCache ) cout << "Ignoring cached data." << endl;
  for( int i = 0; i < opts.inFiles.size(); ++i ) {
    cout << "Processing " << i << " : " << opts.inFiles[i] << endl;
    Mat view, viewGray;

    view = imread(opts.inFiles[i], 1);

    // Fix this later
    imageSize = view.size();

    //if(!view.data)
    //{
    //  if( imagePoints.size() > 0 )
    //    runAndSave(outputFilename, imagePoints, imageSize,
    //        boardSize, pattern, squareSize, aspectRatio,
    //        flags, cameraMatrix, distCoeffs,
    //        writeExtrinsics, writePoints);
    //  break;
    //}

    Image img( opts.inFiles[i], view );
    bool found;


    // Check for cached data
    string imageCache = opts.imageCache( img.fileName() );
    if( !opts.ignoreCache && img.loadCache( imageCache ) ) {

      cout << "  ... loaded data from cache." << endl;
    } else {

      cout << "  No cached data, searching for calibration pattern." << endl;

      //if( flipVertical )
      //  flip( view, view, 0 );

      vector<Point2f> pointbuf;
      cvtColor(view, viewGray, COLOR_BGR2GRAY);


      switch( board.pattern )
      {
        case CHESSBOARD:
          found = findChessboardCorners( view, board.size(), pointbuf,
              CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
          break;
        case CIRCLES_GRID:
          found = findCirclesGrid( view, board.size(), pointbuf );
          break;
        case ASYMMETRIC_CIRCLES_GRID:
          found = findCirclesGrid( view, board.size(), pointbuf, CALIB_CB_ASYMMETRIC_GRID );
          break;
        default:
          return fprintf( stderr, "Unknown pattern type\n" ), -1;
      }

      // improve the found corners' coordinate accuracy
      if( board.pattern == CHESSBOARD && found) 
        cornerSubPix( viewGray, pointbuf, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));



      if( found )  {
        cout << "  Found calibration pattern." << endl;
        img.points = pointbuf;
      }

      //if( mode == CAPTURING && found &&
      //    (!capture.isOpened() || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC) )
      //{
      //  imagePoints.push_back(pointbuf);
      //  prevTimestamp = clock();
      //  blink = capture.isOpened();
      //}
      img.writeCache( opts.imageCache( img.fileName() ) );
    }

    if( img.points.size() > 0 ) {
      imagesUsed.push_back( img );
      imagePoints.push_back( img.points );
      drawChessboardCorners( view, board.size(), Mat(img.points), found );
    }

    string outfile( opts.tmpPath( img.basename() ) );
    mkdir_p( outfile );
    imwrite(  outfile, view );


    //      string msg = mode == CAPTURING ? "100/100" :
    //        mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
    //      int baseLine = 0;
    //      Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
    //      Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);
    //
    //      if( mode == CAPTURING )
    //      {
    //        if(undistortImage)
    //          msg = format( "%d/%d Undist", (int)imagePoints.size(), nframes );
    //        else
    //          msg = format( "%d/%d", (int)imagePoints.size(), nframes );
    //      }
    //
    //      putText( view, msg, textOrigin, 1, 1,
    //          mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));
    //
    //      if( blink )
    //        bitwise_not(view, view);
    //
    //      if( mode == CALIBRATED && undistortImage )
    //      {
    //        Mat temp = view.clone();
    //        undistort(temp, view, cameraMatrix, distCoeffs);
    //      }

//    imshow("Image View", view);
//    waitKey(1000);

    //int key = 0xff & waitKey(capture.isOpened() ? 50 : 500);

    //if( (key & 255) == 27 )
    //  break;

    //if( key == 'u' && mode == CALIBRATED )
    //  undistortImage = !undistortImage;

    //      if( capture.isOpened() && key == 'g' )
    //      {
    //        mode = CAPTURING;
    //        imagePoints.clear();
    //      }
    //
    //      if( mode == CAPTURING && imagePoints.size() >= (unsigned)nframes )
    //      {
    //
    //

  } // For each image


cout << "Have points from " << imagePoints.size() << " images" << endl;

char strtime[32];
time_t tt;
time( &tt );
struct tm *t2 = localtime( &tt );
strftime( strtime, 32, "cal_%y%m%d_%H%M%S.yml", t2 );
string cameraFile( opts.cameraPath( strtime ) );

cout << "Appended results to " << cameraFile << endl;
  runAndSave(cameraFile, imagePoints, imageSize, board,
      imagesUsed,
      aspectRatio,
      flags, cameraMatrix, distCoeffs,
      writeExtrinsics, writePoints);

  //    if( inputFilename )
  //    {
  //        if( !videofile && readStringList(inputFilename, imageList) )
  //            mode = CAPTURING;
  //        else
  //            capture.open(inputFilename);
  //    }
  //    else
  //        capture.open(cameraId);
  //
  //    if( !capture.isOpened() && imageList.empty() )
  //        return fprintf( stderr, "Could not initialize video (%d) capture\n",cameraId ), -2;
  //
  //    if( !imageList.empty() )
  //        nframes = (int)imageList.size();
  //
  //    if( capture.isOpened() )
  //        printf( "%s", liveCaptureHelp );
  //
  //    namedWindow( "Image View", 1 );
  //
  //    for(i = 0;;i++)
  //    {
  //
  //    if( !capture.isOpened() && showUndistorted )
  //    {
  //        Mat view, rview, map1, map2;
  //        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
  //                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
  //                                imageSize, CV_16SC2, map1, map2);
  //
  //        for( i = 0; i < (int)imageList.size(); i++ )
  //        {
  //            view = imread(imageList[i], 1);
  //            if(!view.data)
  //                continue;
  //            //undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
  //            remap(view, rview, map1, map2, INTER_LINEAR);
  //            imshow("Image View", rview);
  //            int c = 0xff & waitKey();
  //            if( (c & 255) == 27 || c == 'q' || c == 'Q' )
  //                break;
  //        }
  //    }

  return 0;
  }
