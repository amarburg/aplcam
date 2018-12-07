#pragma once

#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tclap/CmdLine.h>

#include "AplCam/file_utils.h"

#include "AplCam/frame_source.h"

#include "AplCam/frame_selector/frame_selector.h"

using namespace std;
using namespace cv;

namespace AplCam {

  struct SplitterOpts {
    public:
      SplitterOpts( )
        : waitKey(-1),
        seekTo(-1),
        scaleDisplay( -1 ),
        doDisplay( false ),
        saveFramesTo(),
        saveVideoTo(),
        _doSaveFrames( false ),
        _doSaveVideo( false ),
        fps(1),
        fpsSet( false )
    {;}

      //typedef enum {EXTRACT_SINGLE, EXTRACT_INTERVAL,  NONE = -1} Verbs;

      int waitKey;
      float  seekTo, scaleDisplay;
      bool doDisplay;
      vector< string > imgNames;
      string saveFramesTo, saveVideoTo, selector;
      bool _doSaveFrames, _doSaveVideo;

      float fps;
      bool fpsSet;

      bool parseArgs( int argc, char **argv, stringstream &msg );
      virtual void doParse( TCLAP::CmdLine &cmd, int argc, char **argv );

      virtual bool validate( stringstream &msg );

      Ptr<FrameSelector> makeSelector( void );

      bool doSaveVideo( void ) const { return _doSaveVideo; }
      bool doSaveFrames( void ) const { return _doSaveFrames; }
  };



  class SplitterApp
  {
    public:
      SplitterApp( SplitterOpts options );
      SplitterApp( SplitterOpts options, Ptr<FrameSelector> selector );

      virtual ~SplitterApp( void );

      virtual bool run( void );

      virtual bool processSplitFrame( Mat &img, Mat &toDisplay )
      { toDisplay = img; return true; }

      virtual bool processRejectedFrame( Mat &img, Mat &toDisplay )
      { return true; }

    protected:

      Ptr<FrameSelector> _selector;
      int _frame;

      VideoWriter _videoWriter;

    private:

      SplitterOpts _splitterOpts;


      const string winname = "input";
  };


}
