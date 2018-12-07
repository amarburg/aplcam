
#pragma once

#include <limits.h>
#include <unistd.h>
#include <getopt.h>

#include <string>

#include <tclap/CmdLine.h>

#include "AplCam/calib_frame_selectors.h"

namespace AplCam {

  namespace CalibFrameSelectors {

    typedef enum { SPLIT_ALL, SPLIT_RANDOM, SPLIT_INTERVAL, SPLIT_KEYFRAME, SPLIT_NONE = -1 } Type_t;

    using std::string;


    class CalibFrameSelectorOpts {
      public:

        CalibFrameSelectorOpts( TCLAP::CmdLine &cmd ) :
          selectorArg("", "selector", "Selector", true, "", "{random}", cmd ),
          randomCountArg( "", "count", "Count", false, -1, "Count", cmd ),
          startArg( "", "start-frame", "Start", false, 0, "Start", cmd ),
          endArg("", "end-frame", "End", false, INT_MAX, "End", cmd ),
          intervalArg("", "interval", "Interval", false, 0, "Interval (in frames)", cmd ),
          minTagsArg("", "min-tags", "Minimum number of tags", false, -1, "Number of tags", cmd )
          {;}

        FrameSelector *construct( void )
        {
          string selector( selectorArg.getValue() );
          if( selector.compare("random") == 0 ) {
            if( ! randomCountArg.isSet() ) {
              LOG(ERROR) << "--count not set for random selector";
              return NULL;
            }

            return new RandomFrameSelector( randomCountArg.getValue(), minTagsArg.getValue() );
          } else if( selector.compare("interval") == 0 ) {
            if( ! intervalArg.isSet() ) {
              LOG(ERROR) << "--interval not set for interval selector";
              return NULL;
            }

            return new IntervalFrameSelector( startArg.getValue(), intervalArg.getValue(), endArg.getValue(), minTagsArg.getValue() );
          } else if( selector.compare("all-good") == 0 ) {
            return new AllGoodFrameSelector( minTagsArg.getValue() );
          } else if( selector.compare("all") == 0 ) {
            return new AllFrameSelector( minTagsArg.getValue() );
          }

          LOG(ERROR) << "Don't know how to create a selector \"" << selector << "\"";
          return NULL;
        }

      protected:

        TCLAP::ValueArg< std::string > selectorArg;
        TCLAP::ValueArg< int > randomCountArg, startArg, endArg, intervalArg, minTagsArg;

    };



    //
    //  struct RandomSelectorOpts {
    //    RandomSelectorOpts()
    //      : count(-1) {;}
    //
    //    long int count;
    //
    //    static struct option long_options[];
    //    bool parseOpts( int argc, char **argv, string &msg );
    //  };

    //  struct IntervalSelectorOpts {
    //    IntervalSelectorOpts()
    //      : start(0), end(INT_MAX), interval(1) {;}
    //
    //    int start, end, interval;
    //
    //    static struct option long_options[];
    //    bool parseOpts( int argc, char **argv, string &msg );
    //  };
    //
    //  struct KeyframeSelectorOpts {
    //    KeyframeSelectorOpts()
    //      : minOverlap( 0.2 ) {;}
    //
    //    float minOverlap;
    //
    //    static struct option long_options[];
    //    bool parseOpts( int argc, char **argv, string &msg );
    //  };



  }
}
