#pragma once

#include <vector>
#include <string>
#include <memory>

#include <opencv2/core/core.hpp>

#include "AplCam/types.h"

namespace AplCam {
enum Pattern { CHESSBOARD,
  CIRCLE,
  COLOR_SEG_CIRCLE,
  CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID, APRILTAGS };

// Forward decl
struct Detection;

using AplCam::ObjectPointsVec;
using AplCam::ImagePointsVec;

using cv::Mat;

  class Board {
   public:
    Board( Pattern pat, int w, int h, float squares, const std::string &nam )
        : name(nam), pattern(pat), width(w), height(h), squareSize( squares )
    {;}

    virtual ~Board() {;}

    std::string name;
    Pattern pattern;
    int width, height;
    float squareSize;

    cv::Size size( void ) const { return cv::Size( width,height ); }

    virtual Detection *detectPattern( const Mat &gray );

    //typedef enum { BOARD_UL, BOARD_CENTER } CornersReference;

    virtual void draw( cv::Mat &img, Detection *detection ) const;

    virtual ObjectPointsVec corners( void );  // const CornersReference ref = BOARD_UL );

    virtual std::vector< int > ids( void );

    virtual void extents( ObjectPointsVec &ext ) const;

    virtual ObjectPoint worldLocation( const cv::Point2i &xy ) const;

    static Board *load( const std::string &infile, const std::string &name );

    void ensureGrayscale( const Mat &img, Mat &gray );

    std::string patternString( void ) const {
      switch(pattern)
      {
        case CHESSBOARD:
          return "chessboard";
          break;
        case CIRCLE:
          return "circle";
          break;
        case COLOR_SEG_CIRCLE:
          return "color_seg_circle";
          break;
        case CIRCLES_GRID:
          return "circles_grid";
          break;
        case ASYMMETRIC_CIRCLES_GRID:
          return "asym_circles_grid";
          break;
        case APRILTAGS:
          return "apriltags";
          break;
      }
      return "";
    }


   protected:

    virtual void loadCallback( cv::FileStorage &fs ) {;}

   private:
  };

  class BoardFactory {
  public:

    static std::shared_ptr<Board> LoadJson( const std::string &filename );

  };

}
