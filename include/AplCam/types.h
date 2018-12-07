#pragma once

#include <vector>

#include <opencv2/core/core.hpp>

namespace AplCam {
  using cv::Vec3f;
  using cv::Vec2f;
  using cv::Vec3d;

  using std::vector;

  typedef Vec3f ObjectPoint;
  typedef vector< ObjectPoint > ObjectPointsVec;
  typedef vector< vector< ObjectPoint > > ObjectPointsVecVec;

  typedef Vec2f ImagePoint;
  typedef vector< ImagePoint > ImagePointsVec;
  typedef vector< vector< ImagePoint > > ImagePointsVecVec;

  typedef vector< Vec3d > RotVec, TransVec;

  struct ReprojError {
    ImagePoint projPoint, error;
  };

  typedef vector< ReprojError > ReprojErrorVec;
  typedef vector< ReprojErrorVec > ReprojErrorVecVec;


  void write(cv::FileStorage &fs, std::string a, const ReprojErrorVec& v );
  void write(cv::FileStorage &fs, std::string a, const ReprojErrorVecVec &v );

}
