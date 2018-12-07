#pragma once

#include <opencv2/core/core.hpp>

#include <vector>
#include <string>

#include <nlohmann/json.hpp>


#include "AplCam/types.h"

namespace AplCam {

  using cv::FileStorage;
  using nlohmann::json;

  struct Result {
    Result( size_t sz = 0 )
      : rms(-1), reprojErrors( sz ), good( true ),
      numPoints(-1), numImages(-1)
    {;}

    virtual void resize( size_t sz )
    {
      rms = -1.0;
      numPoints = -1;
      numImages = -1;
      reprojErrors.resize( sz );
    }

    // virtual void serialize( FileStorage &fs )
    // {
    //   fs << "success" << good;
    //   fs << "numPoints" << numPoints;
    //   fs << "numImages" << numImages;
    //   fs << "rms" << rms;
    //   fs << "reprojErrors" << reprojErrors;
    // }

    virtual void to_json( json &j ) const
    {
      j["success"] = good;
      j["numPoints"] = numPoints;
      j["numImages"] = numImages;
      j["rms"] = rms;

      std::transform( reprojErrors.begin(), reprojErrors.end(), std::back_inserter( j["reprojErrors"]),
                      []( const ReprojErrorVec &errors ) -> json {
                        json j;
                        std::transform( errors.begin(), errors.end(), std::back_inserter(j),
                                        []( const ReprojError &err ) -> json {
                                          return {err.projPoint[0], err.projPoint[1], err.error[0], err.error[1]};
                                        } );
                        return j;
                      } );

      //j["reprojErrors"] = reprojErrors;
    }

    // std::string toString(  void )
    // {
    //   FileStorage fs("foo.yml", FileStorage::WRITE | FileStorage::MEMORY );
    //   //serialize( fs );
    //   return fs.releaseAndGetString();
    // }

    double rms;
    ReprojErrorVecVec reprojErrors;
    bool good;
    int numPoints, numImages;

  };

  void   to_json(json& j, const Result& p);
  void from_json(const json& j, Result& p);




  struct CalibrationResult : public Result {
    CalibrationResult( size_t sz = 0)
      : Result( sz ),
      totalTime(-1.0), residual(-1.0),
      rvecs( sz, Vec3d(0,0,0) ),
      tvecs( sz, Vec3d(0,0,0) ),
      status( sz, false )
    { good = false; }

    virtual void resize( size_t sz )
    {
      good = false;
      totalTime = -1.0;
      residual = -1.0;
      rvecs.resize( sz, Vec3d(0,0,0) );
      tvecs.resize( sz, Vec3d(0,0,0) );
      status.resize( sz, false );
    }

    // virtual void serialize( FileStorage &fs )
    // {
    //   Result::serialize(fs);
    //
    //   fs << "totalTime" << totalTime;
    //   fs << "residual" << residual;
    // }


    virtual void to_json( json &j ) const
    {
      Result::to_json(j);

      j["totalTime"] = totalTime;
      j["residual"] = residual;
    }



    double totalTime, residual;

    RotVec rvecs;
    TransVec tvecs;
    vector< bool > status;
  };

  void   to_json(json& j, const CalibrationResult& p);
  void from_json(const json& j, CalibrationResult& p);

}
