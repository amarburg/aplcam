
#include <fstream>
#include <iomanip>

#include "libg3logger/g3logger.h"

#include "AplCam/detection_db.h"


namespace AplCam {

  using namespace std;

  // JsonDetectionDb::JsonDetectionDb(  )
  //   : _filename("")
  // {
  // }

  JsonDetectionDb::JsonDetectionDb( const std::string &filename )
    : _filename(filename)
  {
    LOG(INFO) << "Filename: " << filename;
    if( !filename.empty() ) {
      load();
    }
  }

  JsonDetectionDb::~JsonDetectionDb()
  {
  }

  void JsonDetectionDb::setFilename( const std::string &filename ) {
    _filename = filename;
  }

  void JsonDetectionDb::save()
  {
    if( _filename.empty() ) return;

    json j = *this;

    LOG(DEBUG) << "Saving db to " << _filename;
    ofstream out( _filename );

    // From the json.hpp docs: the setw manipulator was overloaded to set the indentation for pretty printing
    out << std::setw(4) << j;
  }

  void JsonDetectionDb::load()
  {
    ifstream in( _filename );

    if( !in.is_open() ) {
      LOG(DEBUG) << "File " << _filename << " doesn't exist, initializing empty database";
      return;
    }

    json j;

    try {
      in >> j;
    } catch( std::exception p ) {
      LOG(WARNING) << "Unable to parse existing JSON, starting with empty db";
      return;
    }

    from_json( j, *this );
  }

  bool JsonDetectionDb::insert( const std::string &frame, const std::shared_ptr<Detection> &detection ) {
    _map.insert( std::make_pair( frame, detection ) );
    return true;
  }

  std::shared_ptr<Detection> JsonDetectionDb::at( const std::string &frame ) {
    return _map.at( frame );
  }

  //=== Meta-information that might be added to detection db

  void JsonDetectionDb::setMeta( const cv::Size &sz, float fps ) {
    _meta["image_size"] = {sz.width, sz.height};
    _meta["fps"] = fps;
  }

  bool JsonDetectionDb::imageSize( cv::Size &sz ) {
    if( _meta.count("image_size") == 0 ) return false;

    sz = cv::Size( _meta["image_size"][0], _meta["image_size"][1] );
    return true;
  }

  //======

  void to_json( json& j, const JsonDetectionDb &db ) {
    j = {};

    json detections = {};

    for( auto const &itr : db._map ) {
      detections[itr.first] = *(itr.second);
    }

    j["detections"] = detections;
    j["meta"] = db._meta;
  }

  void from_json(const json& j, JsonDetectionDb& db) {
    db._map.clear();

    if( j.count("detections") > 0 ) {
      json jdet = j["detections"];

      for (json::iterator det = jdet.begin(); det != jdet.end(); ++det) {
          LOG(DEBUG) << "   loading: " << det.key();

          std::shared_ptr<Detection> detection( new Detection );
          *(detection.get()) = det.value();

          db.insert( det.key(), detection );
      }
    }

    if( j.count("meta") > 0 ) db._meta = j["meta"];

  }


}
