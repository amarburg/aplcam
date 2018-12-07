
#include <fstream>
#include <iomanip>

#include "libg3logger/g3logger.h"

#include "AplCam/detection_db.h"


namespace AplCam {

  using namespace std;

  // InMemoryDetectionDb::InMemoryDetectionDb(  )
  //   : _filename("")
  // {
  // }

  InMemoryDetectionDb::InMemoryDetectionDb( const std::string &filename )
    : _filename(filename)
  {
    LOG(INFO) << "Filename: " << filename;
    if( !filename.empty() ) {
      load();
    }
  }

  InMemoryDetectionDb::~InMemoryDetectionDb()
  {
  }

  void InMemoryDetectionDb::setFilename( const std::string &filename ) {
    _filename = filename;
  }

  void InMemoryDetectionDb::save()
  {
    if( _filename.empty() ) return;

    json j = *this;

    LOG(DEBUG) << "Saving db to " << _filename;
    ofstream out( _filename );

    // From the json.hpp docs: the setw manipulator was overloaded to set the indentation for pretty printing
    out << std::setw(4) << j;
  }

  void InMemoryDetectionDb::load()
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

  bool InMemoryDetectionDb::insert( const std::string &frame, const std::shared_ptr<Detection> &detection ) {
    _map.insert( std::make_pair( frame, detection ) );
    return true;
  }

  std::shared_ptr<Detection> InMemoryDetectionDb::at( const std::string &frame ) {
    return _map.at( frame );
  }

  bool InMemoryDetectionDb::setMeta( unsigned int length, int width, int height, float fps ) {
    return true;
  }

  //======

  void to_json(json& j, const InMemoryDetectionDb& p) {
    j = {};

    json detections = {};

    for( auto const &itr : p._map ) {
      detections[itr.first] = *(itr.second);
    }

    j["detections"] = detections;
  }

  void from_json(const json& j, InMemoryDetectionDb& db) {
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

  }


}
