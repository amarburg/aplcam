#pragma once

#include <string>
#include <vector>
#include <map>

#include <opencv2/core/core.hpp>

#include "nlohmann/json.hpp"

#include "AplCam/detection/detection.h"


namespace AplCam {

  using nlohmann::json;

  class DetectionDb {
     public:

       static const string MetaKey,
                           MetaFpsKey,
                           MetaWidthKey,
                           MetaHeightKey,
                           MetaLengthKey;

       DetectionDb() {;}
       virtual ~DetectionDb() {;}

       virtual void save() {;}

  //     virtual Detection *load( const int frame ) = 0;

       virtual bool insert( const std::string &frame, const std::shared_ptr<Detection> &detection ) = 0;
       virtual std::shared_ptr<Detection> at( const std::string &frame ) = 0;

       virtual bool setMeta( unsigned int length, int width, int height, float fps ) = 0;

  };


  class InMemoryDetectionDb;

  class InMemoryDetectionDb {
  public:

    typedef std::map< std::string, std::shared_ptr<Detection> > DetectionMap;

    //InMemoryDetectionDb( );
    InMemoryDetectionDb( const std::string &filename = std::string() );
    ~InMemoryDetectionDb();

    void setFilename( const std::string &filename );
    std::string filename() const { return _filename; }

    void load();
    virtual void save();

    virtual bool insert( const std::string &frame, const std::shared_ptr<Detection> &detection );
    virtual std::shared_ptr<Detection> at( const std::string &frame );

    virtual bool setMeta( unsigned int length, int width, int height, float fps );

    // Friend functions for serializing and unserializaing to JSON
    friend void to_json(json& j, const InMemoryDetectionDb& p);
    friend void from_json(const json& j, InMemoryDetectionDb& p);

    const DetectionMap &map() const { return _map; }

  protected:

    DetectionMap _map;

    std::string _filename;

  };

  void   to_json(json& j, const InMemoryDetectionDb& p);
  void from_json(const json& j, InMemoryDetectionDb& p);



    //   DetectionDb( const string &dbFile, bool writer = false );
    //
    //   ~DetectionDb();
    //
    //   bool open( const string &dbFile, bool writer = false );
    //   bool open( const string &dbDir, const string &videoFile, bool writer = false );
    //

    //   bool save( const DetectionSet &detSet );
    //
    //   bool has( const int frame );
    //   bool has( const string &key );
    //   bool has_meta( void );
    //
    //   bool update( const int frame, const Detection &detection );
    //   bool update( const string &key, const Detection &detection );
    //
    //   Detection *load( const int frame );
    //   Detection *load( const string &key );
    //
    //   kyotocabinet::BasicDB::Error error( void ) { return _db.error(); }
    //
    //   int maxKey( void );
    //
    //   static const std::string FrameToKey( const int frame );
    //
    //   kyotocabinet::DB::Cursor *cursor( void );
    //
    //   cv::Size imageSize( void ) const { return _imageSize; }
    //   int vidLength( void ) const { return _vidLength; }
    //   float fps( void ) const { return _fps; }
    //
    //
    //   static const string MetaKey,
    //                       MetaFpsKey,
    //                       MetaWidthKey,
    //                       MetaHeightKey,
    //                       MetaLengthKey;
    //
    // protected:
    //
    //   bool saveMeta( void );
    //   void loadMeta( void );
    //
    //   HashDB _db;
    //   kyotocabinet::DB::Cursor *_cursor;
    //
    //   // Metadata
    //   cv::Size _imageSize;
    //   int _vidLength;
    //   float _fps;
    //
//  };

  //
  // class JsonDbSerializer {
  // public:
  //
  //   static bool Write( const std::string &filename, const DetectionDb &db );
  //   static DetectionDb *Read( const std::string &filename );
  //
  // private:
  //
  //   JsonDbSerializer() = delete;
  //   JsonDbSerializer( const JsonDbSerializer &other ) = delete;
  // };

}
