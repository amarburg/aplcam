#pragma once

#include <nlohmann/json.hpp>

#include "AplCam/board/circle.h"

namespace AplCam {

  using json = nlohmann::json;

  class TrailerHitch : public ColorSegmentationCircleBoard {
   public:
    TrailerHitch( const std::string &name );
    virtual ~TrailerHitch();

    virtual Detection *detectPattern( const cv::Mat &gray );

    virtual void to_json( json &j ) const;
    virtual void from_json( const json &j );

   protected:

     struct HueTarget {
       HueTarget( const std::string &n, unsigned int hue = 0 )
        : name(n), _hue(hue) {;}

       virtual Detection *detectPattern( const cv::Mat &img ) const;

       std::string name;
       unsigned int _hue;
     };

     vector<HueTarget> _targets;

  };

  inline void   to_json(json& j, const TrailerHitch& p) { p.to_json(j); }
  inline void from_json(const json& j, TrailerHitch& p) { p.from_json(j); }


}
