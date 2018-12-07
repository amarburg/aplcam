
#include <iostream>

#include <gtest/gtest.h>

#include "AplCam/detection_db.h"
#include "nlohmann/json.hpp"

namespace {

  using namespace AplCam;
  using namespace std;

  using nlohmann::json;

  TEST( JsonDetectionDb, Constructor ) {
    JsonDetectionDb db;
  }


  TEST( JsonDetectionDb, SerializeToJson ) {
    JsonDetectionDb db;

    json j = db;

  }

  TEST( JsonDetectionDb, SerializeFromJson ) {

    json j;

    JsonDetectionDb db = j;
  }

}
