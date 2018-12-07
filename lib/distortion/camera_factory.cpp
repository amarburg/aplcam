
#include <iostream>

#include <libg3logger/g3logger.h>

#include "AplCam/distortion/camera_factory.h"

#include "AplCam/distortion/angular_polynomial.h"
#include "AplCam/distortion/radial_polynomial.h"

namespace Distortion {

  using namespace std;
  using namespace cv;

  DistortionModel *CameraFactory::LoadDistortionModel( const string &file )
  {
    FileStorage fs( file, FileStorage::READ );
    return Unserialize( fs );
  }

  DistortionModel *CameraFactory::FromString( const string &str )
  {
    FileStorage fs( str, FileStorage::READ | FileStorage::MEMORY );
    return Unserialize( fs );
  }

  DistortionModel *CameraFactory::Unserialize( FileStorage &fs )
  {
    DistortionModel *out = NULL;

    string type;
    fs["camera_model"] >> type;

    if( type.compare( AngularPolynomial::Name()  ) == 0 ) {
      out = AngularPolynomial::Load( fs );
    } else if( (type.compare( CeresRadialPolynomial::Name() ) == 0) or
               (type.compare( OpencvRadialPolynomial::Name() ) == 0)) {
      out = RadialPolynomial::Load( fs );
    } else {
      LOG(WARNING) << "Don't know how to create a camera of type \"" << type << "\"";
    }

    return out;
  }

}
