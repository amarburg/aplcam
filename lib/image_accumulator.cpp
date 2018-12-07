
#include "libg3logger/g3logger.h"

#include "AplCam/image_accumulator.h"

namespace AplCam {

using namespace cv;

ImageAccumulator::ImageAccumulator( void )
    : _upToDate( false )
{;}

Mat ImageAccumulator::mean( void )
{
  if( !_upToDate ) update();
  return _mean;
}

Mat ImageAccumulator::var( void )
{
  if( !_upToDate ) update();
  return _var;
}

bool ImageAccumulator::add( const Mat &mat )
{
  _upToDate = false;

  if( _imgs.size() > 0 ) {
    if( mat.size() != _imgs.front().size() ) {
      LOG(WARNING) << "Trying to add image to accumulator which doesn't have same size as others.";
      return false;
    }

    if( mat.type() != _imgs.front().type() ) {
      LOG(WARNING) << "Trying to add image to accumulator which doesn't have same type as others.";
      return false;
    }

  }

  _imgs.push_back( mat );

  return true;
}

void ImageAccumulator::update( void )
{
  if( _imgs.size() == 0 ) {
    _mean = Mat::zeros(0,0,CV_64F);
    _var = Mat::zeros(0,0,CV_64F);
    return;
  }

  _mean = Mat::zeros( _imgs.begin()->size(),
                     CV_MAKE_TYPE( CV_64F, _imgs.begin()->channels() ) );
  _var = Mat::zeros( _mean.size(), _mean.type() );

  for( vector< Mat >::iterator itr = _imgs.begin(); itr != _imgs.end(); ++itr ) {
    Mat m( _mean.size(), _mean.type() );
    (*itr).convertTo( m, m.type() );
    _mean += m;
  }
  _mean /= _imgs.size();

  for( vector< Mat >::iterator itr = _imgs.begin(); itr != _imgs.end(); ++itr ) {
    Mat m( _mean.size(), _mean.type() );
    (*itr).convertTo( m, m.type() );

    Mat diff = m - _mean;
    _var += diff.mul( diff );
  }
  _var /= _imgs.size();

  _upToDate = true;
}

}
