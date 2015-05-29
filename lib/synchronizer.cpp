
#include <algorithm>

#include <Eigen/LU>

#include "synchronizer.h"
#include "composite_canvas.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const float Synchronizer::Scale = 1.0;

Synchronizer::Synchronizer( Video &v0, Video &v1 )
: _video0( v0 ), _video1( v1 ), _offset( 0 )
{;}


void Synchronizer::rewind( void )
{
  if( _offset < 0 ) {
    _video0.seek( -_offset );
    _video1.seek( 0 );
  } else {
    _video0.seek( 0 );
    _video1.seek( _offset );
  }
  cout << "Rewind to frames: " << _video0.frame() << ' ' << _video1.frame() << endl;
}

bool Synchronizer::seek( int which, int dest0 )
{
  if( which == 0 ) {
    int dest1 = dest0 + _offset;

    if( dest0 >= 0 && dest0 < _video0.frameCount() && 
       dest1 >= 0 && dest1 < _video1.frameCount() ) {
      _video0.seek(dest0);
      _video1.seek(dest1);
      cout << "Seeking to: " << _video0.frame() << ' ' << _video1.frame() << endl;
      return true;
    } 

  } else {
    return seek( 0, dest0 - _offset );
  }

  return  false;
}

bool Synchronizer::scrub( int offset )
{  int dest0 = _video0.frame() + offset;
  return seek( 0, dest0 );
}


bool Synchronizer::advanceToNextTransition( int which )
{
  int current = (which == 0) ? _video0.frame() : _video1.frame();
  Video &vid( (which == 0) ? _video0 : _video1 );
  const TransitionMap &transitions( vid.transitions() );

  if( transitions.size() == 0 )
    return false;
  else 
  {
    if( transitions.begin()->first > current ) {
      seek( which, transitions.begin()->first );
      cout << "Advancing o " << which << " to frame " << transitions.begin()->first << endl;
      return true;
    } 

    if( transitions.size() > 1 ) {
      TransitionMap::const_iterator itr = transitions.begin(), prev = itr;
      itr++;
      for( ; itr != transitions.end(); ++itr, ++prev ) {
        if( (prev->first <= current) && (itr->first > current) ) {
          seek( which , itr->first );
          cout << "Advancing o " << which << " to frame " << itr->first << endl;
          return true;
        }
      }
    }
  }

  return false;
}

//Size Synchronizer::compositeSize( void )
//{ 
//  return  Size( Scale*(_video0.width() + _video1.width()), Scale*std::max(_video0.height(), _video1.height()) ); 
//}

void Synchronizer::advanceOnly( int which )
{
  if( which == 0 ) {
    _offset--;
    _video1.scrub(-1);
  } else {
    _offset++;
    _video0.scrub(-1);
  }
}


bool Synchronizer::nextSynchronizedFrames( cv::Mat &video0, cv::Mat &video1 )
{
  if( _video0.read( video0 ) && _video1.read( video1 ) ) {
    cout << "Frames: " << _video0.frame() << ' ' << _video1.frame() <<  ' ' << _offset << endl;
    return true;
  }

  return false;
}

bool Synchronizer::nextCompositeFrame( AplCam::CompositeCanvas &canvas )
{


  Mat frame0, frame1;
  if( nextSynchronizedFrames( frame0, frame1 ) ) {
    canvas = AplCam::CompositeCanvas( frame0, frame1 );
  } else return false;

  return true;
}

//void Synchronizer::compose( const cv::Mat &img0, cv::Mat &img1, cv::Mat &composite, float scale )
//{
//  // TODO.  Should check size of input images
//
//  Size compSize( scale*(img0.cols + img1.cols),
//      scale*std::max(img0.rows, img1.rows) ); 
//  composite.create( compSize, CV_8UC3 );
//
//  Mat video0ROI( composite, Rect( 0, 0,               scale*img0.cols, scale*img0.rows) );
//  Mat video1ROI( composite, Rect( scale*img0.cols, 0, scale*img1.cols, scale*img1.rows) );
//
//  if( scale != 1.0 ) {
//    resize( img0, video0ROI, video0ROI.size() );
//    resize( img1, video1ROI, video1ROI.size() );
//  } else {
//    img0.copyTo( video0ROI );
//    img1.copyTo( video1ROI );
//  }
//}



//---------------------------------------------------------------------------
// Tools for estimating initial offset
//---------------------------------------------------------------------------

IndexPair Synchronizer::getSpan( const TransitionVec &transitions, int start, int length )
{
  cout << "Getting span from " << start << " to " << start+length << endl;
  //if( (start+length) > frameCount() ) { start = frameCount()-length;

  //  cout << "Adjusted span from " << start << " to " << length << endl;
  //}

  int startIdx = 0;
  for( size_t i = 0; i < transitions.size(); ++i )
    if( transitions[i].frame > start ) { startIdx = i; break; }

  int endIdx = transitions.size();
  for( size_t i = startIdx; i < transitions.size(); ++i ) 
    if( transitions[i].frame > (start+length) ) {endIdx = i; break;}


  //      cout << "Excluded these transitios:" << endl;
  //      for( int i = 0; i < startIdx; ++i ) cout << _transitions[i].frame << endl;
  //      cout << endl;
  //
  //      cout << "Included there transitions"<< endl;
  //      for( int i = startIdx; i < endIdx; ++i ) cout << _transitions[i].frame << endl;
  //      cout <<endl;
  //
  //      cout << "Excluded there transitions"<< endl;
  //      for( int i = endIdx; i < _transitions.size(); ++i ) cout << _transitions[i].frame << endl;
  //      cout <<endl;


  return make_pair( startIdx, endIdx );
}

bool Synchronizer::shiftSpan( const TransitionVec &transitions, IndexPair &pair, int length, int direction )
{
  if( direction < 0 ) {
    if( pair.first == 0 ) return false;
    direction = -1;
  } else if (direction > 0 ) {
    if( pair.second == transitions.size() ) return false;
    direction = +1;

  }
  pair.first += direction;
  pair.second = transitions.size();
  int max = transitions[pair.first].frame + length;
  for( size_t i = pair.first; i < transitions.size(); ++i ) 
    if( transitions[i].frame > max ) {pair.second = i; break;}

  return true;
}

float Synchronizer::compareSpans( const TransitionVec &thisTransitions,  IndexPair &thisSpan, 
                                 const TransitionVec &otherTransitions, IndexPair &otherSpan )
{
  cout << "======================" << endl;
  cout << "this span:  " << thisSpan.first << ' ' << thisSpan.second << endl;
  for( int i = thisSpan.first; i < thisSpan.second; ++i ) {
    if( i > thisSpan.first ) 
      cout << thisTransitions[i].frame << ' ' << thisTransitions[i].frame - thisTransitions[i-1].frame << endl;
    else
      cout << thisTransitions[i].frame << ' ' <<endl;
  }

  cout << "other span: " << otherSpan.first << ' ' << otherSpan.second << endl;
  for( int i = otherSpan.first; i < otherSpan.second; ++i ) {
    if( i > otherSpan.first ) 
      cout << otherTransitions[i].frame << ' ' << otherTransitions[i].frame - otherTransitions[i-1].frame << endl;
    else
      cout << otherTransitions[i].frame << ' ' <<endl;
  }


  if( (thisSpan.second - thisSpan.first) != (otherSpan.second - otherSpan.first) ) {
    cout << "Spans are different lengths, using shorter of the two " << endl;
  }
  int  length = std::min( thisSpan.second-thisSpan.first, otherSpan.second-otherSpan.first );

  float total = 0.0;
  for( int i = 0; i < length; ++i ) {
    total += norm( thisTransitions[ thisSpan.first+i ].before, otherTransitions[ otherSpan.first+i ].before, NORM_L2 );
    total += norm( thisTransitions[ thisSpan.first+i ].after, otherTransitions[ otherSpan.first+i ].after, NORM_L2 );
  }
  total /= length;

  return total;
}

int Synchronizer::estimateOffset( const TransitionVec &trans0,  const TransitionVec &trans1, float windowFrames, float maxDeltaFrames, int seekTo ) 
{
  // TODO:  Currently assumes both videos have same FPS
  map <float, OffsetResult> results;

  IndexPair thisSpan( getSpan( trans0,  seekTo, windowFrames ) );
  IndexPair otherSpan( getSpan(  trans1, seekTo+maxDeltaFrames, windowFrames ) );

  cout << "Getting span with max delta " << maxDeltaFrames << "  length " << windowFrames << endl;
  cout << " Got " << thisSpan.first << ' ' << thisSpan.second << endl;
  cout << " Got " << otherSpan.first << ' ' << otherSpan.second << endl;


  // Start with set of transitions from maxDelta to maxDelta+window (this is the maximum backwards shift on video1)

  do {
    float result = compareSpans( trans0, thisSpan, trans1,  otherSpan );
    cout << "    result: " << result << endl;
    results.insert( make_pair( result, OffsetResult( thisSpan, otherSpan ) ) );
  } while( shiftSpan( trans1, otherSpan, windowFrames, -1 ) ) ;

  // Now start shifting my span forward
  while( shiftSpan(  trans0, thisSpan, windowFrames, +1 ) ) {
    float result = compareSpans( trans0, thisSpan, trans1,  otherSpan );
    cout << "    result: " << result << endl;
    results.insert( make_pair( result, OffsetResult( thisSpan, otherSpan ) ) );
  }

  OffsetResult best = results.begin()->second;

  // Calculate offset from end of spans...
  // Need to handle case where spans are different length
  //int dt = std::min( best.v0.second-best.v0.first, best.v1.second-best.v1.first ) - 1;
  //_offset = trans1[ dt ].frame - trans0[ dt ].frame;
  thisSpan = results.begin()->second.v0;
  otherSpan = results.begin()->second.v1;
  int thisFrame = trans0[ thisSpan.first ].frame,
      otherFrame = trans1[ otherSpan.first ].frame;
  _offset = otherFrame - thisFrame;

  cout << "Best alignment has score " << results.begin()->first << endl;
  cout << "With frames " << thisFrame << " " << otherFrame << endl;
  cout << "With video1 offset to video0 by " << _offset << endl;

  return _offset;
}

int Synchronizer::bootstrap( float window, float maxDelta, int seekTo )
{
  TransitionVec transitions[2];
  int windowFrames = window * _video0.fps(),
      maxDeltaFrames = maxDelta * _video0.fps();

  if( !_video0.capture.isOpened() ) {
    cerr << "Can't open video 0" << endl;
    exit(-1);
  }
  if( !_video1.capture.isOpened() ) {
    cerr << "Can't open video 1" << endl;
    exit(-1);
  }

  cout << _video0.dump() << endl;
  cout << _video1.dump() << endl;

  _video0.initializeTransitionStatistics( seekTo, 2*maxDeltaFrames, transitions[0] );
  _video1.initializeTransitionStatistics( seekTo, 2*maxDeltaFrames, transitions[1] );

  //cout << "Found " << transitions[i].size() << " transitions" << endl;

  //stringstream filename;
  //filename << "/tmp/transitions_" << i << ".png";

  //Video::dumpTransitions( transitions[i], filename.str() );

  return estimateOffset( transitions[0], transitions[1], windowFrames, maxDeltaFrames, seekTo  );
}




//===========================================================================

KFSynchronizer::KFSynchronizer( VideoLookahead &video0, VideoLookahead &video1 )
: Synchronizer( video0, video1 ), _lvideo0( video0 ), _lvideo1( video1 ),
    _kf( std::min( _lvideo0.lookaheadFrames(), _lvideo1.lookaheadFrames() ) ),
    _count( 0 ), _sinceLastUpdate( 0 )
{
  _lastObs[0] = _lastObs[1] = 0;
}

bool KFSynchronizer::nextSynchronizedFrames( cv::Mat &video0, cv::Mat &video1 )
{
  int predOffset = _kf.predict();

  if( predOffset  != _offset ) {
    //cout << "Predicted offset of " << predOffset << " does not agree with curent estimate " << _offset << endl;

    if( predOffset > _offset ) {
      // Video 1 is moving ahead, take a frame and drop if
      _lvideo1.drop();
      _offset = predOffset;
    } else if ( predOffset < _offset ) {
      _lvideo0.drop();
      _offset = predOffset;
    }

  }

  bool result = Synchronizer::nextSynchronizedFrames( video0, video1 );

  ++_sinceLastUpdate;

  const int rep = 5;
  if( _count++ > rep ) {
    _count = 0;

    vector<int> trans0, trans1;
    trans0 = _video0.transitionsAfter( std::max(_video0.frame(), _lastObs[0] ) );
    trans1 = _video1.transitionsAfter( std::max(_video1.frame(), _lastObs[1] ) );

    cout << "Got " << trans0.size() << " and " << trans1.size() << " transitions" << endl;

    if( trans0.size() > 2 || trans1.size() > 2 ) {
      cout << "I think one of the transitions is flapping, no check for transitions." << endl;

      if( trans0.size() > 0 ) _lastObs[0] = trans0.back();
      if( trans1.size() > 0 )_lastObs[1] = trans1.back();

      return result;
    }


    if( (trans0.size() > 0) && (trans1.size() > 0) ) {

      // Consider all possibilities
      vector< pair< int, int > > hypotheses;

      for( size_t i = 0; i < trans0.size(); ++i ) {
        for( size_t j = 0; j < trans1.size(); ++j ) {
          int dt = trans1[j] - trans0[i];

          int future0 = trans0[i] - _video0.frame(),
              future1 = trans1[j] - _video1.frame();
          int future = std::min( future0, future1 );

          cout << "Considering offset of dt = " << dt << " at " << future << " frames in the future" << endl;

          hypotheses.push_back(  make_pair(future, dt ) );

        }
      }

      int best_dt = -1, best_future = -1;
      float best_p = 1e6;

      for( size_t i = 0; i < hypotheses.size(); ++i ) {
        int future = hypotheses[i].first,
            dt     = hypotheses[i].second;

        if( future < 10 ) continue;

        float p = fabs( _kf[future] - dt );

        if( p < best_p ) {
          best_p = p;
          best_future = future;
          best_dt = dt;
        }

      }

      if( best_p > 1e5 ) return result;

      float max_p = std::min( 0.02 * _sinceLastUpdate, .5 ) * best_future;

      cout << "Best estimate dt = " << best_dt << " at " << best_future << " frames.  Predicted is " << _kf[best_future] << endl;
      cout << "           Delta = " << best_p << " max delta = " << max_p << endl; 


      if( best_dt == 44 ) {
        cout << "Skipping 44!" << endl;
        return result;
      }


      if (best_p <= max_p ) {

        cout << " !!! Accepting update" << endl;


        _kf.update( best_dt, best_future );

        _sinceLastUpdate = 0;

      } else {
        cout << "  --- Too large a disparity with prediction" << endl;
      }


      _lastObs[0] = trans0.back();
      _lastObs[1] = trans1.back();


    }
  }



  return result;
}

int KFSynchronizer::estimateOffset( const TransitionVec &trans0,  const TransitionVec &trans1, float windowFrames, float maxDeltaFrames, int seekTo ) 
{
  int out = Synchronizer::estimateOffset( trans0, trans1, windowFrames, maxDeltaFrames, seekTo );
  _kf.setOffset( _offset );
  return out;
}

//===========================================================================


SynchroKalmanFilter::SynchroKalmanFilter( int depth )
: _state(depth+1), _cov( states(), states() ),
    _f( states(), states() ), _q( states(), states() ), _r()
{
  float cov0 = 0.25, cov1 = 0.05;


  _cov.setZero();
  _cov.topLeftCorner( depth, depth ).setIdentity();
  _cov.topLeftCorner( depth, depth ) *= cov0;
  _cov( depth, depth ) = cov1;

  // Set the state propagation matrix
  _f.setZero();
  _f.topRightCorner( depth, depth ).setIdentity();
  //_f.col( depth ).setOnes();
  _f(depth-1,depth-1) = 1;
  _f(depth,depth) = 1;

  // Set the additive noise term
  _q.topLeftCorner( depth, depth ).setIdentity();
  _q.topLeftCorner( depth, depth ) *= 0.05;

  _q( depth, depth ) = 0.0;

  _state.setZero();
  _r.setIdentity();
  _r *= 4.0;
}

void SynchroKalmanFilter::setOffset( int offset )
{
  _state.head( states()-1 ).fill( offset );
}

int SynchroKalmanFilter::predict( void )
{
  _state = _f * _state;
  _cov = _f * _cov * _f.transpose() + _q;


  //cout << _state.transpose() << endl;

  return lround( _state(0) );
}

int SynchroKalmanFilter::update( int obs, int future )
{
  // Generate a Y (observation) matrix
  Matrix< double, 1, 1> y;
  y(0,0) = obs;

  // generate an H matrix
  RowVectorXd h( states() );
  h.setZero();
  h( future ) = 1.0;

  MatrixXd inno( states(), states() );
  inno = y - h * _state;

  //  cout << "h: " << endl << h << endl;
  //  cout << "Inno: " << endl << inno << endl;

  // Zero innovation, no update
  if( inno.isZero() ) return 0;

  MatrixXd innoCov( states(), states() );
  innoCov = h * _cov * h.transpose() + _r;

  //  cout << "Cov: " << endl << _cov << endl;
  //cout << "Innocov: " << endl << innoCov << endl;

  MatrixXd kg;
  kg = _cov * h.transpose() * innoCov.inverse();

  //  cout << "KG: " << endl << kg << endl;

  _state = _state + kg * inno;
  _cov = ( MatrixXd::Identity( states(), states() ) - kg * h ) * _cov;

//  const double v_limit = 0.05;
//  _state[ states()-1 ] = std::max( -v_limit, std::min( v_limit, _state[ states()-1 ] ) );

  cout << "States after update: " << endl << _state << endl;

  return 0;
}

