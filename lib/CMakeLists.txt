
fips_begin_lib( aplcam )

  fips_files(
    bits_to_hex.cpp
    types.cpp
    board/board.cpp
    board/circle.cpp
    board/trailer_hitch.cpp
    image.cpp
    video.cpp
    detection/detection.cpp
    detection/circle.cpp
    detection_db.cpp
    #leveldb_detection_db.cpp
    detection_set.cpp
    my_undistort.cpp
    file_utils.cpp
    synchronizer.cpp
    trendnet_time_code.cpp
    distortion/camera_model.cpp
    distortion/pinhole_camera.cpp
    distortion/distortion_model.cpp
    distortion/angular_polynomial.cpp
    distortion/radial_polynomial.cpp
    distortion/opencv_radial_polynomial.cpp
    distortion/ceres_radial_polynomial.cpp
    distortion/camera_factory.cpp
    distortion/distortion_stereo.cpp
    distortion/stereo_calibration.cpp
    motion_model.cpp
    feature_tracker.cpp
    calibration_db.cpp
    calibration_result.cpp
    #leveldb_calibration_db.cpp
    splitter_common.cpp
    hough_circles.cpp
    image_accumulator.cpp
    sonar_pose.cpp
    frame_selector/keyframe_selector.cpp
    calib_frame_selectors/random_frame_selector.cpp
    calib_frame_selectors/keyframe_frame_selector.cpp )

    if( USE_APRILTAGS )
      message( INFO " USE_APRILTAGS set" )
      fips_files(
        april_tag_board_generator.cpp
        april_tag_detection_set.cpp
        board/apriltags.cpp
        detection/apriltags.cpp )

      fips_deps( apriltags )
    else()
      message( INFO " USE_APRILTAGS not set" )
    endif()

  fips_deps( g3logger )

  fips_libs(
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${GSL_LIBRARIES}
    ${CERES_LIBRARIES}
    ${LevelDB_LIBRARIES}
    cryptopp )

fips_end_lib()
