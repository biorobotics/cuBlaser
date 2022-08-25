find_package(raspicam REQUIRED)

MJPG_STREAMER_PLUGIN_OPTION(input_raspicam_2shutter "Raspicam_2shutter input plugin")

if (PLUGIN_INPUT_RASPICAM_2SHUTTER)
    enable_language(CXX)

    MJPG_STREAMER_PLUGIN_COMPILE(input_raspicam_2shutter input_raspicam_2shutter.cpp)
    
    target_link_libraries(input_opencv ${raspicam_LIBS})
    
endif()


