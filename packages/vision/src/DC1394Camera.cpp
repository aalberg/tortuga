/*
 * Copyright (C) 2009 Robotics at Maryland
 * Copyright (C) 2009 Joseph Lisee
 * All rights reserved.
 *
 * Author: Joseph Lisee <jlisee@umd.edu>
 * File:  packages/vision/include/DC1934Camera.cpp
 */

// STD Includes
#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <iostream>

// Library Includes
#include <dc1394/video.h>
#include <dc1394/camera.h>
#include <dc1394/conversions.h>

#include <boost/algorithm/string/case_conv.hpp>

// Project includes
#include "vision/include/DC1394Camera.h"
#include "vision/include/OpenCVImage.h"

// Initialize static variables
dc1394_t* ram::vision::DC1394Camera::s_libHandle = 0;
size_t ram::vision::DC1394Camera::s_camCount = 0;

namespace ram {
namespace vision {

static const int DMA_BUFFER_SIZE = 10;    
    
DC1394Camera::DC1394Camera(core::ConfigNode config) :
    m_width(0),
    m_height(0),
    m_fps(0),
    m_camera(0),
    m_newFrame(0)
{
    initLibDC1394();

    // Grab our list of cameras 
    dc1394camera_list_t * list;
    dc1394error_t err = dc1394_camera_enumerate(s_libHandle, &list);
    assert(DC1394_SUCCESS == err && "Failed to enumerate cameras");
    assert(list->num > 0 && "No cameras found");

    // Create with the first camera 
    init(config, list->ids[0].guid);

    dc1394_camera_free_list(list);
}

DC1394Camera::DC1394Camera(core::ConfigNode config, size_t num) :
    m_width(0),
    m_height(0),
    m_fps(0),
    m_camera(0),
    m_newFrame(0)
{
    initLibDC1394();

    // Grab our list of cameras 
    dc1394camera_list_t * list;
    dc1394error_t err = dc1394_camera_enumerate(s_libHandle, &list);
    assert(DC1394_SUCCESS == err && "Failed to enumerate cameras");
    assert(list->num > 0 && "No cameras found");
    assert(num < list->num && "Num too large");

    // Create with the first camera 
    init(config, list->ids[num].guid);

    dc1394_camera_free_list(list);
}   

DC1394Camera::DC1394Camera(core::ConfigNode config, uint64_t guid) :
    m_width(0),
    m_height(0),
    m_fps(0),
    m_camera(0),
    m_newFrame(0)
{
    initLibDC1394();

    // Create out camera with the desired guid
    init(config, guid);
}

DC1394Camera::~DC1394Camera()
{
    // Stop any background image processing
    cleanup();
    
    dc1394_video_set_transmission(m_camera, DC1394_OFF);
    dc1394_capture_stop(m_camera);
    dc1394_camera_set_power(m_camera, DC1394_OFF);
    dc1394_camera_free(m_camera);

    free(m_newFrame->image);
    free(m_newFrame);
    
    shutdownLibDC1394();
}

void DC1394Camera::update(double timestep)
{
    dc1394video_frame_t* frame = 0;
    dc1394error_t err = DC1394_FAILURE;
    
    // Grab a frame
    err = dc1394_capture_dequeue(m_camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
    assert(DC1394_SUCCESS == err && "Could not capture a frame\n");
    
    // Convert to RGB and place in our new frame
    dc1394_convert_frames(frame, m_newFrame);
//    dc1394_convert_to_RGB8(frame->image, m_newFrame->getData(), 640, 480,
//                           frame->yuv_byte_order, frame->color_coding, 8);

    // Free the space back up on the queue
    err = dc1394_capture_enqueue(m_camera, frame);
    assert(DC1394_SUCCESS == err && "Could not enqueue used frame\n");


    // Copy image to public side of the interface        
    Image* newImage = new OpenCVImage(m_newFrame->image, 640, 480, false);

    // Flip RGB -> BGR
    unsigned char* data = newImage->getData();
    size_t numPixels = m_width * m_height;
    for (size_t i = 0; i < numPixels; ++i)
    {
        unsigned char tmp = *data;
        *data = *(data + 2);
        *(data + 2) = tmp;
        data += 3;
    }
    
    capturedImage(newImage);
    delete newImage;
}

size_t DC1394Camera::width()
{    
    return m_width;
}

size_t DC1394Camera::height()
{
    return m_height;
}

double DC1394Camera::fps()
{
    return m_fps;
}

double DC1394Camera::duration()
{
    return 0;
}

void DC1394Camera::seekToTime(double seconds)
{
}

double DC1394Camera::currentTime()
{
    return 0;
}

void DC1394Camera::init(core::ConfigNode config, uint64_t guid)
{
    // Grab our camera 
    m_camera = dc1394_camera_new(s_libHandle, guid);
    if (!m_camera) {
        std::cout << "Failed to initialize camera with guid: " << guid
                  << std::endl;
        assert(m_camera && "Couldn't initialize camera");
    }

    // Determines settings and frame size
    dc1394error_t err = DC1394_FAILURE;
    dc1394video_mode_t videoMode = DC1394_VIDEO_MODE_640x480_YUV411;
    dc1394framerate_t frameRate = DC1394_FRAMERATE_30;

    // Actually set the values if the user wants to
    if (config.exists("uValue") && config.exists("vValue"))
    {
        bool uAuto =
            boost::to_lower_copy(config["uValue"].asString()) == "auto";
        bool vAuto =
            boost::to_lower_copy(config["vValue"].asString()) == "auto";
        bool autoVal = uAuto && vAuto;

        if ((uAuto || vAuto) && !autoVal)
        {
            assert(false &&
                   "Both Whitebalance values must either be auto or manual");
        }
        
        if (autoVal)
        {
            setWhiteBalance(0, 0, true);
        }
        else
        {
            // Read in config values
            uint32_t u_b_value = (uint32_t)config["uValue"].asInt();
            uint32_t v_r_value = (uint32_t)config["vValue"].asInt();

            // Set values
            setWhiteBalance(u_b_value, v_r_value);
        }
    }
    else if (config.exists("uValue") || config.exists("vValue"))
    {
        assert(false && "Must set both the U and V values for white balance");
    }
    
    if (config.exists("brightness"))
    {
        // Read in and set values
        if (boost::to_lower_copy(config["brightness"].asString()) == "auto")
        {
            setBrightness(0, true);
        }
        else
        {
            uint32_t value = (uint32_t)config["brightness"].asInt();
            setBrightness(value);
        }
    }

    
    // Grab image size
    err = dc1394_get_image_size_from_video_mode(m_camera, videoMode,
                                                &m_width, &m_height);
    assert(DC1394_SUCCESS == err && "Could not get image size");
    
    float fRate;
    err = dc1394_framerate_as_float(frameRate, &fRate);
    assert(DC1394_SUCCESS == err && "Could not get framerate as float");
    m_fps = fRate;

    // Create our RGB version of the frame
    m_newFrame = (dc1394video_frame_t*)calloc(1,sizeof(dc1394video_frame_t));
    m_newFrame->color_coding = DC1394_COLOR_CODING_RGB8;
    
    // Setup the capture
    err = dc1394_video_set_iso_speed(m_camera, DC1394_ISO_SPEED_400);
    assert(DC1394_SUCCESS == err && "Could not set iso speed");
           
    err = dc1394_video_set_mode(m_camera, videoMode);
    assert(DC1394_SUCCESS == err && "Could not set video mode");

    err = dc1394_video_set_framerate(m_camera, frameRate);
    assert(DC1394_SUCCESS == err && "Could not set framerate");

    // Start data transfer
    err = dc1394_video_set_transmission(m_camera, DC1394_ON);
    assert(DC1394_SUCCESS == err && "Could not start camera iso transmission");

    err = dc1394_capture_setup(m_camera, DMA_BUFFER_SIZE,
                               DC1394_CAPTURE_FLAGS_DEFAULT);
    assert(DC1394_SUCCESS == err && "Could not setup camera make sure"
           " that the video mode and framerate are supported by your camera");
}

void DC1394Camera::setBrightness(uint32_t value, bool makeAuto)
{
    // Grab the white balance feature
    dc1394feature_info_t brightness;
    brightness.id = DC1394_FEATURE_BRIGHTNESS;
    dc1394error_t err = dc1394_feature_get(m_camera, &brightness);
    assert(DC1394_SUCCESS == err && "Could not get brightness feature info");
    
    // Make sure its available
    assert((brightness.available == DC1394_TRUE) &&
           "White balance not supported by camera");

    if (makeAuto)
    {
        err = dc1394_feature_set_mode(m_camera, DC1394_FEATURE_BRIGHTNESS,
                                      DC1394_FEATURE_MODE_AUTO);
        assert(DC1394_SUCCESS == err && "Could not set brightness to auto");
    }
    else
    {
        // Set manual mode
        err = dc1394_feature_set_mode(m_camera, DC1394_FEATURE_BRIGHTNESS,
                                      DC1394_FEATURE_MODE_MANUAL);
        assert(DC1394_SUCCESS == err && "Could not set brightness to manual");

        // Error on out of bounds values
        if ((value > brightness.max) || (value < brightness.min))
        {
            fprintf(stderr, "ERROR: Brightness is out of bounds: (%u, %u)\n",
                    brightness.min, brightness.max);
            assert(false && "Brightness out of bounds");
        }
        
        // Set value
        err = dc1394_feature_set_value(m_camera, DC1394_FEATURE_BRIGHTNESS,
                                       value);
        assert(DC1394_SUCCESS == err && "Could not set brightness");
    }

}
    
void DC1394Camera::setWhiteBalance(uint32_t uValue, uint32_t vValue,
                                   bool makeAuto)
{
    // Grab the white balance feature
    dc1394feature_info_t whiteBalance;
    whiteBalance.id = DC1394_FEATURE_WHITE_BALANCE;
    dc1394error_t err = dc1394_feature_get(m_camera, &whiteBalance);
    assert(DC1394_SUCCESS == err && "Could not get white balance feature info");
    
    // Make sure its available
    assert((whiteBalance.available == DC1394_TRUE) &&
           "White balance not supported by camera");

    if (makeAuto)
    {
        err = dc1394_feature_set_mode(m_camera, DC1394_FEATURE_WHITE_BALANCE,
                                      DC1394_FEATURE_MODE_AUTO);
        assert(DC1394_SUCCESS == err && "Could not set whitebalance to auto");
    }
    else
    {
        // Set manual mode
        err = dc1394_feature_set_mode(m_camera, DC1394_FEATURE_WHITE_BALANCE,
                                      DC1394_FEATURE_MODE_MANUAL);
        assert(DC1394_SUCCESS == err && "Could not set whitebalance to manual");

        // Error on out of bounds values
        if (((uValue > whiteBalance.max) || (uValue < whiteBalance.min)) ||
            ((vValue > whiteBalance.max) || (vValue < whiteBalance.min)))
        {
            fprintf(stderr, "ERROR: WhiteBalance out of bounds: (%u, %u)\n",
                    whiteBalance.min, whiteBalance.max);
            assert(false && "White balance out of bounds");
        }
        
        // Set values
        err = dc1394_feature_whitebalance_set_value(m_camera, uValue, vValue);
        assert(DC1394_SUCCESS == err && "Could not set whitebalance");
    }
}

    
void DC1394Camera::initLibDC1394()
{
    if (s_camCount == 0)
    {
        // Create library handle
        s_libHandle = dc1394_new();
    }

   // Increment reference count
    s_camCount++;
}
    
void DC1394Camera::shutdownLibDC1394()
{
    assert(s_camCount > 0 && "Invalid reference count");

    // Decrement reference count
    s_camCount--;
    
    if (s_camCount == 0)
    {
        // Free library
        dc1394_free(s_libHandle);
        s_libHandle = 0;
    }
}
    
} // namespace vision
} // namespace ram