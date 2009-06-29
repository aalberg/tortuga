
/*
 * Copyright (C) 2008 Robotics at Maryland
 * Copyright (C) 2008 Daniel Hakim <dhakim@umd.edu>
 * All rights reserved.
 *
 * Author: Daniel Hakim <dhakim@umd.edu>
 * File:  packages/vision/test/src/TestOrangePipeDetector.cxx
 */

// STD Includes
#include <cmath>
#include <set>
#include <iostream>

// Library Includes
#include <UnitTest++/UnitTest++.h>

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <cv.h>

// Project Includes
#include "vision/include/Image.h"
#include "vision/include/BinDetector.h"
#include "vision/include/OpenCVImage.h"
#include "vision/include/Events.h"

#include "core/include/EventHub.h"

#include "vision/test/include/Utility.h"

#include "math/include/Matrix3.h"

using namespace ram;

/*static boost::filesystem::path getImagesDir()
{
    boost::filesystem::path root(getenv("RAM_SVN_DIR"));
    return root / "packages" / "vision" / "test" / "data" / "references";
    }*/

struct BinDetectorFixture
{
    BinDetectorFixture() :
        found(false),
        centered(false),
        dropped(false),
        receivedMultiBinAngleEvent(false),
        event(vision::BinEventPtr()),
        droppedEvent(vision::BinEventPtr()),
        input(640, 480),
        eventHub(new core::EventHub()),
        detector(core::ConfigNode::fromString("{}"), eventHub)
    {
        // Subscribe to events like so
        eventHub->subscribeToType(vision::EventType::BIN_FOUND,
            boost::bind(&BinDetectorFixture::foundHandler, this, _1));
        eventHub->subscribeToType(vision::EventType::BIN_CENTERED,
            boost::bind(&BinDetectorFixture::centeredHandler, this, _1));
        eventHub->subscribeToType(vision::EventType::BINS_LOST,
            boost::bind(&BinDetectorFixture::lostHandler, this, _1));
        eventHub->subscribeToType(vision::EventType::BIN_DROPPED,
            boost::bind(&BinDetectorFixture::droppedHandler, this, _1));
        eventHub->subscribeToType(vision::EventType::MULTI_BIN_ANGLE,
            boost::bind(&BinDetectorFixture::multiBinAngleHandler, this, _1));
        detector.setSymbolDetectionOn(false);
    }

    void foundHandler(core::EventPtr event_)
    {
        found = true;
        event = boost::dynamic_pointer_cast<vision::BinEvent>(event_);
    }

    void centeredHandler(core::EventPtr event_)
    {
        centered = true;
        event = boost::dynamic_pointer_cast<vision::BinEvent>(event_);
    }

    void lostHandler(core::EventPtr event_)
    {
        found = false;
        event = vision::BinEventPtr();
    }

    void droppedHandler(core::EventPtr event_)
    {
        dropped = true;
        droppedEvent = boost::dynamic_pointer_cast<vision::BinEvent>(event_);
    }
    
    void multiBinAngleHandler(core::EventPtr event_)
    {
        multiBinAngleEvent = boost::dynamic_pointer_cast<vision::BinEvent>(event_);
        receivedMultiBinAngleEvent = true;
    }

    void processImage(vision::Image* image, bool show = false)
    {
        if (show)
	{
	    vision::OpenCVImage input(640, 480);
	    input.copyFrom(image);
	    vision::Image::showImage(&input, "Input");

	    vision::OpenCVImage output(640, 480);
	    detector.processImage(image, &output);
	    vision::Image::showImage(&output, "Output");
	}
	else
        {
            detector.processImage(image);
	}
    }

    
    bool found;
    bool centered;
    bool dropped;
    bool receivedMultiBinAngleEvent;
    vision::BinEventPtr multiBinAngleEvent;
    vision::BinEventPtr event;
    vision::BinEventPtr droppedEvent;
    vision::OpenCVImage input;
    core::EventHubPtr eventHub;
    vision::BinDetector detector;
};

SUITE(BinDetector) {

// TODO Test upright, and onside angle

TEST_FIXTURE(BinDetectorFixture, multiBinAngles3)
{
    detector.setSymbolDetectionOn(false);
    receivedMultiBinAngleEvent = false;
    
    vision::makeColor(&input, 0, 0, 255);
    drawBin(&input, 640/5, 480/4, 125, 90);
    drawBin(&input, 640*2/5 + 5, 480/4 + 640/5, 125, 90);
    drawBin(&input, 640*3/5 - 3, 480/4 + 640*2/5, 125, 90);
    drawBin(&input, 640*4/5, 480/4 + 640*3/5, 125, 90);
    
    processImage(&input);
    CHECK(receivedMultiBinAngleEvent);
    CHECK(multiBinAngleEvent);
    
    if (receivedMultiBinAngleEvent);
    {
        receivedMultiBinAngleEvent = false;
        CHECK_CLOSE(-45, multiBinAngleEvent->angle.valueDegrees(),2);
    }
}

TEST_FIXTURE(BinDetectorFixture, multiBinAngles2)
{
    detector.setSymbolDetectionOn(false);
    receivedMultiBinAngleEvent = false;
    
    vision::makeColor(&input, 0, 0, 255);
    drawBin(&input, 640/4, 480/4, 125, 90);
    drawBin(&input, 640/2, 480/4 + 640/4, 125, 90);
    
    processImage(&input);
    CHECK(receivedMultiBinAngleEvent);
    CHECK(multiBinAngleEvent);
    
    if (receivedMultiBinAngleEvent);
    {
        receivedMultiBinAngleEvent = false;
        CHECK_CLOSE(-45, multiBinAngleEvent->angle.valueDegrees(),.25);
    }

    vision::makeColor(&input, 0, 0, 255);
    drawBin(&input, 640*3/4, 480/4, 125, 90);
    drawBin(&input, 640/2, 480/4 + 640/4, 125, 90);
    
    processImage(&input);
    CHECK(receivedMultiBinAngleEvent);
    CHECK(multiBinAngleEvent);
    
    if (receivedMultiBinAngleEvent);
    {
        receivedMultiBinAngleEvent = false;
        CHECK_CLOSE(45, multiBinAngleEvent->angle.valueDegrees(),.25);
    }
}

TEST_FIXTURE(BinDetectorFixture, multiBinAngles)
{
    printf("Starting Multi Bin Angles\n");
    detector.setSymbolDetectionOn(false);
    receivedMultiBinAngleEvent = false;
    
    // Blue Image with orange rectangle in it
    vision::makeColor(&input, 0, 0, 255);
    processImage(&input);
    CHECK(!receivedMultiBinAngleEvent);
    
    drawBin(&input, 640/2, 480/4, 125, 90);
    
    processImage(&input);
    CHECK(!receivedMultiBinAngleEvent); 
    
    drawBin(&input, 640/2, 480*3/4, 125, 90);
    processImage(&input);
    CHECK(receivedMultiBinAngleEvent);
    CHECK(multiBinAngleEvent);
    
    if (receivedMultiBinAngleEvent)
    {
        receivedMultiBinAngleEvent = false;
        CHECK_CLOSE(-90, multiBinAngleEvent->angle.valueDegrees(),.25);
    }


    vision::makeColor(&input, 0, 0, 255);
    processImage(&input);
    CHECK(!receivedMultiBinAngleEvent);
    
    drawBin(&input, 640/4, 480/3, 125, 0);
    drawBin(&input, 640/4, 480*2/3, 125, 0);
    
    processImage(&input);
    CHECK(receivedMultiBinAngleEvent);
    CHECK(multiBinAngleEvent);
    
    if (receivedMultiBinAngleEvent)
    {
        receivedMultiBinAngleEvent = false;
        CHECK_CLOSE(-90, multiBinAngleEvent->angle.valueDegrees(),.25);
    }

    
    vision::makeColor(&input, 0, 0, 255);
    processImage(&input);
    CHECK(!receivedMultiBinAngleEvent);
    
    drawBin(&input, 640/4, 480/3, 125, 0);
    drawBin(&input, 640*3/4, 480/3, 125, 0);
    
    processImage(&input);
    CHECK(receivedMultiBinAngleEvent);
    CHECK(multiBinAngleEvent);
    
    if (receivedMultiBinAngleEvent)
    {
        receivedMultiBinAngleEvent = false;
        CHECK_CLOSE(0, multiBinAngleEvent->angle.valueDegrees(),.25);
    }
}

TEST_FIXTURE(BinDetectorFixture, UpperLeft)
{
    detector.setSymbolDetectionOn(false);

    // Blue Image with orange rectangle in it
    vision::makeColor(&input, 0, 0, 255);
    // draw the bin (upper left)
    drawBin(&input, 640/4, 480/4, 130, 25);

    // Process it
    processImage(&input);
    double expectedX = -0.5 * 640.0/480.0;
    double expectedY = 0.5;
    math::Degree expectedAngle(25);
    
    CHECK(detector.found());
    CHECK_CLOSE(expectedX, detector.getX(), 0.05);
    CHECK_CLOSE(expectedY, detector.getY(), 0.05);

    // Check Events
    CHECK(found);
    CHECK(event);
    CHECK_CLOSE(expectedX, event->x, 0.05);
    CHECK_CLOSE(expectedY, event->y, 0.05);
}

TEST_FIXTURE(BinDetectorFixture, BinSpinAngleTest)
{
    printf("Starting BinSpinAngleTest:\n");
    double thresh = 3;
    for (int deg = -180; deg < 180; deg+=13)
    { 
        int deg2 = deg%180;
        if (deg2 >= 90)
            deg2-=180;
        else if (deg2 < -90)
            deg2+=180;
        vision::makeColor(&input, 0, 0, 255);
        vision::drawBin(&input, 320, 240, 150, deg, vision::Heart);
        processImage(&input);
        CHECK(event);
        
        bool good = true;
        if (event)
        {
            float angle = event->angle.valueDegrees();
            good = false;
            if (deg2 - thresh <= angle && deg2 + thresh >= angle)
            {
                //good
                good = true;
            }
            else
            {
                if (angle > 90-thresh)
                {
                    angle-=180;
                    if (deg2 - thresh <= angle && deg2 + thresh >= angle)
                    {
                        //good
                        good = true;
                    }
                }
                else if (angle < -90 + thresh)
                {
                    angle+=180;
                    if (deg2 - thresh <= angle && deg2 + thresh >= angle)
                    {
                        //good
                        good = true;
                    }
                }
            }
            
            if (!good)
            {
                //This is guaranteed to fail, so CHECK(false) would work, but 
                //this will display more info.
                CHECK_CLOSE(deg2, event->angle.valueDegrees(), thresh);
            }
        }
//        printf("\n");
    }
    printf("Finished BinSpinAngleTest:\n");

}
/*
FIX ME: I AM BROKEN!
  
TEST_FIXTURE(BinDetectorFixture, SuperSymbolTest)
{ 
    detector.setSymbolDetectionOn(true);
    int right = 0;
    for (int deg = 0; deg < 360; deg+=10)
    {
        
        vision::makeColor(&input, 0, 0, 255);
        vision::drawBin(&input, 100,100, 200, deg, vision::Club);
        vision::OpenCVImage output(640,480);
        processImage(&input);
        if (detector.getSymbol() == vision::Symbol::CLUB)
        {
            right++;
        }
        else if (detector.getSymbol() == vision::Symbol::UNKNOWN)
        {
            
        }
        else
        {
            CHECK(false);
        }
//        vision::Image::showImage(&output);
    }
    //CHECK(right >= 12);
    
    right = 0;
    for (int deg = 0; deg < 360; deg+=10)
    {
        vision::makeColor(&input, 0, 0, 255);
        vision::drawBin(&input, 320,240, 200, deg, vision::Spade);
        vision::OpenCVImage output(640,480);
        processImage(&input);
        if (detector.getSymbol() == vision::Symbol::SPADE)
        {
            right++;
        }
        else if (detector.getSymbol() == vision::Symbol::UNKNOWN)
        {
            
        }
        else
        {
            CHECK(false);
        }
//        vision::Image::showImage(&output);
    }
    //CHECK(right >= 12);

    right = 0;
    for (int deg = 0; deg < 360; deg+=10)
    {
        vision::makeColor(&input, 0, 0, 255);
        vision::drawBin(&input, 320,240, 200, deg, vision::Heart);
        vision::OpenCVImage output(640,480);
        processImage(&input);
        if (detector.getSymbol() == vision::Symbol::HEART)
        {
            right++;
        }
        else if (detector.getSymbol() == vision::Symbol::UNKNOWN)
        {
            
        }
        else
        {
            CHECK(false);
        }

    }
    //CHECK(right >= 12);

    right = 0;
    for (int deg = 0; deg < 360; deg+=10)
    {
        vision::makeColor(&input, 0, 0, 255);
        vision::drawBin(&input, 320,240, 200, deg, vision::Diamond);
        vision::OpenCVImage output(640,480);
        processImage(&input);
        if (detector.getSymbol() == vision::Symbol::DIAMOND)
        {
            right++;
        }
        else if (detector.getSymbol() == vision::Symbol::UNKNOWN)
        {
            
        }
        else
        {
            CHECK(false);
        }

}
    //CHECK(right >= 12);

    detector.setSymbolDetectionOn(false);
}
*/
/*TEST_FIXTURE(BinDetectorFixture, FourBins)
{
    detector.setSymbolDetectionOn(true);
    vision::Image* ref = vision::Image::loadFromFile(
        ((getImagesDir()/"distorted-grainy.png").string()));//Negative flag means load as is, positive means force 3 channel, 0 means force grayscale
    
    CHECK(ref != NULL);
    
    detector.processImage(ref);
    detector.setSymbolDetectionOn(false);
}
*/
TEST_FIXTURE(BinDetectorFixture, BinTracking)
{
    detector.setSymbolDetectionOn(false);
    vision::makeColor(&input, 0, 0, 255);
    vision::drawBin(&input, 160,240, 150, 70, vision::Heart);
    
    vision::drawBin(&input, 480,240, 150, 70, vision::Diamond);
   
    processImage(&input);
   
    vision::BinDetector::BinList bins = detector.getBins();
    std::vector<vision::BinDetector::Bin> binVect(bins.size());
    std::copy(bins.begin(), bins.end(), binVect.begin());

    CHECK_EQUAL(2u, bins.size());
    
    int minId = binVect[0].getId();
    int maxId = binVect[1].getId();
    
    if (maxId < minId)
    {
        minId = minId^maxId;
        maxId = maxId^minId;
        minId = minId^maxId;
    }
    
    CHECK_EQUAL(0, minId);
    CHECK_EQUAL(1, maxId);
    
    vision::makeColor(&input, 0, 0, 255);
    vision::drawBin(&input, 160,240, 150, 70, vision::Heart);
    
    vision::drawBin(&input, 480,240, 150, 70, vision::Diamond);

    // Process Images
    processImage(&input);
    bins = detector.getBins();
    binVect.reserve(bins.size());
    std::copy(bins.begin(), bins.end(), binVect.begin());

    minId = binVect[0].getId();
    maxId = binVect[1].getId();
    
    if (maxId < minId)
    {
        minId = minId^maxId;
        maxId = maxId^minId;
        minId = minId^maxId;
    }
    
    CHECK_EQUAL(0, minId);
    CHECK_EQUAL(1, maxId);
}

TEST_FIXTURE(BinDetectorFixture, Left)
{
    detector.setSymbolDetectionOn(false);
    // Blue Image with orange rectangle in it
    vision::makeColor(&input, 0, 0, 255);
    // draw the bin (left)
    drawBin(&input, 640/4, 480/2, 130, 0);

    // Process it
    processImage(&input);
    
    double expectedX = -0.5 * 640.0/480.0;
    double expectedY = 0;
    
    CHECK(detector.found());
    CHECK_CLOSE(expectedX, detector.getX(), 0.05);
    CHECK_CLOSE(expectedY, detector.getY(), 0.05);

    // Check Events
    CHECK(found);
    CHECK(event);
    CHECK_CLOSE(expectedX, event->x, 0.05);
    CHECK_CLOSE(expectedY, event->y, 0.05);
}

TEST_FIXTURE(BinDetectorFixture, LowerRight)
{
    detector.setSymbolDetectionOn(false);
    // Blue Image with orange rectangle in it
    vision::makeColor(&input, 0, 0, 255);
    // draw the bin (lower right)
    drawBin(&input, 640 - (640/4), 480/4 * 3, 130, -25);

    // Process it
    processImage(&input);
    
    double expectedX = 0.5 * 640.0/480.0;
    double expectedY = -0.5;
    
    CHECK(detector.found());
    CHECK_CLOSE(expectedX, detector.getX(), 0.05);
    CHECK_CLOSE(expectedY, detector.getY(), 0.05);

    // Check Events
    CHECK(found);
    CHECK(event);
    CHECK_CLOSE(expectedX, event->x, 0.05);
    CHECK_CLOSE(expectedY, event->y, 0.05);
}

TEST_FIXTURE(BinDetectorFixture, CenterUp)
{
    detector.setSymbolDetectionOn(false);
    // Blue Image with orange rectangle in it
    vision::makeColor(&input, 0, 0, 255);
    // draw the bin in center sideways
    drawBin(&input, 640/2, 480/2, 130, 0);

    // Process it
    processImage(&input);

    double expectedX = 0 * 640.0/480.0;
    double expectedY = 0;
    
    CHECK(detector.found());
    CHECK_CLOSE(expectedX, detector.getX(), 0.05);
    CHECK_CLOSE(expectedY, detector.getY(), 0.05);

    // Check Events
    CHECK(found);
    CHECK(event);
    CHECK_CLOSE(expectedX, event->x, 0.05);
    CHECK_CLOSE(expectedY, event->y, 0.05);
}

TEST_FIXTURE(BinDetectorFixture, CenterSideways)
{
    detector.setSymbolDetectionOn(false);
    // Blue Image with orange rectangle in it
    vision::makeColor(&input, 0, 0, 255);
    // draw the bin in center sideways
    drawBin(&input, 640/2, 480/2, 130, 90);
    
    // Process it
    processImage(&input);
    
    double expectedX = 0 * 640.0/480.0; 
    double expectedY = 0;
    
    CHECK(detector.found());
    CHECK_CLOSE(expectedX, detector.getX(), 0.05);
    CHECK_CLOSE(expectedY, detector.getY(), 0.05);

    // Check Events
    CHECK(found);
    CHECK(event);
    CHECK_CLOSE(expectedX, event->x, 0.05);
    CHECK_CLOSE(expectedY, event->y, 0.05);
}

TEST_FIXTURE(BinDetectorFixture, Events_BINS_LOST)
{
    detector.setSymbolDetectionOn(false);
    // No light at all
    makeColor(&input, 0, 0, 255);
    
    processImage(&input);
    CHECK(found == false);
    CHECK(!event);

    // Now we found the bin (lower right location)
    drawBin(&input, 640 - (640/4), 480/4 * 3, 130, -25);
    processImage(&input);
    CHECK(found);
    CHECK(event);
    CHECK_CLOSE(0.5  * 640.0/480.0, event->x, 0.05);
    CHECK_CLOSE(-0.5, event->y, 0.05);

    // Now we lost the bin
    makeColor(&input, 0, 0, 255);
    processImage(&input);
    CHECK(found == false);
    CHECK(!event);

    // Make sure we don't get another lost event
    found = true;
    processImage(&input);
    CHECK(found == true);
}

TEST_FIXTURE(BinDetectorFixture, Events_BIN_DROPPED)
{
    detector.setSymbolDetectionOn(false);
    // Place a set of four bins on screen
    makeColor(&input, 0, 0, 255);
    drawBin(&input, 640/5, 480/2, 100, 0);
    drawBin(&input, 640/5 * 2, 480/2, 100, 0);
    drawBin(&input, 640/5 * 3, 480/2, 100, 0);
    drawBin(&input, 640/5 * 4, 480/2, 100, 0);
    
    processImage(&input);

    // Make sure we haven't dropped anything
    CHECK_EQUAL(false, dropped);
    
    // Ensure unique IDs
    vision::BinDetector::BinList bins = detector.getBins();
    CHECK_EQUAL(4u, bins.size());
    std::set<int> startingIds;
    BOOST_FOREACH(vision::BinDetector::Bin bin, bins)
    {
        startingIds.insert(bin.getId());
    }
    CHECK_EQUAL(4u, startingIds.size());
    
    // Record ID of bin, then nuke it
    makeColor(&input, 0, 0, 255);
    drawBin(&input, 640/5, 480/2, 100, 0);
    drawBin(&input, 640/5 * 2, 480/2, 100, 0);
    drawBin(&input, 640/5 * 3, 480/2, 100, 0);    

    processImage(&input);
    bins = detector.getBins();
    CHECK_EQUAL(3u, bins.size());
    std::set<int> endingIds;
    BOOST_FOREACH(vision::BinDetector::Bin bin, bins)
    {
        endingIds.insert(bin.getId());
    }
    CHECK_EQUAL(3u, endingIds.size());

    // Find the difference
    std::set<int> lostIds;
    std::set_difference(startingIds.begin(), startingIds.end(),
                        endingIds.begin(), endingIds.end(),
                        std::inserter(lostIds, lostIds.begin()));
    CHECK_EQUAL(1u, lostIds.size());
    
    // Now make sure we got the right dropped event
    CHECK(dropped);
    CHECK(droppedEvent);
    CHECK_EQUAL((*lostIds.begin()), droppedEvent->id);
}

TEST_FIXTURE(BinDetectorFixture, Events_BIN_CENTERED)
{
    detector.setSymbolDetectionOn(false);
    // Bin in the lower right
    makeColor(&input, 0, 0, 255);
    drawBin(&input, 640 - (640/4), 480/4 * 3, 130, 0);
    processImage(&input);
    CHECK(found);
    CHECK(event);
    CHECK(!centered);
    CHECK_CLOSE(0.5 * 640.0/480.0, event->x, 0.05);
    CHECK_CLOSE(-0.5, event->y, 0.05);    

    // Now bin is dead center
    makeColor(&input, 0, 0, 255);
    drawBin(&input, 640/2, 480/2, 130, 0);
    processImage(&input);
    CHECK(found);

    // Make sure we get the centered event
    CHECK(centered);
    CHECK(event);
    CHECK_CLOSE(0, event->x, 0.05);
    CHECK_CLOSE(0, event->y, 0.05);

    // Blank image
    centered = false;
    found = false;
    event = vision::BinEventPtr();
    makeColor(&input, 0, 0, 255);
    processImage(&input);

    CHECK(!event);
    CHECK(!found);
    CHECK(!centered);

    // Now make a dead centered bin, and make sure get another centered
    makeColor(&input, 0, 0, 255);
    drawBin(&input, 640/2, 480/2, 130, 0);
    processImage(&input);

    CHECK(found);
    CHECK(centered);
    CHECK(event);
    CHECK_CLOSE(0, event->x, 0.05);
    CHECK_CLOSE(0, event->y, 0.05);
}
/*
TEST_FIXTURE(BinDetectorFixture, Symbol)
{
    // Bin in center
    makeColor(&input, 0, 0, 255);
    drawBin(&input, 200, 150, 130, 25, vision::Heart);
//    vision::Image::showImage(&input);
}
*/
} // SUITE(BinDetector)
