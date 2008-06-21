/*
 * Copyright (C) 2008 Robotics at Maryland
 * Copyright (C) 2008 Joseph Lisee
 * All rights reserved.
 *
 * Author: Joseph Lisee <jlisee@umd.edu>
 * File:  packages/vision/src/Image.cpp
 */

// STD Includes
#include <iostream>

// Library Includes
#include "cv.h"
#include "highgui.h"

// Project Includes
#include "vision/include/OpenCVImage.h"

static const char* DEBUG_WINDOW = "Debug Image (close w/ESC Key)";

namespace ram {
namespace vision {

Image* Image::loadFromFile(std::string fileName)
{
    IplImage* image = cvLoadImage(fileName.c_str());
    if (image)
	{
        return new OpenCVImage(image, true);
	}
	else
	{
		std::cout << "Could not load file: \"" << fileName << "\"" << std::endl;
		assert(false && "Could not load image");
	}
    return 0;
}

void Image::saveToFile(Image* image, std::string fileName)
{
    cvSaveImage(fileName.c_str(), image->asIplImage());
}

void Image::rotateAndScale(Image* src, Image* dest,
                           math::Degree rotation, double scale)
{
    float m[6] = {0};
    CvMat M = cvMat( 2, 3, CV_32F, m );
    
    double rotateFactor = rotation.valueRadians();
    double scaleFactor = 1;
    if(1.0 != scale)
        scaleFactor = 1/scale;
    
    m[0] = (float)(scaleFactor * cos(rotateFactor));
    m[1] = (float)(scaleFactor * sin(rotateFactor));
    m[2] = src->getWidth() * 0.5f;
    m[3] = -m[1];
    m[4] = m[0];
    m[5] = src->getHeight() * 0.5f;
    
    cvGetQuadrangleSubPix(src->asIplImage(), dest->asIplImage(), &M);
}

void Image::blitImage(Image* toBlit, Image* src, Image* dest,
                      unsigned char R, unsigned char G, unsigned char B)
{
    size_t toBlitBytes = toBlit->getWidth() * toBlit->getHeight() * 3;
    size_t destBytes = dest->getWidth() * dest->getHeight() * 3;
    size_t srcBytes = src->getWidth() * src->getHeight() * 3;
    assert((destBytes == toBlitBytes) && (toBlitBytes == srcBytes) &&
           "Images are not the same size");

    unsigned char* srcData = src->getData();
    unsigned char* blitData = toBlit->getData();
    unsigned char* destData = dest->getData();

    for (size_t i = 0; i < toBlitBytes; i += 3)
    {
        unsigned char b = blitData[i];
        unsigned char g = blitData[i + 1];
        unsigned char r = blitData[i + 2];
        if ((b == B) && (g == G) && (r == R))
        {
            // We have clear color, copy from source
            destData[i] = srcData[i];
            destData[i + 1] = srcData[i + 1];
            destData[i + 2] = srcData[i + 2];
        }
        else
        {
            // No clear color, copy from blit image
            destData[i] = b;
            destData[i + 1] = g;
            destData[i + 2] = r;
        }
    }
}
    
void Image::showImage(Image* image)
{
    cvNamedWindow(DEBUG_WINDOW, CV_WINDOW_AUTOSIZE);

    while(1)
    {
        cvShowImage(DEBUG_WINDOW, image->asIplImage());

        // Check for escape key
        char key;
        if ((key = (char)(cvWaitKey(10) & 255)) == 27)
            break;
    }

    cvDestroyWindow(DEBUG_WINDOW);
}
    
Image* Image::loadFromBuffer(unsigned char* buffer, int width, int height,
                             bool ownership)
{
    return new OpenCVImage(buffer, width, height, ownership);
}

} // namespace vision
} // namespace ram
