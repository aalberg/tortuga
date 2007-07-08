
/*
 * Copyright (C) 2007 Robotics at Maryland
 * Copyright (C) 2007 Joseph Lisee <jlisee@umd.edu>
 * All rights reserved.
 *
 * Author: Joseph Lisee <jlisee@umd.edu>
 * File:  packages/control/include/Controller.h
 */

#ifndef RAM_CONTROL_ICONTROL_07_03_2007
#define RAM_CONTROL_ICONTROL_07_03_2007

// Project Includes
#include "core/include/IUpdatable.h"
#include "pattern/include/Subject.h"

namespace ram {
namespace control {

class IController : public pattern::Subject, public core::IUpdatable
{
public:
    enum UPDATE_EVENTS {
        SPEED_UPDATE,
        HEADING_UPDATE,
        DEPTH_UPDATE
    };
    
    /** Set the current speed, clamped between -5 and 5 */
    virtual void setSpeed(int speed) = 0;

    /** Sets the current heading in degrees off north */
    virtual void setHeading(double degrees) = 0;

    /** Sets the current depth of the sub in meters */
    virtual void setDepth(double depth) = 0;

    virtual int getSpeed() = 0;

    virtual double getHeading() = 0;

    virtual double getDepth() = 0;    
};
    
} // namespace control
} // namespace ram

#endif // RAM_CONTROL_ICONTROL_07_03_2007
