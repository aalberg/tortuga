/*
 * Copyright (C) 2008 Robotics at Maryland
 * Copyright (C) 2008 Joseph Lisee <jlisee@umd.edu>
 * All rights reserved.
 *
 * Author: Joseph Lisee <jlisee@umd.edu>
 * File:  packages/math/include/Events.h
 */

#ifndef RAM_MATH_EVENTS_01_08_2008
#define RAM_MATH_EVENTS_01_08_2008

// Project Includes
#include "core/include/Event.h"
#include "math/include/Quaternion.h"
#include "math/include/Vector3.h"

namespace ram {
namespace math {
namespace impl {

struct OrientationEvent : public core::Event
{
    Quaternion orientation;
};

typedef boost::shared_ptr<OrientationEvent> OrientationEventPtr;

struct Vector3Event : public core::Event
{
    Vector3 vector3;
};

typedef boost::shared_ptr<Vector3Event> Vector3EventPtr;
    
struct NumericEvent : public core::Event
{
    double number;
};

typedef boost::shared_ptr<NumericEvent> NumericEventPtr;

} // namespace impl
} // namespace math
} // namespace ram

#endif // RAM_MATH_EVENTS_01_08_2008