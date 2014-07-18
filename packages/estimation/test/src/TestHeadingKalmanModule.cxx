/*
 * Copyright (C) 2010 Robotics at Maryland
 * Copyright (C) 2010 Jonathan Wonders <jwonders@umd.edu>
 * All rights reserved.
 *
 * Author: Jonathan Wonders <jwonders@umd.edu>
 * File:  packages/estimation/test/src/TestBasicIMUEstimationModule.cxx
 */

// Library Includes
#include <UnitTest++/UnitTest++.h>

// Project Includes
#include "estimation/include/modules/HeadingKalmanModule.h"
#include "estimation/include/EstimatedState.h"
#include "vehicle/include/Events.h"
#include "core/include/EventHub.h"

using namespace ram;

TEST(headingKalmanModule)
{
    core::EventHubPtr eventHub = core::EventHubPtr(
        new core::EventHub("eventHub"));

    estimation::EstimatedStatePtr estimatedState = 
        estimation::EstimatedStatePtr(
            new estimation::EstimatedState(core::ConfigNode::fromString("{}"),
                                           eventHub));

    estimation::EstimationModulePtr module = 
        estimation::EstimationModulePtr(
            new estimation::BasicIMUEstimationModule(
                core::ConfigNode::fromString("{}"),
                eventHub, estimatedState));

    for (int i = 0; i < 102; i++)
    {
        module->update(cgEvent);
        module->update(magEvent);
    }
}
