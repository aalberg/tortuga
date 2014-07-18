
#ifndef RAM_DVL_ACCELEROMETER_ESTIMATIONMODULE_H
#define RAM_DVL_ACCELEROMETER_ESTIMATIONMODULE_H

// STD Includes

// Library Includes
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

// Project Includes
#include "estimation/include/EstimatedState.h"
#include "estimation/include/EstimationModule.h"

#include "core/include/ConfigNode.h"
#include "core/include/Event.h"
#include "core/include/EventPublisher.h"
#include "core/include/EventConnection.h"

#include "vehicle/include/Events.h"
#include "vehicle/include/device/IMU.h"

#include "math/include/Math.h"
#include "math/include/MatrixN.h"
#include "math/include/VectorN.h"
#include "math/include/Vector3.h"
#include "math/include/Vector2.h"
#include "math/include/Quaternion.h"

namespace ram {
namespace estimation {

class HeadingKalmanEstimator : public EstimationModule
{
  public:
    HeadingKalmanEstimator(core::ConfigNode config,
                           core::EventHubPtr eventHub =  core::EventHubPtr(), 
                           EstimatedStatePtr estState  = EstimatedStatePtr());

    math::MatrixN get_Estimate();
    math::MatrixN get_Covariance();
    math::MatrixN get_Identity();

    virtual ~HeadingKalmanEstimator();
    virtual void update(core::EventPtr event);
    virtual void updateOrientation(math::MatrixN zk, double dt);
    
  protected:
    std::string m_name;
    EstimatedStatePtr m_estimatedState;
    BasicIMUEstimationModule m_basicModule; 
 
    math::MatrixN m_estimate;
    math::MatrixN m_covariance;
    math::MatrixN m_identity; 
};

} // estimation
} // ram

#endif
