/*
 * Kalman filter for heading using IMU measurements.
 */

#include "estimation/include/HeadingKalmanEstimator.h"

namespace ram {
namespace estimation {

HeadingKalmanEstimator::HeadingKalmanEstimator(core::ConfigNode config,
                                               core::EventHubPtr eventHub,
                                               EstimatedStatePtr estState):
    EstimationModule(eventHub, "HeadingKalmanEstimator",estState,
                     vehicle::device::IIMU::RAW_UPDATE),
    m_basicModule(config, eventHub, estState),
    m_estimate(0.0, 7, 1),
    m_covariance(0.0, 7, 7),
    m_identity(0.0, 7, 7),
    m_Q(0.0, 7, 7),
    m_R(0.0, 7, 7),
    observation_matrix(0.0, 7, 7),
{
    int i;
    int j;
    for (i = 0; i < 4; i ++) {
        for (j = 0; j < 4; j ++) {
            m_Q[i][j] = 5E-9;
        }
    }
    for (i = 0; i < 7; i ++) {
        m_identity[i][i] = 1; 
        m_covariance[i][i] = .5;
        m_Q[i][i] = 5E-8;
        m_observation_matrix[i][i] = 1;
    }

    // Yes this is stupid.
    m_R[0][0] = 1.93075779540854e-05;
    m_R[0][1] = -5.17687681430900e-06;
    m_R[0][2] = 0.000362149251070183;
    m_R[0][3] = -3.51558007693396e-05;
    m_R[0][4] = 1.18548439343944e-05;
    m_R[0][5] = 6.58657132555796e-07;
    m_R[0][6] = 1.71331739626450e-07;
    m_R[1][0] = -5.17687681430900e-06;
    m_R[1][1] = 6.63151149227604e-06;
    m_R[1][2] = -9.08022268698529e-05;
    m_R[1][3] = 1.33323915199394e-05;
    m_R[1][4] = -1.05911808917911e-05;
    m_R[1][5] = -2.00929348467753e-08;
    m_R[1][6] = -8.24732793835018e-07;
    m_R[2][0] = 0.000362149251070183;
    m_R[2][1] = -9.08022268698529e-05;
    m_R[2][2] = 0.00871945330570462;
    m_R[2][3] = 0.000875665590920829;
    m_R[2][4] = 0.000226498136524004;
    m_R[2][5] = -4.22047916255665e-06;
    m_R[2][6] = 2.82916222265447e-06;
    m_R[3][0] = -3.51558007693396e-05;
    m_R[3][1] = 1.33323915199394e-05;
    m_R[3][2] = -0.000875665590920829;
    m_R[3][3] = 0.000101148418931553;
    m_R[3][4] = -3.45071339264182e-05;
    m_R[3][5] = 2.05970488080616e-07;
    m_R[3][6] = -9.86402562483753e-07;
    m_R[4][0] = 1.18548439343944e-05;
    m_R[4][1] = -1.05911808917911e-05;
    m_R[4][2] = 0.000226498136524004;
    m_R[4][3] = -3.45071339264182e-05;
    m_R[4][4] = 9.95525142724403e-05;
    m_R[4][5] = -7.02933227783904e-08;
    m_R[4][6] = -9.79717506470727e-07;
    m_R[5][0] = -6.58657132555796e-07;
    m_R[5][1] = -2.00929348467753e-08;
    m_R[5][2] = -4.22047916255665e-06;
    m_R[5][3] = 2.05970488080616e-07;
    m_R[5][4] = -7.02933227783904e-08;
    m_R[5][5] = 4.28433197645635e-06l;
    m_R[5][6] = -1.40894857301751e-06;
    m_R[6][0] = 1.71331739626450e-07;
    m_R[6][1] = -8.24732793835018e-07;
    m_R[6][2] = 2.82916222265447e-06;
    m_R[6][3] = -9.86402562483753e-07;
    m_R[6][4] = -9.79717506470727e-07;
    m_R[6][5] = -1.40894857301751e-06;
    m_R[6][6] = 5.00966017849615e-06;
}

HeadingKalmanEstimator::~HeadingKalmanEstimator()
{
}
 
math::MatrixN HeadingKalmanEstimator::get_Estimate() {
    return estimate;
}

math::MatrixN HeadingKalmanEstimator::get_Covariance() {
    return covariance;
}

math::MatrixN HeadingKalmanEstimator::get_Identity() {
    return identity;
}

void HeadingKalmanEstimator::update(core::EventPtr event) {
    vehicle::RawIMUDataEventPtr ievent = 
        boost::dynamic_pointer_cast<vehicle::RawIMUDataEvent>(event);
    /* Return if the cast failed and let people know about it. */
    if (!ievent) {
        return;
    }
    m_basicModule.update(event);

    RawIMUData newState = ievent->rawIMUData;
    double dt = ievent->timestep;

    /*
     * Measurement Update
     */
    math::MatrixN zk(0.0, 7, 1);
    zk[0][0] = m_basicModule.getOrientation().w;
    zk[1][0] = m_basicModule.getOrientation().x;
    zk[2][0] = m_basicModule.getOrientation().y;
    zk[3][0] = m_basicModule.getOrientation().z;
    zk[4][0] = m_basicModule.getAngularRate().x;
    zk[5][0] = m_basicModule.getAngularRate().y;
    zk[6][0] = m_basicModule.getAngularRate().z;

    updateOrientation(zk, dt);
}

void updateOrientation(math::MatrixN zk, double dt) {
    math::MatrixN state_transition(0.0, 7, 7);// state transition model
    state_transition[0][0] = 1;
    state_transition[0][1] = -dt*m_estimate[4][0]/2;
    state_transition[0][2] = -dt*m_estimate[5][0]/2;
    state_transition[0][3] = -dt*m_estimate[6][0]/2;
    state_transition[0][4] = -dt*m_estimate[1][0]/2;
    state_transition[0][5] = -dt*m_estimate[2][0]/2;
    state_transition[0][6] = -dt*m_estimate[3][0]/2;

    state_transition[1][0] = -dt*m_estimate[4][0]/2;
    state_transition[1][1] = 1
    state_transition[1][2] = -dt*m_estimate[6][0]/2;
    state_transition[1][3] = -dt*m_estimate[5][0]/2;
    state_transition[1][4] = -dt*m_estimate[0][0]/2;
    state_transition[1][5] = -dt*m_estimate[3][0]/2;
    state_transition[1][6] = -dt*m_estimate[2][0]/2;

    state_transition[2][0] = -dt*m_estimate[5][0]/2;
    state_transition[2][1] = -dt*m_estimate[6][0]/2;
    state_transition[2][2] = 1
    state_transition[2][3] = -dt*m_estimate[4][0]/2;
    state_transition[2][4] = -dt*m_estimate[3][0]/2;
    state_transition[2][5] = -dt*m_estimate[0][0]/2;
    state_transition[2][6] = -dt*m_estimate[1][0]/2;

    state_transition[3][0] = -dt*m_estimate[6][0]/2;
    state_transition[3][1] = -dt*m_estimate[5][0]/2;
    state_transition[3][2] = -dt*m_estimate[4][0]/2;
    state_transition[3][3] = 1
    state_transition[3][4] = -dt*m_estimate[2][0]/2;
    state_transition[3][5] = -dt*m_estimate[1][0]/2;
    state_transition[3][6] = -dt*m_estimate[0][0]/2;
    
    state_transition[4][4] = 1;
    state_transition[5][5] = 1;
    state_transition[6][6] = 1;
 
    // Measurement Prediction
    m_estimate = state_transition * m_estimate;
    m_covariance = state_transition * m_covariance * state_transition.transpose() + m_Q;

    // Measurement Residual
    math::MatrixN y(zk - (m_observation_matrix * m_estimate));

    // Covariance Residual
    math::MatrixN S(m_observation_matrix * m_covariance * m_observation_matrix.transpose() +
	m_R);

    // Optimal Kalman gain
    math::MatrixN K(m_covariance * m_observation_matrix.transpose() * m_S.invert());

    m_estimate = m_estimate + K * y;
    m_covariance = (m_identity - (K * m_observation_matrix)) * m_covariance;

    // Set Estimated State Values
    math::Quaternion orientation(m_estimate[1], m_estimate[2], m_estimate[3], m_estimate[0]);
    m_estimatedState->setEstimatedOrientation(orientation);
    math::Vector3 angularRate(m_estimate[4], m_estimate[5], m_estimate[6])
    m_estimatedState->setEstimatedAngularRate(angularRate);
}
    
} // namespace estimation
} // namespace ram
