#if !defined(MEASUREMENT_PACKAGE_HPP_)
#define MEASUREMENT_PACKAGE_HPP_

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "helper_functions.h"

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
class MeasurementPackage
{
public:

    MeasurementPackage() : m_delta_t(0.1) {}

    enum SensorType
    {
        GPS,
        FUSED_LIDAR_RADAR
    };

    std::vector<double> m_controlInfo;

    std::vector<LandmarkObs> m_noisyObservations;

    // Time elapsed between measurements [sec]
    double m_delta_t;
};

#endif // MEASUREMENT_PACKAGE_HPP_
