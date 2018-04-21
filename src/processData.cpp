/*
 * processData.cpp
 *
 *  Created on: Apr 8, 2018
 *      Author: rohitbahl
 */

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "bayesFilter.hpp"
#include "measurement_package.hpp"
#include "processData.hpp"

#include <tuple>
#include <vector>

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using namespace std;

/**************************************************************************************************
 *  @brief Constructor
 *************************************************************************************************/
ProcessData::ProcessData() : m_pBayesFilter(BayesFilter::createFilter(PARTICLE_FILTER))
{
}

/**************************************************************************************************
 *  @brief Destructor
 *************************************************************************************************/
ProcessData::~ProcessData()
{
}

/**************************************************************************************************
 *  @brief Process the received measurement and predict the position.
 *************************************************************************************************/
tuple<Particle, vector<double>> ProcessData::estimatePosition(const MeasurementPackage &meas_package)
{
    tuple<Particle, vector<double>> tp = m_pBayesFilter->filter(meas_package);

    return tp;
}

