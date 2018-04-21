/*
 * processData.hpp
 *
 *  Created on: Apr 8, 2018
 *      Author: rohitbahl
 */

#if !defined(PROCESS_DATA_HPP_)
#define PROCESS_DATA_HPP_

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "bayesFilter.hpp"

#include <memory>
#include <tuple>
#include <vector>

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
class ProcessData
{
public:
    ProcessData();

    ~ProcessData();

    /**
     *  @brief Process the received measurement and predict the position.
     */
    std::tuple<Particle, std::vector<double>> estimatePosition(const MeasurementPackage &meas_package);

private:

    std::unique_ptr<BayesFilter> m_pBayesFilter;
};

#endif /// PROCESS_DATA_HPP_
