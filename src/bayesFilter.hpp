/*
 * bayesFilter.hpp
 *
 *  Created on: Apr 8, 2018
 *      Author: rohitbahl
 */

#if !defined(BAYES_FILTER_HPP_)
#define BAYES_FILTER_HPP_

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "measurement_package.hpp"
#include "particle.hpp"

#include <tuple>
#include <vector>

/**************************************************************************************************
 *  TYPES
 *************************************************************************************************/
enum TypeOfBayesFilter
{
    PARTICLE_FILTER
};

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
/**
 * @brief This class provides abstraction for any bayesian filter.
 * Every bayesian state prediction algorithm must implement this class.
 */

class BayesFilter
{
public:
    BayesFilter()
    {
    };

    virtual ~BayesFilter()
    {
    };

    /**
     * @brief Factory method to create objects of the bayesian filter.
     * @param[in] : Type of bayesian filter
     * @return : A pointer to
     */
    static BayesFilter *createFilter(TypeOfBayesFilter bayesFilter);

    /**
     * @breif Invoke the processing for the filter.
     * @param[in] Measurement package
     * @return Return a tuple consisting of the estimated position and the error of the filter.
     */
    virtual std::tuple<Particle, std::vector<double>> filter(const MeasurementPackage &meas_package) = 0;
};

#endif // BAYES_FILTER_HPP_
