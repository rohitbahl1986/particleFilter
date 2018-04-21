/*
 * bayesFilter.cpp
 *
 *  Created on: Apr 17, 2018
 *      Author: rohitbahl
 */

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "bayesFilter.hpp"
#include "particleFilter.hpp"

/**************************************************************************************************
 *  CLASS METHODS
 *************************************************************************************************/

/**************************************************************************************************
 *  @brief Factory method to create objects of the bayesian filter.
 *************************************************************************************************/
BayesFilter *BayesFilter::createFilter(TypeOfBayesFilter bayesFilter)
{
    if (PARTICLE_FILTER == bayesFilter)
    {
        return (new ParticleFilter);
    }
    else
    {
        return nullptr;
    }
}
