/*
 * particle.hpp
 *
 *  Created on: Apr 17, 2018
 *      Author: rohitbahl
 */

#if !defined(PARTICLE_HPP_)
#define PARTICLE_HPP_

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include <vector>
#include <string>

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
class Particle
{
public:
    int id;
    double x;
    double y;
    double theta;
    double weight;
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;

    std::string getAssociations();
    std::string getSenseX();
    std::string getSenseY();
    void setAssociations(int associations, double x, double y);

};

#endif // PARTICLE_HPP_
