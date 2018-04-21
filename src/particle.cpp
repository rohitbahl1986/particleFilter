/*
 * particle.cpp
 *
 *  Created on: Apr 18, 2018
 *      Author: rohitbahl
 */

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "particle.hpp"

#include <algorithm>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using namespace std;

/**************************************************************************************************
 *  @brief
 *************************************************************************************************/
string Particle::getAssociations()
{
    vector<int> v = associations;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

/**************************************************************************************************
 *  @brief
 *************************************************************************************************/
string Particle::getSenseX()
{
    vector<double> v = sense_x;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

/**************************************************************************************************
 *  @brief
 *************************************************************************************************/
string Particle::getSenseY()
{
    vector<double> v = sense_y;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}


/**************************************************************************************************
 * @brief Set a particles list of associations, along with the associations calculated
 * world x,y coordinates
 *************************************************************************************************/
void Particle::setAssociations(int associations, double x, double y)
{
    this->associations.push_back(associations);
    sense_x.push_back(x);
    sense_y.push_back(y);
}
