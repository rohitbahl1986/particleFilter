/*
 * particle_filter.cpp
 */

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "particleFilter.hpp"

#include <algorithm>
#include <assert.h>
#include <iostream>
#include <iterator>
#include <math.h>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using namespace std;

/**************************************************************************************************
 *  @brief Constructor
 *************************************************************************************************/
ParticleFilter::ParticleFilter() :
        m_numParticles(500), m_particles(m_numParticles), m_filterState(INIT), m_sigmaGps { 0.3, 0.3, 0.01 }, m_sigmaLandmark {
            0.3, 0.3 }, m_sensorRange(50), m_numericalTolerance(0.000001)
{
    // Read map data
    if (!read_map_data("../data/map_data.txt", m_map))
    {
        cout << "Error: Could not open map file" << endl;
    }

    for (uint32_t i = 0; i < m_numParticles; ++i)
    {
        m_particles[i].weight = 1;
        m_particles[i].id = i;
    }
}

/**************************************************************************************************
 *  @brief
 *************************************************************************************************/
tuple<Particle, vector<double>> ParticleFilter::filter(const MeasurementPackage &meas_package)
{
    if (INIT == m_filterState)
    {
        init(meas_package);
        m_filterState = ACTIVE;
    }
    else
    {
        prediction(meas_package);
    }

    // Update the weights and resample
    updateWeights(meas_package);

    resample();

    Particle best_particle;
    double highest_weight = 0.0;

    for (auto &particle : m_particles)
    {
        if (particle.weight > highest_weight)
        {
            highest_weight = particle.weight;
            best_particle = particle;
        }
    }

    vector<double> error;
    return make_tuple(best_particle, error);
}

/**************************************************************************************************
 *  @brief Initializes particle filter by initializing particles to Gaussian distribution around
 *  first position.
 *************************************************************************************************/
void ParticleFilter::init(const MeasurementPackage &meas_package)
{
    default_random_engine gen;
    gen.seed(300);
    normal_distribution<double> dist_x(meas_package.m_controlInfo[0], m_sigmaGps[0]);
    normal_distribution<double> dist_y(meas_package.m_controlInfo[1], m_sigmaGps[1]);
    normal_distribution<double> dist_theta(meas_package.m_controlInfo[2], m_sigmaGps[2]);

    for (auto &particle : m_particles)
    {
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
    }
}

/**************************************************************************************************
 *  @brief Predicts the state for the next time step
 *************************************************************************************************/
void ParticleFilter::prediction(const MeasurementPackage &meas_package)
{
    const double velocity = meas_package.m_controlInfo[0];
    const double yaw_rate = meas_package.m_controlInfo[1];
    const double delta_t = meas_package.m_delta_t;

    const double vel_mult_deltat = velocity * delta_t;
    const double yaw_mult_deltat = yaw_rate * delta_t;
    const double vel_div_yaw = velocity / yaw_rate;

    // add noise
    default_random_engine gen;

    // define normal distributions for sensor noise
    normal_distribution<double> dist_x(0, m_sigmaGps[0]);
    normal_distribution<double> dist_y(0, m_sigmaGps[1]);
    normal_distribution<double> dist_theta(0, m_sigmaGps[2]);

    for (auto &particle : m_particles)
    {
        const double theta = particle.theta;

        if (fabs(yaw_rate) < m_numericalTolerance)
        {
            particle.x += vel_mult_deltat * cos(theta);
            particle.y += vel_mult_deltat * sin(theta);
        }
        else
        {
            particle.x += vel_div_yaw
                    * (sin(theta + yaw_mult_deltat) - sin(theta));
            particle.y += vel_div_yaw
                    * (cos(theta) - cos(theta + yaw_mult_deltat));
            particle.theta += yaw_rate * delta_t;
        }

        particle.x += dist_x(gen);
        particle.y += dist_y(gen);
        particle.theta += dist_theta(gen);

    }
}

/**************************************************************************************************
 *  @brief Find the predicted measurement that is closest to each observed measurement and assign
 *  the observed measurement to this particular landmark.
 *************************************************************************************************/
void ParticleFilter::dataAssociation(
        const vector<LandmarkObs> &landmarkList, vector<LandmarkObs> &observations)
{
    for (auto &obs : observations)
    {
        double min_dist = numeric_limits<double>::max();

        int map_id = -1;

        for (const auto &lm : landmarkList)
        {
            const double cur_dist = dist(obs.x, obs.y, lm.x, lm.y);
            if (cur_dist <= min_dist)
            {
                min_dist = cur_dist;
                map_id = lm.id;
            }
        }

        assert(map_id != -1);
        obs.id = map_id;
    }
}

/**************************************************************************************************
 *  @brief Transform car co-ordinate system to map co-ordinate
 *************************************************************************************************/
void ParticleFilter::transformCoordinateSystem(
        LandmarkObs &observation, const Particle &particle) const
{
    const double sin_theta = sin(particle.theta);
    const double cos_theta = cos(particle.theta);
    const double obsX = observation.x;
    const double obsY = observation.y;

    observation.x = particle.x + (obsX * cos_theta) - (obsY * sin_theta);
    observation.y = particle.y + (obsX * sin_theta) + (obsY * cos_theta);
    observation.id = -1;
}

/**************************************************************************************************
 *  @brief Find all the landmarks within the sensor range of the particle.
 *************************************************************************************************/
vector<LandmarkObs> ParticleFilter::findLandmarkInRange(const Particle &particle) const
{
    // Landmarks within the sensor range of the particle.
    vector<LandmarkObs> landmarkList;

    for (const auto &lm: m_map.landmark_list)
    {
        // Calculate the Euclidean distance between two 2D points
        const double distance = dist(particle.x, particle.y, lm.x_f, lm.y_f);

        // only consider landmarks within sensor range of the particle
        if (distance <= m_sensorRange)
        {
            landmarkList.push_back(LandmarkObs { lm.id_i, lm.x_f, lm.y_f });
        }
    }

    return landmarkList;
}

/**************************************************************************************************
 *  @brief Updates the weights for each particle based on the likelihood of the
 *  observed measurements
 *************************************************************************************************/
void ParticleFilter::calcWeight(
        const vector<LandmarkObs> &observations, Particle &particle) const
{
    particle.weight = 1.0;

    const double s_xx = pow(m_sigmaLandmark[0], 2);
    const double s_yy = pow(m_sigmaLandmark[1], 2);
    const double norm = (2 * M_PI * s_xx * s_yy);

    for (const auto &obs : observations)
    {
        double pr_x = 0, pr_y = 0;

        pr_x = m_map.landmark_list[obs.id - 1].x_f;
        pr_y = m_map.landmark_list[obs.id - 1].y_f;

        // multivariate Gaussian
        double obs_w = (1 / norm)
                * exp( -(pow(obs.x - pr_x, 2) / (2 * s_xx) + (pow(obs.y - pr_y, 2) / (2 * s_yy))));

        particle.weight *= obs_w;
    }
}

/**************************************************************************************************
 *  @brief Updates the weights for each particle based on the likelihood of the
 *  observed measurements
 *************************************************************************************************/
void ParticleFilter::updateWeights(const MeasurementPackage &meas_package)
{
    for (auto &particle : m_particles)
    {
        vector<LandmarkObs> observations = meas_package.m_noisyObservations;

        // Transform car co-ordinate system to map co-ordinate
        for (auto &obs : observations)
        {
            transformCoordinateSystem(obs, particle);
        }

        // Landmarks within the sensor range of the particle.
        vector<LandmarkObs> landmarkList = findLandmarkInRange(particle);

        dataAssociation(landmarkList, observations);

        calcWeight(observations, particle);
    }
}

/**************************************************************************************************
 *  @brief Resamples from the updated set of particles to form the new set of particles.
 *************************************************************************************************/
void ParticleFilter::resample()
{
    default_random_engine gen;
    vector<Particle> newParticles;
    vector<double> weights;

    for (const auto &particle: m_particles)
    {
        weights.push_back(particle.weight);
    }

    uniform_int_distribution<int> uniintdist(0, m_numParticles - 1);
    auto index = uniintdist(gen);
    double max_weight = *max_element(weights.begin(), weights.end());
    uniform_real_distribution<double> unirealdist(0.0, max_weight);

    double beta = 0.0;

    for (uint32_t i = 0; i < m_numParticles; ++i)
    {
        beta += unirealdist(gen) * 2.0;
        while (beta > weights[index])
        {
            beta -= weights[index];
            index = (index + 1) % m_numParticles;
        }
        newParticles.push_back(m_particles[index]);
    }

    m_particles = newParticles;
}
