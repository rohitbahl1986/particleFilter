/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "bayesFilter.hpp"
#include "helper_functions.h"
#include "particle.hpp"

class ParticleFilter: public BayesFilter
{

public:

    enum FilterState
    {
        INIT,
        ACTIVE
    };

    // Constructor
    ParticleFilter();

    // Destructor
    ~ParticleFilter()
    {
    }
    virtual std::tuple<Particle, std::vector<double>> filter(
            const MeasurementPackage &meas_package);

private:

    // Number of particles to draw
    const int m_numParticles;

    // Set of current particles
    std::vector<Particle> m_particles;

    // Flag, if filter is initialized
    bool m_filterState;

    static const uint32_t m_gpsDim = 3;

    // GPS measurement uncertainty [x [m], y [m], theta [rad]]
    const double m_sigmaGps[m_gpsDim];

    // Landmark measurement uncertainty [x [m], y [m]]
    const double m_sigmaLandmark[2];

    // Sensor range [m]
    const double m_sensorRange;

    const double m_numericalTolerance;

    // map data
    Map m_map;

    /**
     * @brief Initializes particle filter by initializing particles to Gaussian
     *   distribution around first position and all the weights to 1.
     * @param[in] meas_package : An object containing control info.
     */
    void init(const MeasurementPackage &meas_package);

    /**
     * @brief Predicts the state for the next time step
     *   using the process model.
     * @param[in] meas_package : An object containing control info.
     */
    void prediction(const MeasurementPackage &meas_package);

    /**
     * @brief Updates the weights for each particle based on the likelihood of the
     * observed measurements.
     * @param[in] meas_package : An object containing control info.
     */
    void updateWeights(const MeasurementPackage &meas_package);

    /**
     * @brief Transform car co-ordinate system to map co-ordinate
     * @param[in] observation
     * @param[in] particle
     */
    void transformCoordinateSystem(LandmarkObs &observation, const Particle &particle) const;

    /**
     * @brief Find all the landmarks within the sensor range of the particle.
     * @param[in] particle
     * @return List of landmarks within the sensor range of the particle.
     */
    std::vector<LandmarkObs> findLandmarkInRange(const Particle &particle) const;

    /**
     * @brief Update the weight of the particle given the new observation.
     * @param[in] observations  observation vector
     * @param[in] particle
     */
    void calcWeight(
            const std::vector<LandmarkObs> &observations, Particle &particle) const;

    /**
     * dataAssociation Finds which observations correspond to which landmarks (likely by using
     *   a nearest-neighbors data association).
     * @param landmarkList Vector of predicted landmark observations
     * @param observations Vector of landmark observations
     */
    void dataAssociation(
            const std::vector<LandmarkObs> &landmarkList, std::vector<LandmarkObs>& observations);

    /**
     * @brief Resamples from the updated set of particles to form the new set of particles.
     */
    void resample();

    void error()
    {
        /*
         // Calculate and output the average weighted error of the particle filter over all time steps so far.
         vector<Particle> particles = pf.m_particles;
         int num_particles = particles.size();
         double highest_weight = -1.0;
         Particle best_particle;
         double weight_sum = 0.0;
         for (int i = 0; i < num_particles; ++i)
         {
         if (particles[i].weight > highest_weight)
         {
         highest_weight = particles[i].weight;
         best_particle = particles[i];
         }
         weight_sum += particles[i].weight;
         }
         cout << "highest w " << highest_weight << endl;
         cout << "average w " << weight_sum/num_particles << endl;
         */
    }
};

#endif /* PARTICLE_FILTER_H_ */
