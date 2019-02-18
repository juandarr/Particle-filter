/**
 * Particle_filter.h
 * Particle filter class in 2 dimensions
 * 
 * Created on: Feb 15/2019
 * Author: Juan David Rios
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H

#include <string>
#include <vector>
#include "helper_functions.h"

struct Particle {
    int id;
    double x;
    double y;
    double theta;
    double weight;
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;
};

class ParticleFilter {
    public:
        // Constructor
        // @param num_particles Number of particles
        ParticleFilter(): num_particles(0), is_initialized(false) {}

        // Destructor
        ~ParticleFilter() {}

        /**
         * init Initializes particle filter by initializing particles to Gaussian 
         * distribution around first position and all the weights to 1.
         * @param x Initial x position [m] (simulated estimate from GPS)
         * @param y Initial y position[m]
         * @param theta Initial orientation [rad]
         * @param std[] Array of dimension 3 [standard devitation of x [m],
         * standard deviation y [m], standard deviation of yaw [rad]]
         */
        void init(double x, double y, double theta, double std[]);

        /**
         * prediction Predicts the state from the next time step
         * using the process model.
         * @param delta_t Time between time step t and t+1 in measurements [s]
         * @param std_pos[] Array of dimension 3 [standard deviation of x [m],
         *  standard deviation of y [m], standard deviation of yaw [rad]]
         * @param velocity Velocity of car from t to t+1 [m/s]
         * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
         */
        void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);

        /**
         * dataAssociation Finds which observations correspond to which landmarks
         *  (likely by using a nearest-neighbors data association).
         * @param predicted Vector of predicted landmark observations
         * @param observations Vector of landmark observations
         */
        void dataAssociation(Particle &particle, std::vector<LandmarkObs> predicted,
                              std::vector<LandmarkObs>& observations);

        /**
         * updateWeights Updates the weights for each particle based on the likelihood 
         *  of the observed measurements.
         * @param sensor_range Range [m] of sensors
         * @param std_landmark[] Array of dimension 2
         *   [Landmark measurement uncertainty [x [m], y [m]]
         * @param observations Vector of landmark observations
         * @param map Map class containing map landmarks
         */
        void updateWeights(double sensor_range, double std_landmark[],
                            const std::vector<LandmarkObs>& observations,
                            const Map &map_landmarks);

        /**
         * resample Resamples from the updated set of particles to form 
         *  the new set of particles
         */
        void resample();

        /**
         * initialized Returns whether particle filter is initialized yet or not
         */
        const bool initialized() {
            return is_initialized;
        }

        /**
         * Set a particles list of associations, along with the associations
         *  calculated world x,y coordinates
         * This can be a very useful debugging tool to make sure transformations
         *  are correct and associations correctly connected
         */
        void SetAssociations(Particle& particle, const std::vector<int>& associations,
                             const std::vector<double>& sense_x,
                             const std::vector<double>& sense_y);

        /**
         * Usef for obtaining debugging information related to particles.
         */
        std::string getAssociations(Particle best);
        std::string getSenseCoord(Particle best, std::string coord);

        // Set of current particles
        std::vector<Particle> particles;

    private:
        // Number of particles to draw
        int num_particles;

        // Flag, true if filter is initialized, false otherwise
        bool is_initialized;

        // Vector of weights of all particles
        std::vector<double> weights;
};

#endif // PARTICLE_FILTER_H_