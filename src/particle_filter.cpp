/**
 * particle_filter.cpp
 * Particle filter class in 2 dimensions
 * 
 * Created on: Feb 15/2019
 * Author: Juan David Rios
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <helper_functions.h>

//for convenience
using std::string;
using std::vector;
using std::normal_distribution;

// Defines default random engine variable to sample gaussian distributions
std::default_random_engine gen;

void ParticleFilter:: init (double x, double y, double theta, double std[]) {

    num_particles = 100;

    //Normal gaussian distribution for x, y and theta
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    for (int i = 0; i < num_particles; ++i) {
        // Initialize particle using gaussian distributions for coordinates x,y, angle theta and weight=1
        struct Particle p = {.id=i, .x=dist_x(gen), .y=dist_y(gen), .theta=dist_theta(gen), .weight= 1};
        // Add particle to the set of particles
        particles.push_back(p);
    }

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
    // Define variables to store predicted x, y and theta values
    double value_x;
    double value_y;
    double value_theta;

    for (int i = 0; i <particles.size(); ++i) {
        // Stores particle i in variable
        struct Particle p = particles[i];

        // Predicts new values for x,y,z based on the bicycle motion model 
        // using velocity and yaw rate values
        value_x = p.x + (velocity/yaw_rate)*(std::sin(p.theta+yaw_rate*delta_t) -std::sin(p.theta));
        value_y = p.y +(velocity/yaw_rate)*(std::cos(p.theta)-std::cos(p.theta+yaw_rate*delta_t));
        value_theta = p.theta + yaw_rate*delta_t;

        //Add noise around each predicted value using the standard deviation vector
        //Normal gaussian distribution for x, y and theta
        normal_distribution<double> dist_x(value_x, std_pos[0]);
        normal_distribution<double> dist_y(value_y, std_pos[1]);
        normal_distribution<double> dist_theta(value_theta, std_pos[2]);

        // Updates x,y and theta values with the new predictions
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
    }
    
}

void ParticleFilter::dataAssociation(Particle& particle, vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> observations) {
    /**
     * Find the predicted measurement that is closest to each observed measurement
     *  and assign the observed measurement to this particular landmark.
     */

    // Define x and y arrays for observations transformed to map coordinate system
    std::vector<double> sense_x;
    std::vector<double> sense_y;
    
    // Define array of associations
    std::vector<int> associations;
    
    for (int i = 0; i < observations.size(); ++i) {
        // Observation x from car to map coordinate system
        sense_x.push_back(observations[i].x);
        // Observation y from car to map coordinate system
        sense_y.push_back(observations[i].y);

        // Define landmark association for a given xm,ym observation
        int chosen_landmark;
        double current_distance = 1000.0;
        double distance;
        
        for (int j = 0; j < predicted.size(); ++j) {
            distance = dist(observations[i].x,observations[i].y, predicted[j].x, predicted[j].y); 
            if ( distance < current_distance) {
                chosen_landmark = predicted[j].id;
                current_distance = distance; 
            }
        }

        associations.push_back(chosen_landmark);
    }

    SetAssociations(particle, associations, sense_x, sense_y);
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
    /**
     * Update the weights of each particle using a multivariate Gaussian
     * distribution.
     */
    
    // 1. Association of each observation point to the nearest predicted landmark

    // Transform each observation x,y coordinates from car frame of reference to map
    for (int i = 0; i < particles.size(); ++i) {

        // Define each coordinate in map coordinate system
        double xm;
        double ym;

        // Define array of observations from particle in map coordinate system
        std::vector<LandmarkObs> observations_map;
        LandmarkObs observation;

        for (int j = 0; j < observations.size(); ++j) {

            // Observation x from car to map coordinate system
            xm = particles[i].x + (std::cos(particles[i].theta) * observations[j].x) - (std::sin(particles[i].theta) * observations[j].y);
            // Observation y from car to map coordinate system
            ym = particles[i].y + (std::sin(particles[i].theta) * observations[j].x) + (std::cos(particles[i].theta) * observations[j].y);
            
            observation = {.x = xm, .y=ym};
            
            observations_map.push_back(observation);
        }

        // Define array of observations from particle in map coordinate system
        std::vector<LandmarkObs> predicted;
        LandmarkObs predicted_lm;
        // Define distance between each particle and map landmark to store only landmarks within sensor range
        double distance;

        for (int z = 0; z < map_landmarks.landmark_list.size(); ++z) {

            distance = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[z].x_f, map_landmarks.landmark_list[z].y_f); 
            if ( distance <= sensor_range ) {
                predicted_lm = {.id =map_landmarks.landmark_list[z].id_i, .x= map_landmarks.landmark_list[z].x_f, .y = map_landmarks.landmark_list[z].y_f };
                predicted.push_back(predicted_lm);
            }
        }

        dataAssociation(particles[i] , predicted, observations_map); 

    }

    // 2. Calculate new weights based on the associations and how 
    // likely is that configuration to be the real agent location by the product 
    // of the probabilities

     // Total weight sum for all particles
    double total_w = 0.0;

    double new_weight;
    double mean_x;
    double mean_y;
    double x_obs;
    double y_obs;

   
    for (int i = 0; i < particles.size(); ++i) {
        
        new_weight = 1.0;
        for (int j = 0; j < particles[i].associations.size(); ++j) {
            mean_x = map_landmarks.landmark_list[particles[i].associations[j]].x_f;
            mean_y = map_landmarks.landmark_list[particles[i].associations[j]].y_f;
            x_obs = particles[i].sense_x[j];
            y_obs = particles[i].sense_y[j];
            new_weight *= multi_gaussian(std_landmark[0], std_landmark[1], mean_x, mean_y, x_obs, y_obs);
        }

        particles[i].weight = new_weight;
        total_w += new_weight;
    }

    for (int i = 0; i < particles.size(); ++i) {
        particles[i].weight /= total_w;
    }
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
    // @param particle The particle to which assign each list association
    //      and associations (x,y) in map coordinate system
    // @param sense_x The associations x mapping already converted to map coordinates
    // @param sense_y The associations y mapping already converted to map coordinates
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}
