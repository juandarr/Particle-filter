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

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> observations) {
    /**
     * Find the predicted measurement that is closest to each observed measurement
     *  and assign the observed measurement to this particular landmark.
     */

    
    // Transform each observation x,y coordinates from car frame of reference to map
    for (int i = 0; i < particles.size(); ++i) {
        
        // Define x and y arrays for observations transformed to map coordinate system
        std::vector<double> sense_x;
        std::vector<double> sense_y;
        // Define each coordinate in map coordinate system
        double xm;
        double ym;
        // Define array of associations
        std::vector<int> associations;
        
        for (int j = 0; j < observations.size(); ++j) {
            // Observation x from car to map coordinate system
            xm = particles[i].x + (std::cos(particles[i].theta) * observations[j].x) - (std::sin(particles[i].theta) * observations[j].y);
            sense_x.push_back(xm);
            // Observation y from car to map coordinate system
            ym = particles[i].y + (std::sin(particles[i].theta) * observations[j].x) + (std::cos(particles[i].theta) * observations[j].y);
            sense_y.push_back(ym);

            // Define landmark association for a given xm,ym observation
            int chosen_landmark;
            double current_distance = 1000.0;
            double distance;
            
            for (int z = 0; z < predicted.size(); ++z) {
                distance = dist(xm,ym, predicted[z].x, predicted[z].y); 
                if ( distance < current_distance) {
                    chosen_landmark = predicted[z].id;
                    current_distance = distance; 
                }
            }

            associations.push_back(chosen_landmark);
        }

        SetAssociations(particles[i], associations, sense_x, sense_y);
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
