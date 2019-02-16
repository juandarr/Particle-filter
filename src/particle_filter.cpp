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

    num_particles = 1000;

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

