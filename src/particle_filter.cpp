/**
 * particle_filter.cpp
 * Particle filter class in 2 dimensions
 * 
 * Created on: Feb 02/2019
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

//#include <helper_functions.h>

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

