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
#include <iomanip>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

//for convenience
using std::string;
using std::vector;
using std::normal_distribution;

// Defines default random engine variable to sample gaussian distributions
std::default_random_engine gen;

void ParticleFilter:: init (double x, double y, double theta, double std[]) {

    num_particles = 150;

    //Normal gaussian distribution for x, y and theta
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    Particle p;

    for (int i = 0; i < num_particles; ++i) {
        // Initialize particle using gaussian distributions for coordinates x,y, angle theta and weight=1
        p = {.id=i, .x=dist_x(gen), .y=dist_y(gen), .theta=dist_theta(gen), .weight= 1.0};
        // Add particle to the set of particles
        particles.push_back(p);
        
        std::cout << "Particle: id " << p.id <<" x: " <<
            p.x << " y: "<< p.y << " theta: " << p.theta <<
            " weight: "<< p.weight << std::endl;
    }

    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {

    //std::cout << "Here is velocity and yaw rate: " << velocity << " , " << yaw_rate << std::endl; 
    for (unsigned int i = 0; i < particles.size(); ++i) {
        // Stores particle i in variable
        struct Particle p = particles[i];

        // Predicts new values for x,y,z based on the bicycle motion model 
        // using velocity and yaw rate values
        if (yaw_rate != 0.0) {
            p.x += (velocity/yaw_rate)*(std::sin(p.theta+yaw_rate*delta_t) - std::sin(p.theta));
            p.y += (velocity/yaw_rate)*(std::cos(p.theta)-std::cos(p.theta+yaw_rate*delta_t));
        } else {
            p.x += velocity*delta_t*std::cos(p.theta);
            p.y += velocity*delta_t*std::sin(p.theta);
        }
        
        p.theta += yaw_rate*delta_t;

        //Add noise around each predicted value using the standard deviation vector
        //Normal gaussian distribution for x, y and theta
        normal_distribution<double> dist_x(p.x, std_pos[0]);
        normal_distribution<double> dist_y(p.y, std_pos[1]);
        normal_distribution<double> dist_theta(p.theta, std_pos[2]);

        // Updates x,y and theta values with the new predictions
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
      
    }
    
}

void ParticleFilter::dataAssociation(Particle &particle, vector<LandmarkObs> predicted,
                                      vector<LandmarkObs>& observations) {
    /**
     * Find the predicted measurement that is closest to each observed measurement
     *  and assign the observed measurement to this particular landmark.
     */

    // Define x and y arrays for observations transformed to map coordinate system
    std::vector<double> sense_x;
    std::vector<double> sense_y;
    
    // Define array of associations
    std::vector<int> associations;
    
    for (unsigned int i = 0; i < observations.size(); ++i) {
        // Observation x from car to map coordinate system
        sense_x.push_back(observations[i].x);
        // Observation y from car to map coordinate system
        sense_y.push_back(observations[i].y);

        // Define landmark association for a given xm,ym observation
        int chosen_landmark;
        double current_distance = 1000000000.0;
        double distance;
        bool stored = false;

        for (unsigned int j = 0; j < predicted.size(); ++j) {
            distance = dist(observations[i].x,observations[i].y, predicted[j].x, predicted[j].y); 
            if ( distance < current_distance) {
                chosen_landmark = predicted[j].id;
                current_distance = distance; 
                stored = true;
            }
        }
        if (stored == false ) std::cout << " No landmark was defined in the loop. Predicted size: " << predicted.size() << std::endl;
        if (chosen_landmark <= 0 || chosen_landmark > 42) std::cout << "Algorithm is storing wrong landmark data: " << chosen_landmark << std::endl;

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
    for (unsigned int i = 0; i < particles.size(); ++i) {

        // Define each coordinate in map coordinate system
        double xm;
        double ym;

        // Define array of observations from particle in map coordinate system
        std::vector<LandmarkObs> observations_map;
        LandmarkObs observation;

        for (unsigned int j = 0; j < observations.size(); ++j) {

            // Observation x from car to map coordinate system
            xm = particles[i].x + (std::cos(particles[i].theta) * observations[j].x) - (std::sin(particles[i].theta) * observations[j].y);
            // Observation y from car to map coordinate system
            
            ym = particles[i].y + (std::sin(particles[i].theta) * observations[j].x) + (std::cos(particles[i].theta) * observations[j].y);
            
            observation = {.id = 0, . x = xm, .y = ym};
            observations_map.push_back(observation);
        }

        // Define array of observations from particle in map coordinate system
        std::vector<LandmarkObs> predicted;
        LandmarkObs predicted_lm;
        // Define distance between each particle and map landmark to store only landmarks within sensor range
        double distance;

        bool distance_exist = false;
        for (unsigned int z = 0; z < map_landmarks.landmark_list.size(); ++z) {

            distance = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[z].x_f, map_landmarks.landmark_list[z].y_f); 
            if ( distance <= sensor_range ) {
                distance_exist = true;
                //std::cout << "Values below the sensor range!" << std::endl;
                predicted_lm = {.id =map_landmarks.landmark_list[z].id_i, .x= map_landmarks.landmark_list[z].x_f, .y = map_landmarks.landmark_list[z].y_f };
                predicted.push_back(predicted_lm);
            }
        }

        if (!distance_exist) std::cout << "No distance less than range found for particle: " << i << ", x: " <<particles[i].x << " y: " <<particles[i].y << std::endl;
        
    
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

    
    for (unsigned int i = 0; i < particles.size(); ++i) {

        new_weight = 1.0;
        for (unsigned int j = 0; j < particles[i].associations.size(); ++j) {
            mean_x = map_landmarks.landmark_list[particles[i].associations[j]-1].x_f;
            mean_y = map_landmarks.landmark_list[particles[i].associations[j]-1].y_f;    
            x_obs = particles[i].sense_x[j];
            y_obs = particles[i].sense_y[j];
        
            new_weight *= multi_gaussian(std_landmark[0], std_landmark[1], mean_x, mean_y, x_obs, y_obs);
        }

        particles[i].weight = new_weight;
        total_w += new_weight;
    
    }
   
}

void ParticleFilter::resample() {
    /**
     * Resample particles with replacement with probability 
     * proportional to their weight
     */
    vector<Particle> new_particles;

    std::uniform_int_distribution<int> distribution_int(0,particles.size()-1);

    unsigned int index = distribution_int(gen) ;
    double beta = 0.0;

    double max_weight = -1.0;
    for (unsigned int i = 0; i < particles.size(); ++i) {
        if (particles[i].weight > max_weight) {
            max_weight = particles[i].weight;
        }
    }
    
    std::uniform_real_distribution<double> distribution_real(0.0,2.0*max_weight);
    for (unsigned int i = 0; i < particles.size(); ++i) {
        beta += distribution_real(gen);

        while (particles[index].weight < beta) {
            beta -= particles[index].weight;
            index += 1;
            if (index == particles.size()) index = 0;
        }

        new_particles.push_back(particles[index]);
    }

    particles = new_particles;
    
}

void ParticleFilter::SetAssociations(Particle &particle, 
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

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1); // Get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
    vector<double> v;

    if (coord == "X") {
        v = best.sense_x;
    } else {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1); // Get rid of the trailing space
    return s;
}