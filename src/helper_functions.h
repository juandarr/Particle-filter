/**
 * helper_functions.h
 * A set of helper function used in the 2D particle filter class file
 * 
 * Created on: Feb 16/2019
 * Author: Juan David Rios
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <math.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "map.h"

// for portability of M_PI (V. Studio, MinGW, etc.)
#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif

/**
 * Struct representing one position/control measurement
 */
struct control_s {
    double velocity; // Velocity [m/s]
    double yawrate; // Yaw rate [rad/s]
};

/**
 * Struct representing ground truth position.
*/
struct ground_truth {
    double x; // Global vehicle x position [m]
    double y; // Global vehicle y position [m]
    double theta; // Global vehicle yaw [rad]
};

/**
 * Struct representing one landmark observation measurement
 */
struct LandmarkObs {
    int id; // Id of matching landmark in the map
    double x; // Local (vehicle coords) x position of landmark observaton [m]
    double y; // Local (vehicle coords) y position of landmark observation [m]
}; 


/**
 * Computes the euclidean distance between two 2D points
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two points
 */
inline double dist (double x1, double y1, double x2, double y2) {
    return sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
};

/**
 * Computes the error between ground truth and particle filter data.
 * @param (gt_x, gt_y, gt_theta) x, y and theta of ground truth 
 * @param (pf_x, pf_y, pf_theta) x, y and theta of particle filter
 * @output Error between ground truth and particle filter data
 */
inline double * getError (double gt_x, double gt_y, double gt_theta, double pf_x,
                            double pf_y, double pf_theta) {
    static double error[3];
    error[0] = fabs(pf_x -gt_x);
    error[1] = fabs(gt_y-pf_y);
    error[2] = fabs(gt_theta - pf_theta);
    error[2] = fmod(error[2], 2.0 * M_PI);
    if (error[2] > M_PI) {
        error[2] =  2 * M_PI - error[2];
    }
    return error;
}

/**
 * Reads map data from file
 * @param filename Name of file containing map data
 * @output True if opening and reading file was succesful
 */
inline bool read_map_data(std::string filename, Map& map) {
    // Get file of map
    std::ifstream in_file_map(filename.c_str(), std::ifstream::in);
    // Return if we can't open the file
    if (!in_file_map) {
        return false;
    }

    // Declare single line of map file
    std::string line_map;

    // Run over each single line
    while (getline(in_file_map, line_map)) {

        std::istringstream iss_map(line_map);

        // Declare landmark values and ID
        float landmark_x_f, landmark_y_f;
        int id_i;

        // Read data from current line to values
        iss_map >> landmark_x_f;
        iss_map >> landmark_y_f;  
        iss_map >> id_i;

        // Declare single landmark 
        Map::single_landmark_s single_landmark_temp;

        // Set values
        single_landmark_temp = {.id_i = id_i, .x_f = landmark_x_f, .y_f =landmark_y_f};

        // Add to landmark list of map
        map.landmark_list.push_back(single_landmark_temp);
    }
    return true;
}

/** Reads control data from file
 * @param filename Name of file containing control measurements 
 * @output True if opening and reading file was succesfull
 */
inline bool read_control_data(std::string filename, std::vector<control_s>& position_meas) {

    // Get file of position measurements 
    std::ifstream in_file_pos(filename.c_str(), std::ifstream::in);

    // Return false if we can't open the file
    if (!in_file_pos) {
        return false;
    }    

    // Declare single line of position measurements file
    std::string line_pos;

    // Run over each single file 
    while (getline(in_file_pos, line_pos)) {

        std::istringstream iss_pos(line_pos);

        // Declare control values
        double velocity, yawrate;

        // Declare single control measurement
        control_s meas;

        // Read data from line to values
        iss_pos >> velocity;
        iss_pos >> yawrate;

        // Set values
        meas = { .velocity = velocity, .yawrate = yawrate};

        // Add to list of control measurements 
        position_meas.push_back(meas);
    }

    return true;
} 

/** Reads ground truth data from file 
 * @param filename Name of file containing ground truth data
 * @output True if opening and reading file was succesful
 */
inline bool read_gt_data(std::string filename, std::vector<ground_truth>& gt) {

    // Get file of position measurements
    std::ifstream in_file_pos(filename.c_str(), std::ifstream::in);

    // Return false if we can't open file
    if (!in_file_pos) {
        return false;
    }

    // Declare single line of position measurement file
    std::string line_pos;

    // Run over each single line
    while(getline(in_file_pos, line_pos)) {

        std::istringstream iss_pos(line_pos);

        // Declare position values
        double x, y, theta;

        // Declare single ground truth
        ground_truth single_gt;

        // Read data from line to values
        iss_pos >> x;
        iss_pos >> y;
        iss_pos >> theta;

        // Set values
        single_gt = { .x= x, .y = y, .theta = theta};

        // Add to list of ground truths
        gt.push_back(single_gt);
    }
    
    return true;
} 

/**
 * Reads landmark observation data from file.
 * @param filename Name of file contraining landmark observation data
 * @output True if opening and reading file was succesful
 */
inline bool read_landmark_data(std::string filename, std::vector<LandmarkObs>& observations) {

    // Get file of landmark measurements
    std::ifstream in_file_obs(filename, std::ifstream::in);

    // Return false if we can't open the file
    if (!in_file_obs) {
        return false;
    }

    // Declare single line of landmark measurement file
    std::string line_obs;

    // Run over each single line
    while (getline(in_file_obs, line_obs)) {

        std::istringstream iss_obs(line_obs);

        // Declare position values
        double local_x, local_y;

        // Read data from line to values
        iss_obs >> local_x;
        iss_obs >> local_y;

        // Declare single landmark measurement
        LandmarkObs meas;

        // Set values
        meas.x = local_x;
        meas.y = local_y;

        // Add to list of landmark observations
        observations.push_back(meas);
    }

    return true;
} 

/**
 * Calculates the multivariate gaussian probability given the standard deviation, 
 * landmark location and observation in x/y dimensions
 */  
inline double multi_gaussian(double std_x, double std_y, double ux, double uy, double x, double y) {
    // Define normalization factor in multivariate gaussian distribution
    double normalization;
    normalization = 1/(2 * M_PI * std_x * std_y);

    // Define exponent of multivariate gaussian distribution
    double exponent;
    exponent = (pow(x-ux, 2)/(2*pow(std_x,2))+pow(y-uy, 2)/(2*pow(std_y,2)));

    // Define probability given the normalization factor and the exponent
    double weight;
    weight = normalization * exp(-exponent);

    return weight;
}

#endif // HELPER_FUNCTIONS_H_