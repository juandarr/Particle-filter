#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <json.hpp>
#include <particle_filter.h>

//Using namespace classes
using nlohmann::json;
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data then JSON object in string format will be returned,
// else the empty string "" will be returned.

string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 =  s.find_first_of("[");
    auto b2 = s.find_first_of("]");

    if (found_null != string::npos){
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2-b1+1);
    }
    return "";
}

int main() {
    uWS::Hub h ;

    // Set up parameters here
    double delta_t = 0.1; //Time elapsed between measurements [sec]
    double sensor_range = 50; // Sensor range [m]

    // GPS measurement uncertainty [x [m], y [m], theta [rad]]
    double sigma_pos[3] = {0.3, 0.3, 0.01};
    // Landmark measurement uncertainty [x [m], y[m]]
    double sigma_landmark[2] = {0.3, 0.3};

    // Read mapa data
    Map map;
    if (!read_map_data("../data/map_data.txt", map)) {
        std::cout << "Error: Could not open map file" << std::endl;
        return -1;
    }

    //Creat particle filter
    ParticleFilter pf;

    h.onMessage([&pf, &map, &delta_t, &sensor_range, &sigma_pos, &sigma_landmark]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                 uWS::OpCode opCode) {
            
        //"42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 3 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(string(data));

            if (s != ""){
                auto j = json::parse(s);

                string event =  j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    if (!pf.initialized()){
                        // Sense noisy position data from the simulator
                        double sense_x = std::stod(j[1]["sense_x"].get<string>());
                        double sense_y = std::stod(j[1]["sense_y"].get<string>());
                        double sense_theta = std::stod(j[1]["sense_theta"].get<string>());

                        pf.init(sense_x, sense_y, sense_theta, sigma_pos);
                    } else {
                        // Predict the vehicle's next state from previous (noiseless control) data
                        double previous_velocity = std::stod(j[1]["previous_velocity"].get<string>());
                        double previous_yawrate = std::stod(j[1]["previous_yawrate"].get<string>());

                        pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);
                    }

                    // Receive noisy observation data from simulator
                    // sense_observations in JSON format
                    // [{obs_x, obs_y},{obs_x, obs_y},...,{obs_x,obs_y}]
                    vector<LandmarkObs> noisy_observations;
                    string sense_observations_x = j[1]["sense_observations_x"];
                    string sense_observations_y = j[1]["sense_observations_y"];

                    vector<float> x_sense;
                    std::istringstream iss_x(sense_observations_x);

                    std::copy(std::istream_iterator<float>(iss_x),
                    std::istream_iterator<float>(),
                    std::back_inserter(x_sense)); 

                    vector<float> y_sense;
                    std::istringstream iss_y(sense_observations_y);

                    std::copy(std::istream_iterator<float>(iss_y),
                    std::istream_iterator<float>(),
                    std::back_inserter(y_sense)); 

                    for (int i = 0; i < x_sense.size(); ++i) {
                        LandmarkObs obs;
                        obs = {.x = x_sense[i], .y = y_sense[i]};

                        noisy_observations.push_back(obs);
                    }

                    // Update the weights
                    pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);



                    //TODO completion of the rest of Particle filter logic
                    
                    //end telemetry section         
                }
            } else {
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        } // end websocket message if
    }); // end h.onMessage

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                            char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listen to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}