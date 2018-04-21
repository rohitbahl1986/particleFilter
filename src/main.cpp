/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include <tuple>
#include <vector>

#include "particleFilter.hpp"
#include "particle.hpp"
#include "processData.hpp"
#include "measurement_package.hpp"
#include "helper_functions.h"

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using namespace std;
using json = nlohmann::json;

/**************************************************************************************************
 *  @brief This function checks if the SocketIO event has JSON data. If there is data the JSON
 *  object in string format will be returned, else the empty string "" will be returned.
 *************************************************************************************************/
std::string hasData(std::string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("]");
    if (found_null != std::string::npos)
    {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos)
    {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

/**************************************************************************************************
 *  @brief Main application which connects to the simulator and feeds data to the processing engine.
 *************************************************************************************************/
int main()
{
    uWS::Hub h;

    ProcessData pd;
    bool isFirstMeasurement = true;

    h.onMessage(
            [&pd, &isFirstMeasurement](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
            {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event

                if (length && length > 2 && data[0] == '4' && data[1] == '2')
                {

                    auto s = hasData(std::string(data));
                    if (s != "")
                    {

                        auto j = json::parse(s);
                        std::string event = j[0].get<std::string>();

                        if (event == "telemetry")
                        {
                            MeasurementPackage measPack;

                            // j[1] is the data JSON object

                            if (isFirstMeasurement)
                            {
                                // Sense noisy position data from the simulator
                                double sense_x = std::stod(j[1]["sense_x"].get<std::string>());
                                double sense_y = std::stod(j[1]["sense_y"].get<std::string>());
                                double sense_theta = std::stod(j[1]["sense_theta"].get<std::string>());
                                measPack.m_controlInfo.push_back(sense_x);
                                measPack.m_controlInfo.push_back(sense_y);
                                measPack.m_controlInfo.push_back(sense_theta);

                                isFirstMeasurement = false;
                            }
                            else
                            {
                                // Provide the control info to predict the next state.
                                double previous_velocity = std::stod(j[1]["previous_velocity"].get<std::string>());
                                double previous_yawrate = std::stod(j[1]["previous_yawrate"].get<std::string>());
                                measPack.m_controlInfo.push_back(previous_velocity);
                                measPack.m_controlInfo.push_back(previous_yawrate);
                            }

                            // receive noisy observation data from the simulator
                            // sense_observations in JSON format [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
                            string sense_observations_x = j[1]["sense_observations_x"];
                            string sense_observations_y = j[1]["sense_observations_y"];

                            std::vector<float> x_sense;
                            std::istringstream iss_x(sense_observations_x);

                            std::copy(std::istream_iterator<float>(iss_x),
                                    std::istream_iterator<float>(),
                                    std::back_inserter(x_sense));

                            std::vector<float> y_sense;
                            std::istringstream iss_y(sense_observations_y);

                            std::copy(std::istream_iterator<float>(iss_y),
                                    std::istream_iterator<float>(),
                                    std::back_inserter(y_sense));

                            for(int i = 0; i < x_sense.size(); i++)
                            {
                                LandmarkObs obs;
                                obs.x = x_sense[i];
                                obs.y = y_sense[i];
                                measPack.m_noisyObservations.push_back(obs);
                            }

                            // Start data processing
                            tuple<Particle, vector<double>> tp = pd.estimatePosition(measPack);
                            Particle best_particle = get<0>(tp);
                            vector<double> error = get<1>(tp);

                            json msgJson;
                            msgJson["best_particle_x"] = best_particle.x;
                            msgJson["best_particle_y"] = best_particle.y;
                            msgJson["best_particle_theta"] = best_particle.theta;

                            //Optional message data used for debugging particle's sensing and associations
                            msgJson["best_particle_associations"] = best_particle.getAssociations();
                            msgJson["best_particle_sense_x"] = best_particle.getSenseX();
                            msgJson["best_particle_sense_y"] = best_particle.getSenseY();

                            auto msg = "42[\"best_particle\"," + msgJson.dump() + "]";
                            // std::cout << msg << std::endl;
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                        }
                    }
                    else
                    {
                        std::string msg = "42[\"manual\",{}]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }

            });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
    {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}

