
#include <math.h>
#include <uWS/uWS.h>


#include <thread>

#include <random>
#include "Eigen-3.3/Eigen/Core"

#include "json.hpp"

#include "utils.h"

#include "Road.h"
#include "EgoVehicle.h"

using namespace std;
using namespace utils;
// for convenience
using json = nlohmann::json;




int main() {

  uWS::Hub h;

  // Load up map values
  Road road(string("../data/highway_map.csv"));
  EgoVehicle myCar(road);

  h.onMessage([&road, &myCar](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();
        //std::cout << "Start cycle"<< std::endl;
//        std::cout << j << endl;
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_yaw = j[1]["yaw"];

          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          road.update(sensor_fusion);

          myCar.update(toWaypoints(previous_path_x, previous_path_y), car_x, car_y, car_yaw,car_s,car_d);

          vector<vector<double> > xy = myCar.getWaypoints();

          //END TODO
          msgJson["next_x"] = xy[0];
          msgJson["next_y"] = xy[1];
//          std::cout << msgJson << endl;
          auto msg = "42[\"control\"," + msgJson.dump() + "]";

        //  this_thread::sleep_for(chrono::milliseconds(7000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}



