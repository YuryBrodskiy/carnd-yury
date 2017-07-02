#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

//Parameters for run
const static bool DO_TUNNING = false;       //<! Switch on tuning using Twiddle one optimization step per cycle
const static double speed_setpoint = 15.0; //<! Speed of the vehicle
const static double cte_setpoint =    0.0; //<! Vehicle should be drive in center of the road
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
  return M_PI;
}
double deg2rad(double x)
{
  return x * pi() / 180;
}
double rad2deg(double x)
{
  return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
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

int main()
{
  uWS::Hub h;

  PID pid;

  if (DO_TUNNING)
  {
    pid.Init(1.5, 0.05, 0.5).InitTwiddle(0.5, 0.5, 0.5);
  }
  else
  {
    //Acquired using twiddle Kp = 1.8125  Ki = 0.175  Kd = 0.500488
    pid.Init(1.8125, 0.175, 0.5).InitTwiddle(0.0, 0.0, 0.0);
  }

  PID pid_speed;
  pid_speed.Init(0.8, 0.04, 0.02).InitTwiddle(0.0, 0.0, 0.0);

  int i = 0;
  h.onMessage([&pid, &pid_speed, &i](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
      if (length && length > 2 && data[0] == '4' && data[1] == '2')
      {
        auto s = hasData(std::string(data).substr(0, length));
        if (s != "")
        {
          auto j = json::parse(s);
          std::string event = j[0].get<std::string>();
          if (event == "telemetry")
          {
            // j[1] is the data JSON object
            const double cte = std::stod(j[1]["cte"].get<std::string>());
            const double speed = std::stod(j[1]["speed"].get<std::string>());
            //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
            /*
             * DONE: Calcuate steering value here, remember the steering value is [-1, 1].
             * Don't clip input here as it is clipped by the client
             */

            json msgJson;
            msgJson["steering_angle"] = pid.UpdateError(cte_setpoint - cte).control();
            msgJson["throttle"] = pid_speed.UpdateError(speed_setpoint - speed).control();

            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //debug
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            i++;
            if(DO_TUNNING && i>8000)
            {
              i=0;
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              pid.twiddle_step();
            }
          }
        }
        else
        {
          // Manual driving
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
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
