#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"
#include "cViz.h"

#include <fstream>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

// for convenience
using nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph2ms(double x) { return x * 0.447; }
double ms2mph(double x) { return x / 0.447; }

// Global varibles for the moving average
double prev_t = Now();
std::deque<double> dist_que;
std::deque<double> dt_que;
double prev_x = 0, prev_y = 0, prev_v = 0;
int max_iters = 300;

double MA_acc(const double x, const double y, const double dt) {
  double dist = Distance(x, y, prev_x, prev_y);
  prev_x = x;
  prev_y = y;

  dist_que.push_back(dist);
  dt_que.push_back(dt);
  double acc = 0;
  if (dist_que.size() > 5) { // 5 steps moving average
    // Pop the old data first to avoid the wrong initial value
    dist_que.pop_front();
    dt_que.pop_front();
    double v = 0;
    double dist = 0, dt = 0;
    for (size_t i = 0; i < dist_que.size(); i++) {
      dist += dist_que[i];
      dt += dt_que[i];
    }
    v = dist / dt;
    acc = (v - prev_v) / dt;
    prev_v = v;
  }
  return acc;
}


int main() {
  uWS::Hub h;
  // Visualization
  DrawThePath();
  int iters = 0;
  std::vector<double> x_vals = {};
  std::vector<double> y_vals = {};

  /*std::vector<double> psi_vals = {};
  std::vector<double> v_vals = {};
  std::vector<double> cte_vals = {};
  std::vector<double> epsi_vals = {};
  std::vector<double> delta_vals = {};
  std::vector<double> a_vals = {};
  */

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc, &x_vals, &y_vals, &iters](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"]; //cout<< "current psi: " <<psi<<endl;
          double v = mph2ms(j[1]["speed"]); //cout<< "current m/s: " << v<<endl;
          double dt = Now() - prev_t;
          prev_t = Now();

          // get the control feedback
          double delta = j[1]["steering_angle"]; //cout<< "delta reading: " << delta<<endl;
          double a = MA_acc(px, py, dt);




          //Plotting
      	  if(true){
      	    //If set, save data for plotting
      	    iters++; cout<<iters<<endl;
      	    x_vals.push_back(px);
      	    y_vals.push_back(py);
      	    /*psi_vals.push_back(psi);
      	    v_vals.push_back(v);
      	    cte_vals.push_back(cte);
      	    epsi_vals.push_back(epsi);
      	    delta_vals.push_back(delta);
      	    a_vals.push_back(a);*/
      	  }

          assert(ptsx.size() == ptsy.size()); // error check
          Eigen::VectorXd xvals(ptsx.size());
          Eigen::VectorXd yvals(ptsy.size());

          // Convert back to vehicule/body coordinates
          for (size_t i = 0; i < ptsx.size(); i++) {
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;

            xvals[i] = x * cos(-psi) - y * sin(-psi);
            yvals[i] = x * sin(-psi) + y * cos(-psi);
          }

          // a spline could be useful for path generation,
          // but since the path is given already, we can directly use a polyfit
          // create the polyfit. and the coefficients are enough to calculate for the path plan

          auto coeffs = polyfit(xvals, yvals, 3);


          // since the coordinate is transfered to local coord
          // current local position will then be
          px = 0;
          py = 0;
          psi = 0;

          double cte = polyeval(coeffs, px) - py; // cout<<"first cte: "<<cte<<endl;
          double epsi = psi - atan(coeffs[1]); // since x is 0

          // If the operation latency is considered
          double Lf = 2.67;
          double delay = 0.1;

          px = px + v * cos(psi) * delay;
          py = py + v * sin(psi) * delay;
          psi = psi - v * delta / Lf * delay; // psi in rad and delta in rad
          v += a * delay;

          cte +=   v * sin(epsi) * delay;
          epsi += - v * delta / Lf * delay;


          // Now update the MPC with the info we got
          Eigen::VectorXd state(8);
          state << px, py, psi, v, cte, epsi,delta,a;
          mpc.VelTune(xvals, yvals);
          auto vars = mpc.Solve(state, coeffs);

          //cout<< "The ouput of MPC: " << vars[0] <<endl;
          json msgJson;

          // delta given is in radius in the range of (-0.4~0.4), in the MPC, we constraint the delta output in the
          // range of (-0.4~0.4) rad about(-25 deg ~ 25 deg) as well
          // plus the output only receive a cmd [-1,1] which represent [-25deg 25deg]
          msgJson["steering_angle"] = rad2deg(vars[0])/25.0;
          msgJson["throttle"] = vars[1];

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (int i = 2; i < vars.size(); i+=2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+1]);
          }


          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line, fill it with 25 seperate points
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double space = 2.5;
          int num_points = 25;
          for (int i = 1; i < num_points; i++) {
            next_x_vals.push_back(space * i);
            next_y_vals.push_back(polyeval(coeffs, space * i));
          }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          if(iters > max_iters){
            plt::named_plot("Position ",x_vals, y_vals,"r--");
            plt::legend();
            plt::show();
            exit(1);
          }
          /*if(iters > max_iters){
         	    //Plot Graph for analysis of the first 100 iterations
         	    plt::subplot(3, 1, 1);
         	    plt::title("CTE");
         	    plt::plot(cte_vals);
         	    plt::subplot(3, 1, 2);
         	    plt::title("Delta (Radians)");
         	    plt::plot(delta_vals);
         	    plt::subplot(3, 1, 3);
         	    plt::title("Velocity");
         	    plt::plot(v_vals);
         	    plt::show();
         	    iters = 0;
         	    exit(1);
         	  }*/
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
