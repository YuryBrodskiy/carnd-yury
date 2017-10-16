//
// Created by Yury Brodskiy on 10/5/17.
//

#include "Road.h"
#include <iostream>
#include <sstream>
#include "spline.h"
using namespace std;
using namespace utils;

Road::Road(std::string road_waipoint_file):
    prev_wp_(0)
{
  ifstream in_map_(road_waipoint_file.c_str(), ifstream::in);

  vector<Waypoint> map_waypoints_orig;
  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    Waypoint waypoint;
    double d_x;
    double d_y;
    iss >> waypoint.x;
    iss >> waypoint.y;
    iss >> waypoint.s;
    iss >> d_x;
    iss >> d_y;
    waypoint.d = 0;
    map_waypoints_orig.push_back(waypoint);
  }
  assert(!map_waypoints_orig.empty());

  map_waypoints_ = getHighDefinitionMap(map_waypoints_orig);

  // debug record of th high definition map
#if 0
  ofstream out_map;
  out_map.open("../data/hd_temp.csv");
  for(Waypoints::const_iterator it =hd_map_points_.begin();it!=hd_map_points_.end();it++) {
    out_map << it->x << " " << it->y << " " << it->s << endl;
  }
  out_map.close();
#endif

}

utils::Waypoint Road::getXY(double s, double d)  {
  s = range_s(s);
  while (
      !(map_waypoints_[prev_wp_ ].s <= s  &&
          (prev_wp_ == (map_waypoints_.size()-1)  || s <= map_waypoints_[prev_wp_+1].s) ))
         {
           if (map_waypoints_[prev_wp_ ].s > s)
           {
             prev_wp_--;
           } else
           {
             prev_wp_++;
           }

  }
  size_t wp2 = (prev_wp_ + 1) % map_waypoints_.size();
  double heading = atan2((map_waypoints_[wp2].y - map_waypoints_[prev_wp_].y),
                         (map_waypoints_[wp2].x - map_waypoints_[prev_wp_].x));
  // the x,y,s along the segment
  double seg_s = (s - map_waypoints_[prev_wp_].s);
  double seg_x = map_waypoints_[prev_wp_].x + seg_s * cos(heading);
  double seg_y = map_waypoints_[prev_wp_].y + seg_s * sin(heading);
  double perp_heading = heading - pi() / 2;
  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y, s, d};
}

std::vector<double> Road::getFrenet(double x, double y, double theta) {
  int next_wp = NextWaypoint(x, y, theta, map_waypoints_);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = map_waypoints_.size() - 1;
  }

  double n_x = map_waypoints_[next_wp].x - map_waypoints_[prev_wp].x;
  double n_y = map_waypoints_[next_wp].y - map_waypoints_[prev_wp].y;
  double x_x = x - map_waypoints_[prev_wp].x;
  double x_y = y - map_waypoints_[prev_wp].y;

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - map_waypoints_[prev_wp].x;
  double center_y = 2000 - map_waypoints_[prev_wp].y;
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(map_waypoints_[i].x, map_waypoints_[i].y, map_waypoints_[i + 1].x, map_waypoints_[i + 1].y);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

void Road::update(const vector<vector<double> > &sensor_fusion) {
  vehicles_.clear();
  for (vector<double> item: sensor_fusion) {
    int id = static_cast<int>(item[0]);
    if(item[6]>0)  // check if car is on our side of the road
      vehicles_[id].update(item);
  }
}


double Road::range_s(double s) const
{
  if (s < max_s & s > 0)
    return s;
  else
   return fmod(s + max_s, max_s);
}

Road::Vehicle Road::getVehicleToFollow(int line, double s, double t) {

  double min_distance = 9999;// infinity
  Road::Vehicle vehicleToFollow;
  vehicleToFollow.id_  = -1;
  for (auto item: vehicles_) {
    FrenetState state = item.second.state_in(t);
    if (line == getLine(state[3]))
    {
      double  curr_distance = range_s(range_s(state[0])-range_s(s));
      if(curr_distance > item.second.vehicleSize[0] && min_distance > curr_distance )
      {
        min_distance = curr_distance;
        vehicleToFollow = item.second;
      }
    }
  }

  return vehicleToFollow;
}

utils::Waypoints Road::getHighDefinitionMap(const utils::Waypoints &map_points_orig) {

  vector<Waypoint> map_points_highdef;
  for (size_t m_indx = 0; m_indx < map_points_orig.size() - 1; m_indx++) {
    ProjectToLocal projector(map_points_orig[m_indx].x, map_points_orig[m_indx].y,
                             map_points_orig[m_indx + 1].x, map_points_orig[m_indx + 1].y);
    int point_to_add = m_indx - 2;
    if (point_to_add < 0)
      point_to_add += map_points_orig.size() - 1;
    Waypoints map_points;
    while (map_points.size() < 5) {
      point_to_add = (point_to_add + 1) % map_points_orig.size();
      map_points.push_back(map_points_orig[point_to_add]);
    }
    Waypoints map_points_car(map_points.size());
    transform(map_points.begin(), map_points.end(), map_points_car.begin(), projector);
    sort(map_points_car.begin(), map_points_car.end(),
         [](const Waypoint &a, const Waypoint &b) {
           return a.x < b.x;
         });

    vector<vector<double> > xy_car = toVector(map_points_car);
    tk::spline path;
    path.set_points(xy_car[0], xy_car[1]);

    Waypoint start = projector(map_points_orig[m_indx]);
    Waypoint end = projector(map_points_orig[m_indx + 1]);

    for (double x = start.x; x <= end.x; x += 0.05/*[m]*/ ) {
      double y = path(x);
      double ratio = distance(x, y, start.x, start.y) / distance(end.x, end.y, start.x, start.y);
      if (end.s < 0.001) {
        end.s = max_s;
      }
      double s = start.s * (1.0 - ratio) + end.s * ratio;

      Waypoint w = {x, y, s, 0};

      map_points_highdef.push_back(projector.inverse()(w));
    }
  }

  return map_points_highdef;

}

void Road::Vehicle::update(std::vector<double> sensor_fusion_item) {
  /**
 *  car's unique ID,
 *  car's x position in map coordinates,
 *  car's y position in map coordinates,
 *  car's x velocity in m/s,
 *  car's y velocity in m/s,
 *  car's s position in frenet coordinates,
 *  car's d position in frenet coordinates.
 */
  id_ = sensor_fusion_item[0];
  //const double x = sensor_fusion_item[1];
  //const double y = sensor_fusion_item[2];
  const double dx = sensor_fusion_item[3];
  const double dy = sensor_fusion_item[4];
  const double s = sensor_fusion_item[5];
  const double d = sensor_fusion_item[6];

  // naive const speed assupmtion nice dynamic filter would do much better
  const double ds = sqrt(dx * dx + dy * dy);

  state_[0] = s;
  state_[1] = ds;
  state_[2] = 0;
  state_[3] = d;
  state_[4] = 0;
  state_[5] = 0;


}

FrenetState Road::Vehicle::state_in(double t) {

  FrenetState new_state;
  new_state[0] = state_[0] + (state_[1] * t) + state_[2] * t * t / 2.0;
  new_state[1] = state_[1] + state_[2] * t;
  new_state[2] = state_[2];

  new_state[3] = state_[3] + (state_[4] * t) + state_[5] * t * t / 2.0;
  new_state[4] = state_[4] + state_[5] * t;
  new_state[5] = state_[5];

  return new_state;
}


double Road::get_out_of_line_driving_cost(const utils::Option &option) const
{
  double result = 0;
  for(auto point: option.trajectory)
  {
    const double d  = point[3];
    for (int i =0; i<= num_lanes();i++)
    {
      result += exp(-5.0*pow(d - i*line_width,2));
    }

  }
  return result;
}

double Road::get_out_of_road_driving_cost(const utils::Option &option) const
{
  double result = 0;
  for(auto point: option.trajectory)
  {
    const double d  = point[3];
    result += exp(-10.0*d)+exp(10.0*(d - num_lanes()*line_width));
  }
  return result;
}

double Road::get_collision_cost(const utils::Option &option) const
{
  double result = 0;
  double step_t = 0.02;


    for(auto v: vehicles_)
    {
      for (int i = 0; i < option.trajectory.size(); i++)
      {
      FrenetState v_state = v.second.state_in(option.start_time+i*step_t);
      FrenetState e_state = option.trajectory[i];
      const double x =v_state[0] - e_state[0];
      const double y =v_state[3] - e_state[3];

      const double d_x = v.second.vehicleSize[0];
      const double d_y = v.second.vehicleSize[1]/2;
      double dist =  distance(v_state[0],v_state[3],e_state[0],e_state[3]);
      if (dist > 5*(v.second.vehicleSize[1]+v.second.vehicleSize[0]))
        break;

        //Function that creates a box like shape around the vehicle
        // it is 1 inside the collision boundary with a smooth fall off in the buffer zone and zero far away from the vehicle
      const double penetration_dist =
          fabs(x - d_x) + fabs(x + 2*d_x) +  fabs(y - d_y) +  fabs(y + d_y) - 3 * d_x - 2* d_y;
      result += logistic(-penetration_dist) + 1;

    }

  }
  return result;
}

