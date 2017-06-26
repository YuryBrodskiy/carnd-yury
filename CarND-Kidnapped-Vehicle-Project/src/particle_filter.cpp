/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;


// Helper functions

/** predicts the next position of the particle based on the system state
 *
 */
Particle motionModel(const Particle& particle, double dt, double v, double dfi)
{
  Particle result = particle;
  const double px = particle.x;
  const double py = particle.y;
  const double fi = particle.theta;

  if (fabs(dfi) < 0.0001)
  {
    //            value   + derivative*time + noise
    result.x = px + v * cos(fi) * dt;
    result.y = py + v * sin(fi) * dt;
  }
  else
  {
    result.x = px + v / dfi * (sin(fi + dfi * dt) - sin(fi));
    result.y = py + v / dfi * (-cos(fi + dfi * dt) + cos(fi));
  }

  result.theta = fi + dfi * dt;

  return result;
}

/**
 * A functor to project the observed landmark to the map using particle state
 */
struct ProjectToMap
{
  ProjectToMap(double _px,double _py,double _fi)
  :px(_px),py(_py),fi(_fi){}
    double px;
    double py;
    double fi;
    LandmarkObs operator () (const LandmarkObs& lm)
    {
      LandmarkObs obs_on_map;
      obs_on_map.id = lm.id;
      obs_on_map.x = px + lm.x* cos(fi) - lm.y*sin(fi);
      obs_on_map.y = py + lm.x* sin(fi) + lm.y*cos(fi);
      return obs_on_map;
    }
};

/**
 * A functor to compute the distance between observed landmark and map landmark
 * A squared distance is used for efficency
 */
struct GetSquaredDistance
{
  GetSquaredDistance(const LandmarkObs& _src): src(_src){}
  LandmarkObs src;
  double operator () (const Map::single_landmark_s& trg)
  {
    return (src.x-trg.x_f)*(src.x-trg.x_f)+ (src.y-trg.y_f)*(src.y-trg.y_f);
  }
};
/**
 *  A functor returns a pair observed landmark and nearest landmark on the map
 */
struct GetNearestBestMatch
{
  GetNearestBestMatch(const Map& _map):map(&_map){}
  const Map* map;
  std::pair<LandmarkObs,Map::single_landmark_s> operator() (const LandmarkObs& lm)
  {
    std::vector<double> dists(map->landmark_list.size());
    std::transform(map->landmark_list.begin(), map->landmark_list.end(), dists.begin(),
        GetSquaredDistance(lm));
    size_t smallest_index = std::distance(dists.begin(),std::min_element(dists.begin(), dists.end()));
    return std::pair<LandmarkObs, Map::single_landmark_s>(lm, map->landmark_list[smallest_index]);
  }
};

/**
 *  a functor return probability density that the observed landmark is the map land mark
 */
struct  Gaussian
{
  Gaussian(  const double* _std):std(_std){}
  const double* std;
  double operator()(const std::pair<LandmarkObs, Map::single_landmark_s>&  match)
  {
    const double dx = match.first.x - match.second.x_f;
    const double dy = match.first.y - match.second.y_f;
    const double result = exp(-0.5*(dx*dx/std[0] + dy*dy/std[1]) )/ sqrt(2.0 * M_PI * std[0] * std[1] );
    return result;
  }
};


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// DONE: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  num_particles = 500;
  default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_th(theta, std[2]);
  for (int i = 0; i < num_particles; ++i)
  {
    Particle particle = {i, dist_x(gen), dist_y(gen), dist_th(gen), 1.0 , std::vector<int>(), std::vector<double>(),std::vector<double>()};
    particles.push_back(particle);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// DONE: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  default_random_engine gen;
  for (int i = 0; i < num_particles; ++i)
  {
    particles[i] = motionModel(particles[i], delta_t, velocity, yaw_rate);
    particles[i].x     += normal_distribution<double>(0, std_pos[0])(gen);
    particles[i].y     += normal_distribution<double>(0, std_pos[1])(gen);
    particles[i].theta += normal_distribution<double>(0, std_pos[2])(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
  //not used
  //TO reviewer: this is done via functors defined above

}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// DONE: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html


  for (int i = 0; i < num_particles; ++i)
  {

    std::vector<LandmarkObs> observations_on_map(observations.size());
    std::transform(observations.begin(), observations.end(), observations_on_map.begin(),
        ProjectToMap(particles[i].x, particles[i].y, particles[i].theta));

    std::vector<std::pair<LandmarkObs, Map::single_landmark_s> > matches(observations.size());
    std::transform(observations_on_map.begin(), observations_on_map.end(), matches.begin(),
            GetNearestBestMatch(map_landmarks));
    particles[i] = SetAssociations(particles[i], matches);

    std::vector<double> probs(matches.size());
    std::transform(matches.begin(), matches.end(), probs.begin(),
                Gaussian(std_landmark));
    double product = std::accumulate(probs.begin(), probs.end(), 1.0, std::multiplies<double>());

    particles[i].weight = product;
  }

}

void ParticleFilter::resample() {
	// DONE: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  //default_random_engine gen;
  std::random_device rd;
  std::mt19937 gen_idex(rd());
  std::discrete_distribution<> index_dist(num_particles,0,num_particles,[](double x){return x;});
  size_t index = index_dist(gen_idex);
  default_random_engine gen;

  const double w_max = std::max_element(particles.begin(),particles.end(),
      [](const Particle& lhs, const Particle & rhs)
      { return lhs.weight < rhs.weight; })->weight;
  std::uniform_real_distribution<double> dist(0.0,w_max);
  double beta = 0;
  std::vector<Particle> new_particles;
  for(int i = 0; i < num_particles; ++i)
  {
    beta = beta + dist(gen);
    while(particles[index].weight < beta)
    {
      beta = beta - particles[index].weight;
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }
  particles.clear();
  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, const std::vector<std::pair<LandmarkObs,Map::single_landmark_s> >& matches)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();
  for (std::vector<std::pair<LandmarkObs,Map::single_landmark_s> >::const_iterator it = matches.begin();it!= matches.end(); it++)
  {
    particle.associations.push_back(it->second.id_i);
    particle.sense_x.push_back(it->second.x_f);
    particle.sense_y.push_back(it->second.y_f);
  }

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
