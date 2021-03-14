/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * Updated: 2021/03/14 by seongcho 
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

using std::string;
using std::vector;
using std::normal_distribution;
using std::uniform_int_distribution;
using std::uniform_real_distribution;
using std::max_element;


//using namespace std;

static std::default_random_engine gen;
 

void ParticleFilter::init(double x, double y, double theta, double std[]) {

	/// Set the number of particles
	num_particles = 100;  
  
	/// Initialize all particles to first position with weight = 1
	///     (based on estimates of x, y, theta and their uncertainties from GPS) 
	normal_distribution<double> nd_x(x, std[0]);
	normal_distribution<double> nd_y(y, std[1]);
	normal_distribution<double> nd_theta(theta, std[2]);
  
	int i = -1;
	while (++i < num_particles)
	{
		Particle sample;
  
		sample.id = i;
		/// Add random Gaussian noise to each particle
		sample.x = nd_x(gen);
		sample.y = nd_y(gen);
		sample.theta = nd_theta(gen); 
		// all weights to 1
		sample.weight = 1.0;

		particles.push_back(sample);
		weights.push_back(sample.weight);
	}
  
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
 
	/// Add measurements to each particle and add random Gaussian noise   
	int i;
  	double x0, y0, theta0;
  	double x1, y1, theta1;
  
	i = -1;
  	while (++i < num_particles)
	{
		x0 = particles[i].x;
		y0 = particles[i].y;
		theta0 = particles[i].theta;

		if (fabs(yaw_rate) > 0.0001)	//theta0 != 0
		{
			theta1 = theta0 + yaw_rate * delta_t;
			x1 = x0 + (velocity / yaw_rate) * (sin(theta1) - sin(theta0));
			y1 = y0 + (velocity / yaw_rate) * (cos(theta0) - cos(theta1));
		}
		else							//theta0 = 0
		{
			theta1 = theta0;
			x1 = x0 + velocity * cos(theta0) * delta_t;
			y1 = y0 + velocity * sin(theta0) * delta_t;
		}        
      
		normal_distribution<double> nd_x(x1, std_pos[0]);
		normal_distribution<double> nd_y(y1, std_pos[1]);
		normal_distribution<double> nd_theta(theta1, std_pos[2]);  
      
		particles[i].x = nd_x(gen);
		particles[i].y = nd_y(gen);
		particles[i].theta = nd_theta(gen);     
	}  
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  
	/// Find the nearest neighbour landmark measurement of each observed measurement 
	/// then pair observations[i].id and nearest_landmark_id  
	double dist_ij, dist_min;
	int nearest_landmark_id;
	int i, j; 
	
	i = -1;
	while (++i < (int)observations.size())
	{
		dist_min = std::numeric_limits<double>::max();
		nearest_landmark_id = -1;

		j = -1;
		while (++j < (int)predicted.size())
        {
			dist_ij = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
			if (dist_ij < dist_min)
			{
				dist_min = dist_ij;
				nearest_landmark_id = predicted[j].id;
			}
		}
		observations[i].id = nearest_landmark_id;
	}
   
}

double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));
    
  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
    
  return weight;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
   
	int i, j, k;
	double weight_sum = 0.0;
  
	i = -1;  
	while (++i < (int)particles.size())
	{
		Particle &p = particles[i];
      
		// Filter map landmarks to keep only in the sensor_range of from particle p
		vector<LandmarkObs> predicted_lmobs;
		j = -1;
		while (++j < (int)map_landmarks.landmark_list.size()) 
		{
			Map::single_landmark_s lm = map_landmarks.landmark_list[j];
			if ((fabs((p.x - lm.x_f)) <= sensor_range) && (fabs((p.y - lm.y_f)) <= sensor_range)) 
			{
				LandmarkObs predicted_lmob;
				predicted_lmob.id = lm.id_i;
				predicted_lmob.x = lm.x_f;
				predicted_lmob.y = lm.y_f;              
				predicted_lmobs.push_back(predicted_lmob);
			}
		}
      
		// Transform obs from vehicle coords to map coords
		vector<LandmarkObs> trans_obs;
		j = -1;
		while (++j < (int)observations.size()) 
		{
			LandmarkObs ob = observations[j];
          	LandmarkObs trans_ob;
			trans_ob.id = j;
			trans_ob.x = p.x + (cos(p.theta) * ob.x) - (sin(p.theta) * ob.y);
			trans_ob.y = p.y + (sin(p.theta) * ob.x) + (cos(p.theta) * ob.y);
			trans_obs.push_back(trans_ob);
		}
      
		// Associate observations with predicted landmarks using nearest neighbor algorithm
		dataAssociation(predicted_lmobs, trans_obs);
      
 		// Calculate the weight of each particle using Multivariate Gaussian distribution.
		p.weight = 1.0;
		j = -1;
		while (++j < (int)trans_obs.size())
		{
			LandmarkObs	&t_ob = trans_obs[j];
			k = -1;
			while (++k < (int)predicted_lmobs.size())
            {
				LandmarkObs &p_lmob = predicted_lmobs[k];
				if (t_ob.id == p_lmob.id)
					p.weight *= multiv_prob(std_landmark[0], std_landmark[1], 
											t_ob.x, t_ob.y, p_lmob.x, p_lmob.y); 
			}   
        }  
      
		weight_sum += p.weight;
	//eoWhile(p)  
	}
  
	// Normalize the weights of all particles; weight /= weight_sum
 	i = -1;
 	while (++i < (int)particles.size()) 
	{
		particles[i].weight /= weight_sum;
		weights[i] = particles[i].weight;
	}  

//eoFunc
}

void ParticleFilter::resample() {

 
	/// Resample particles with replacement with probability proportional to their weight
  	vector<Particle> new_particles;
	uniform_int_distribution<int> ud_p_idx(0, num_particles - 1);  
	int p_index = ud_p_idx(gen);
	double beta = 0.0;
	double max_weight = *max_element(weights.begin(), weights.end());
	int i;  
  
	i = -1;
	while (++i < (int)particles.size()) 
	{
		uniform_real_distribution<double> random_weight(0.0, max_weight * 2.0);
		beta += random_weight(gen);

	 	while (weights[p_index] < beta) 
		{
			beta -= weights[p_index];
			p_index = (p_index + 1) % num_particles;
		}
		new_particles.push_back(particles[p_index]);
	}
	
	particles = new_particles;
    
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
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
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}