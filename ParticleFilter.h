#ifndef __PARTICLE_FILTER_H
#define __PARTICLE_FILTER_H

#include "opencv2/core/core.hpp"
#include "PerspectiveTransform.h"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

class ParticleFilter {
  public:
    ParticleFilter(){};
    void initalise(cv::Mat& frame, cv::Mat& track, int numParticles);
    void update(cv::Mat& frame);
    cv::Point getLocation();
    void drawParticles(cv::Mat& dest, cv::Scalar color);

  private:
    class Particle {
      public:
        Particle(PerspectiveTransform t) {
          this->t = t;
          this->score = 1;
        }
        PerspectiveTransform t;
        double score;
    };

    void getCosts(cv::Mat& frame, std::vector<std::pair<double, Particle> >& cdf);
    double inverseScore(double score);
    double squareDiffCost(cv::Mat& source, cv::Mat& track,
        PerspectiveTransform at);
    void resample(std::vector<std::pair<double, Particle> >& cdf, cv::Mat& frame);
    void estimateState();
    Particle findParticle(std::vector<std::pair<double, Particle> >& cdf, 
        double target);
    void mutateTransform(PerspectiveTransform& t,
        cv::Mat& frame, cv::Mat& tracked);

    cv::Mat tracked;
    cv::Mat mask;
    std::vector<Particle> particles;
    double totalCost;
    cv::Point estimateLoc;
    boost::mt19937 rng;
};

#endif
