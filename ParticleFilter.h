#ifndef __PARTICLE_FILTER_H
#define __PARTICLE_FILTER_H

#include "opencv2/core/core.hpp"

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
        Particle(int x, int y) {
          this->x = x;
          this->y = y;
          this->score = 1;
        }
        int x;
        int y;
        double score;
    };

    void getCosts(cv::Mat& frame, std::vector<std::pair<double, Particle> >& cdf);
    double inverseScore(double score);
    double squareDiffCost(cv::Mat& source, cv::Mat& track);
    void resample(std::vector<std::pair<double, Particle> >& cdf, cv::Mat& frame);
    void estimateState();
    Particle findParticle(std::vector<std::pair<double, Particle> >& cdf, 
        double target);

    cv::Mat tracked;
    cv::Mat mask;
    std::vector<Particle> particles;
    double totalCost;
    cv::Point estimateLoc;
};

#endif
