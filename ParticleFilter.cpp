#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ParticleFilter.h"
#include <stdlib.h>
#include <iostream>

using namespace std;
using namespace cv;

void ParticleFilter::initalise(Mat& frame, Mat& track, int numParticles) {
  track.copyTo(tracked);
  particles.clear();
  // Randomly distribute particles over image
  for (int i = 0; i < numParticles; ++i) {
    double x = ((double)rand() / (double)RAND_MAX) * 
        (frame.cols - track.cols);
    double y = ((double)rand() / (double)RAND_MAX) * 
        (frame.rows - track.rows);
    Particle p = Particle(x, y);
    particles.push_back(p);
  } 
}

void ParticleFilter::update(Mat& frame) {
  // drift (ignore when no v tracked)
  vector<pair<double, Particle> > cdf;
  getCosts(frame, cdf);
  estimateState();
  resample(cdf, frame);
}

void ParticleFilter::estimateState() {
  // Weighted average
  double avX = 0;
  double avY = 0;
  for (vector<Particle>::iterator it = particles.begin(); it != particles.end();
    ++it) {
    avX += (double)it->x * ((double)it->score / totalCost);
    avY += (double)it->y * ((double)it->score / totalCost);
  }
  estimateLoc.x = avX;
  estimateLoc.y = avY;
}

ParticleFilter::Particle ParticleFilter::findParticle(
    vector<pair<double, Particle> >& cdf, double target) {
  int index = cdf.size() / 2;
  int offset = index;
  for (int i = 0; i < cdf.size(); ++i) {
    if (cdf[index].first > target && index == 0) {
      return cdf[index].second;
    }
    if (cdf[index].first > target && cdf[index - 1].first < target) {
      return cdf[index].second;
    }
    offset /= 2;
    if (offset == 0) offset = 1;
    if (cdf[index].first > target) index -= offset;
    else if (cdf[index].first < target) index += offset;
  }
  cout << "find failed" << endl;
  abort();
}

void ParticleFilter::resample(vector<pair<double, Particle> >& cdf, Mat& frame) {
  for (vector<Particle>::iterator it = particles.begin(); it != particles.end();
      ++it) {
    double random = ((double)rand() / (double)RAND_MAX) * totalCost;
    Particle p = findParticle(cdf, random);
    *it = p;
    // Add some noise
    int noise = 20;
    it->x += -(noise/2) + (((double)rand() / (double)RAND_MAX) * noise);
    it->y += -(noise/2) + (((double)rand() / (double)RAND_MAX) * noise);
    // keep the particle in the image
    it->x = it->x < 0 ? 0 :
      it->x > frame.cols - tracked.cols ? frame.cols - tracked.cols :
      it->x;
    it->y = it->y < 0 ? 0 :
      it->y > frame.rows - tracked.rows ? frame.rows - tracked.rows :
      it->y;
  }
}

void ParticleFilter::getCosts(Mat& frame, vector<pair<double, Particle> >& cdf) {
  totalCost = 0;
  for (vector<Particle>::iterator it = particles.begin(); 
      it != particles.end(); ++it) {
    cout << it->x << " " << it->y << endl;
    Rect myROI(it->x, it->y, tracked.cols, tracked.rows);
    Mat roi = frame(myROI);
    double score = squareDiffCost(roi, tracked);
    it->score = inverseScore(score);
    totalCost += it->score;
    cout << totalCost << " " << score << " " << it->score << endl;
    cdf.push_back(make_pair(totalCost, *it));
  }
}

double ParticleFilter::inverseScore(double score) {
  // Doesn't take into account the mask - doubt this makes a different
  double cap = 25;
  double maxScore = tracked.cols * tracked.rows * 3 * cap * cap;
  if (score > maxScore) score = maxScore;
  return ((maxScore - score) / maxScore) + 0.00001;
}

double ParticleFilter::squareDiffCost(Mat& source, Mat& track) {
  double totalCost = 0;
  for (int row = 0; row < track.rows; ++row) {
    Vec3b *t = track.ptr<Vec3b>(row);
    Vec3b *s = source.ptr<Vec3b>(row);
    for (int col = 0; col < track.cols; ++col) {
      int r = (int)t[col][0] - (int)s[col][0];
      int g = (int)t[col][1] - (int)s[col][1];
      int b = (int)t[col][2] - (int)s[col][2];
      totalCost += (r * r) + (g * g) + (b * b);
    }
  }
  return totalCost;
}


Point ParticleFilter::getLocation() {
  return estimateLoc;
}

void ParticleFilter::drawParticles(Mat& dest, Scalar color) {
  for (vector<Particle>::iterator it = particles.begin(); it != particles.end();
      ++it) {
    circle(dest, Point(it->x, it->y), 3, color, -1);
  }
}
