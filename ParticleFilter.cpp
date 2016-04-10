#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ParticleFilter.h"
#include <stdlib.h>
#include <iostream>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

using namespace std;
using namespace cv;

void ParticleFilter::initalise(Mat& frame, Mat& track, int numParticles) {
  track.copyTo(tracked);
  particles.clear();
  // Randomly distribute particles over image
  for (int i = 0; i < numParticles; ++i) {
    Particle p = Particle(
        PerspectiveTransform(
            Point3d(0, 0, frame.cols), // Translation
            Point3d(0, 0, 0), // Rotation
            Point3d(-frame.cols / 2, -frame.rows / 2, frame.cols)));
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
  Point3d va = particles[0].t.getViewingAngle();
  for (vector<Particle>::iterator it = particles.begin(); 
      it != particles.end(); ++it) {
    Point3d p = it->t.getTranslation();
    avX += p.x * ((double)it->score / totalCost);
    avY += p.y * ((double)it->score / totalCost);
  }
  estimateLoc.x = avY - va.y;
  estimateLoc.y = avX - va.x;
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

PerspectiveTransform
    ParticleFilter::mutateTransform(PerspectiveTransform& t, Mat& frame,
    Mat& tracked) {
  int noise = 20;
  Point3d trans = t.getTranslation();
  // todo: gaussian
  boost::normal_distribution<> nd(0.0, 1.0);
  boost::variate_generator<boost::mt19937&, 
      boost::normal_distribution<> > var_nor(rng, nd);
  trans.x += var_nor() * noise;
  trans.y += var_nor() * noise;
  // keep the particle in the image
  trans.x = trans.x < -frame.rows / 2 ? -frame.rows / 2 :
    trans.x > (frame.rows / 2) - tracked.rows ?
    (frame.rows / 2) - tracked.rows : trans.x;
  trans.y = trans.y < -frame.cols / 2 ? -frame.cols / 2 :
    trans.y > (frame.cols / 2) - tracked.cols ?
    (frame.cols / 2) - tracked.cols : trans.y;
  return 
    PerspectiveTransform(trans, Point3d(0, 0, 0), t.getViewingAngle());
}

void ParticleFilter::resample(vector<pair<double, Particle> >& cdf,
    Mat& frame) {
  for (vector<Particle>::iterator it = particles.begin(); 
      it != particles.end(); ++it) {
    double random = ((double)rand() / (double)RAND_MAX) * totalCost;
    Particle p = findParticle(cdf, random);
    *it = p;
    it->t = mutateTransform(it->t, frame, tracked);
  }
}

void ParticleFilter::getCosts(Mat& frame, vector<pair<double,
  Particle> >& cdf) {
  totalCost = 0;
  for (vector<Particle>::iterator it = particles.begin(); 
      it != particles.end(); ++it) {
    //cout << it->t.getTransform() << endl;
    double score = squareDiffCost(frame, tracked, it->t);
    it->score = inverseScore(score);
    totalCost += it->score;
    //cout << totalCost << " " << score << " " << it->score << endl;
    cdf.push_back(make_pair(totalCost, *it));
  }
}

double ParticleFilter::inverseScore(double score) {
  // Doesn't take into account the mask - doubt this makes a different
  double cap = 50;
  double maxScore = tracked.cols * tracked.rows * 3 * cap * cap;
  if (score > maxScore) score = maxScore;
  return ((maxScore - score) / maxScore) + 0.00001;
}

double ParticleFilter::squareDiffCost(Mat& frame, Mat& track,
    PerspectiveTransform at) {
  double totalCost = 0;
  for (int row = 0; row < track.rows; ++row) {
    Vec3b *t = track.ptr<Vec3b>(row);
    for (int col = 0; col < track.cols; ++col) {
      Point fp = at.transformPoint(Point(row, col));
      //cout << "Transformed (" << row << " " << col << ") to (" <<
      //  fp.x << " " << fp.y << ")" << endl;
      Vec3b *fr = frame.ptr<Vec3b>(fp.x);
      int r = (int)t[col][0] - (int)fr[fp.y][0];
      int g = (int)t[col][1] - (int)fr[fp.y][1];
      int b = (int)t[col][2] - (int)fr[fp.y][2];
      totalCost += (r * r) + (g * g) + (b * b);
    }
  }
  return totalCost;
}

Point ParticleFilter::getLocation() {
  return estimateLoc;
}

void ParticleFilter::drawParticles(Mat& dest, Scalar color) {
  for (vector<Particle>::iterator it = particles.begin();
      it != particles.end(); ++it) {
    // X & Y has been inversed...
    Point3d p = it->t.getTranslation();
    Point3d va = it->t.getViewingAngle();
    circle(dest, Point(p.y - va.y, p.x - va.x), 3, color, -1);
  }
}
