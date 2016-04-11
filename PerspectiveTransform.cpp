#include "PerspectiveTransform.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdlib.h>

using namespace std;
using namespace cv;

PerspectiveTransform::PerspectiveTransform(Point3d translation, 
    Point3d rotation, Point3d viewingAngle) {
  this->translation = translation;
  this->rotation = rotation;
  this->viewingAngle = viewingAngle;
  createTransformMatrix();
  init = true;
}

PerspectiveTransform::PerspectiveTransform() {
  init = false;
}
    
PerspectiveTransform::PerspectiveTransform(const PerspectiveTransform &a) {
  translation = a.getTranslation();
  rotation = a.getRotation();
  viewingAngle = a.getViewingAngle();
  a.getTransform().copyTo(transform);
  init = true;
}

Point3d PerspectiveTransform::getTranslation() const {
  return translation;
}

Point3d PerspectiveTransform::getRotation() const {
  return rotation;
}

Point3d PerspectiveTransform::getViewingAngle() const {
  return viewingAngle;
}

Mat PerspectiveTransform::getTransform() const {
  return transform;
}

// Point is in template frame of reference
Point PerspectiveTransform::transformPoint(Point point) {
  if (!init) abort();
  double px = point.x + translation.x;
  double py = point.y + translation.y;
  double pz = translation.z;
  double result[3];
  for (int i = 0; i < 3; ++i) {
    double *t = transform.ptr<double>(i);
    result[i] = t[0] * px + t[1] * py + t[2] * pz;
  }
  return Point(result[0] * (viewingAngle.z / result[2]) - viewingAngle.x,
       result[1] * (viewingAngle.z / result[2]) - viewingAngle.y);
}

void PerspectiveTransform::createTransformMatrix() {
  Mat mX = Mat::eye(3, 3, CV_64F);
  mX.at<double>(1, 1) = cos(rotation.x);
  mX.at<double>(1, 2) = -sin(rotation.x);
  mX.at<double>(2, 1) = sin(rotation.x);
  mX.at<double>(2, 2) = cos(rotation.x);
  
  Mat mY = Mat::eye(3, 3, CV_64F);
  mY.at<double>(0, 0) = cos(rotation.y);
  mY.at<double>(0, 2) = sin(rotation.y);
  mY.at<double>(2, 0) = -sin(rotation.y);
  mY.at<double>(2, 2) = cos(rotation.y);
 
  Mat mZ = Mat::eye(3, 3, CV_64F);
  mZ.at<double>(0, 0) = cos(rotation.z);
  mZ.at<double>(0, 1) = -sin(rotation.z);
  mZ.at<double>(1, 0) = sin(rotation.z);
  mZ.at<double>(1, 1) = cos(rotation.z);
 
  transform = mX * (mY * mZ);
}
