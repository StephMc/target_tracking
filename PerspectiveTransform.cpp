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
  Mat p(3,1,CV_64F);
  p.at<double>(0,0) = point.x + translation.x;
  p.at<double>(1,0) = point.y + translation.y;
  p.at<double>(2,0) = translation.z;
  Mat result = transform * p;
  //cout << transform << " * " << p << " = " << result << endl;
  return Point(result.at<double>(0, 0) * 
      (viewingAngle.z / result.at<double>(0, 2)) - viewingAngle.x,
       result.at<double>(1, 0) * 
       (viewingAngle.z / result.at<double>(0, 2)) - viewingAngle.y);
}

void PerspectiveTransform::createTransformMatrix() {
  Mat mX = Mat(3, 3, CV_64F, cvScalar(0.0));
  mX.at<double>(0, 0) = 1;
  mX.at<double>(1, 1) = cos(rotation.x);
  mX.at<double>(1, 2) = -sin(rotation.x);
  mX.at<double>(2, 1) = sin(rotation.x);
  mX.at<double>(2, 2) = cos(rotation.x);
  
  Mat mY = Mat(3, 3, CV_64F, cvScalar(0.0));
  mY.at<double>(0, 0) = cos(rotation.y);
  mY.at<double>(0, 2) = sin(rotation.y);
  mY.at<double>(1, 1) = 1;
  mY.at<double>(2, 0) = -sin(rotation.y);
  mY.at<double>(2, 2) = cos(rotation.y);
 
  Mat mZ = Mat(3, 3, CV_64F, cvScalar(0.0));
  mZ.at<double>(0, 0) = cos(rotation.z);
  mZ.at<double>(0, 1) = -sin(rotation.z);
  mZ.at<double>(1, 0) = sin(rotation.z);
  mZ.at<double>(1, 1) = cos(rotation.y);
  mZ.at<double>(2, 2) = 1;
 
  transform = mX * (mY * mZ);
}
