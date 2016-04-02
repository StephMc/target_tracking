#include "AffineTransform.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdlib.h>

using namespace std;
using namespace cv;

AffineTransform::AffineTransform(Point translation, Point scale,
    Point shear, double rotation) {
  this->translation = translation;
  this->scale = scale;
  this->shear = shear;
  this->rotation = rotation;
  createTransformMatrix();
  init = true;
}

AffineTransform::AffineTransform() {
  init = false;
}
    
AffineTransform::AffineTransform(const AffineTransform &a) {
  translation = a.getTranslation();
  scale = a.getScale();
  shear = a.getShear();
  rotation = a.getRotation();
  //createTransformMatrix();
  a.getTransform().copyTo(transform);
  init = true;
}

Point AffineTransform::getTranslation() const {
  return translation;
}

double AffineTransform::getRotation() const {
  return rotation;
}

Point AffineTransform::getScale() const {
  return scale;
}

Point AffineTransform::getShear() const {
  return shear;
}

Mat AffineTransform::getTransform() const {
  return transform;
}

Point AffineTransform::transformPoint(Point point) {
  if (!init) abort();
  Mat p(3,1,CV_64F);
  p.at<double>(0,0) = point.x;
  p.at<double>(1,0) = point.y;
  p.at<double>(2,0) = 1;
  Mat result = transform * p;
  //cout << transform << " * " << p << " = " << result << endl;
  return Point(result.at<double>(0, 0), result.at<double>(1, 0));
}

void AffineTransform::createTransformMatrix() {
  Mat translationM = Mat(3, 3, CV_64F, cvScalar(0.0));
  translationM.at<double>(0, 0) = 1;
  translationM.at<double>(0, 2) = translation.x;
  translationM.at<double>(1, 1) = 1;
  translationM.at<double>(1, 2) = translation.y;
  translationM.at<double>(2, 2) = 1;
  
  Mat scaleM = Mat(3, 3, CV_64F, cvScalar(0.0));
  scaleM.at<double>(0, 0) = scale.x;
  scaleM.at<double>(1, 1) = scale.y;
  scaleM.at<double>(2, 2) = 1;

  Mat shearM = Mat(3, 3, CV_64F, cvScalar(0.0));
  shearM.at<double>(0, 0) = 1;
  shearM.at<double>(0, 1) = shear.x;
  shearM.at<double>(1, 1) = 1;
  shearM.at<double>(1, 0) = shear.y;
  shearM.at<double>(2, 2) = 1;
 
  Mat rotationM = Mat(3, 3, CV_64F, cvScalar(0.0));
  rotationM.at<double>(0, 0) = cos(rotation);
  rotationM.at<double>(0, 1) = -sin(rotation);
  rotationM.at<double>(1, 0) = sin(rotation);
  rotationM.at<double>(1, 1) = cos(rotation);
  rotationM.at<double>(2, 2) = 1; 
  
  // translation * shearing * scaling * rotation
 // cout << "translation matrix " << translationM << endl;
  //cout << "shear matrix " << shearM << endl;
  //cout << "scale matrix " << scaleM << endl;
  //cout << "rotation matrix " << rotationM << endl;
  Mat result = translationM * (shearM * (scaleM * rotationM));
  //cout << "result " << result << endl;

  transform = Mat(2, 3, CV_64F, cvScalar(0.0));
  transform.at<double>(0, 0) = result.at<double>(0, 0);
  transform.at<double>(0, 1) = result.at<double>(0, 1);
  transform.at<double>(0, 2) = result.at<double>(0, 2);
  transform.at<double>(1, 0) = result.at<double>(1, 0);
  transform.at<double>(1, 1) = result.at<double>(1, 1);
  transform.at<double>(1, 2) = result.at<double>(1, 2);
  //cout << "transform " << transform << endl;
  //waitKey(0);
}
