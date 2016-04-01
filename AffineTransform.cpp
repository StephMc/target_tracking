#include "AffineTransform.h"
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

AffineTransform::AffineTransform(Point translation, Point scale,
    Point shear, double rotation) {
  this->translation = translation;
  this->scale = scale;
  this->shear = shear;
  this->rotation = rotation;
  createTransformMatrix();
}

AffineTransform::AffineTransform() {
  AffineTransform(Point(0, 0), Point(1, 1), Point(0, 0), 0);
}

Point AffineTransform::getTranslation() {
  return translation;
}

double AffineTransform::getRotation() {
  return rotation;
}

Point AffineTransform::getScale() {
  return scale;
}

Point AffineTransform::getShear() {
  return shear;
}

Point AffineTransform::transformPoint(Point point) {
  Mat p(3,1,CV_64F);
  p.at<double>(0,0) = point.x;
  p.at<double>(1,0) = point.y;
  p.at<double>(2,0) = 1;
  Mat result = transform * p;
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
  translationM.at<double>(0, 0) = scale.x;
  translationM.at<double>(1, 1) = scale.y;
  translationM.at<double>(2, 2) = 1;

  Mat shearM = Mat(3, 3, CV_64F, cvScalar(0.0));
  translationM.at<double>(0, 0) = 1;
  translationM.at<double>(0, 1) = shear.x;
  translationM.at<double>(1, 1) = 1;
  translationM.at<double>(1, 0) = shear.y;
  translationM.at<double>(2, 2) = 1;
 
  Mat rotationM = Mat(3, 3, CV_64F, cvScalar(0.0));
  translationM.at<double>(0, 0) = cos(rotation);
  translationM.at<double>(0, 1) = -sin(rotation);
  translationM.at<double>(1, 0) = sin(rotation);
  translationM.at<double>(1, 1) = cos(rotation);
  translationM.at<double>(2, 2) = 1; 
  
  // translation * shearing * scaling * rotation
  Mat result = translationM * (shearM * (scaleM * rotation));
  transform = Mat(result.rows - 1, result.cols, CV_64F, result.data);
}
