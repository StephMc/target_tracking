#ifndef __AFFINE_TRANSFORM_H
#define __AFFINE_TRANSFORM_H

#include "opencv2/core/core.hpp"

class AffineTransform {
  public:
    AffineTransform();
    AffineTransform(cv::Point translation, cv::Point scale,
        cv::Point shear, double rotation); 
    cv::Point getTranslation();
    double getRotation();
    cv::Point getScale();
    cv::Point getShear();
    cv::Point transformPoint(cv::Point point);

  private:
    void createTransformMatrix();

    cv::Point translation;
    cv::Point scale;
    cv::Point shear;
    double rotation;
    cv::Mat transform;
};

#endif
