#ifndef __AFFINE_TRANSFORM_H
#define __AFFINE_TRANSFORM_H

#include "opencv2/core/core.hpp"

class AffineTransform {
  public:
    AffineTransform();
    AffineTransform(cv::Point translation, cv::Point scale,
        cv::Point shear, double rotation); 
    AffineTransform(const AffineTransform &a);
    cv::Point getTranslation() const;
    double getRotation() const;
    cv::Point getScale() const;
    cv::Point getShear() const;
    cv::Mat getTransform() const;
    cv::Point transformPoint(cv::Point point);

  private:
    void createTransformMatrix();

    cv::Point translation;
    cv::Point scale;
    cv::Point shear;
    double rotation;
    cv::Mat transform;

    bool init;
};

#endif
