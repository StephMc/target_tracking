#ifndef __PERSPECTIVE_TRANSFORM_H
#define __PERSPECTIVE_TRANSFORM_H

#include "opencv2/core/core.hpp"

class PerspectiveTransform {
  public:
    PerspectiveTransform();
    PerspectiveTransform(cv::Point3d translation, cv::Point3d rotation,
        cv::Point3d viewingAngle); 
    PerspectiveTransform(const PerspectiveTransform &a);
    cv::Point3d getTranslation() const;
    cv::Point3d getRotation() const;
    cv::Point3d getViewingAngle() const;
    cv::Mat getTransform() const;
    cv::Point transformPoint(cv::Point point);

  private:
    void createTransformMatrix();

    cv::Point3d translation;
    cv::Point3d rotation;
    cv::Point3d viewingAngle;
    cv::Mat transform;

    cv::Mat result;
    cv::Mat p;
    bool init;
};

#endif
