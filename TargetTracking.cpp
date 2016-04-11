#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <iostream>
#include <stdio.h>

#include "ParticleFilter.h"
#include "PerspectiveTransform.h"

using namespace std;
using namespace cv;

int main( int argc, char** argv )
{
  Mat frame;
  VideoCapture video(0);
  if (!video.isOpened()) {
    cout << "Failed to open video stream" << endl;
    return -1;
  }
  video >> frame;
  imshow("d", frame);
  waitKey(0);
   
  Mat objectToTrack = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  if (!objectToTrack.data) {
    cout << "Failed to open template image" << endl;
    return -1;
  }
  imshow("d", objectToTrack);
  waitKey(0);

  ParticleFilter particleFilter;
  bool initalised = false;
  while (true) {
    char key = waitKey(1);
    if (key == 'q') break;
    video >> frame;
    if (!initalised) {
      particleFilter.initalise(frame, objectToTrack, 500);
      initalised = true;
    }
    particleFilter.update(frame);

    // Draw predicted location
    PerspectiveTransform est = particleFilter.getEstimateTransform();
    Point tl(0, 0), tr(objectToTrack.cols, 0), bl(0, objectToTrack.rows),
         br(objectToTrack.cols, objectToTrack.rows);
    line(frame, est.transformPoint(tl), est.transformPoint(tr),
        CV_RGB(255, 0, 0));
    line(frame, est.transformPoint(tr), est.transformPoint(br),
        CV_RGB(255, 0, 0));
    line(frame, est.transformPoint(br), est.transformPoint(bl),
        CV_RGB(255, 0, 0));
    line(frame, est.transformPoint(bl), est.transformPoint(tl),
        CV_RGB(255, 0, 0));

    cout << est.getTranslation() << endl;
    cout << est.getRotation() << endl;
    particleFilter.drawParticles(frame, Scalar::all(100));

    imshow("Tracking window", frame);
  }
  return 0;
}
