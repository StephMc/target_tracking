#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <iostream>
#include <stdio.h>

#include "ParticleFilter.h"

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

  namedWindow("Tracking window", CV_WINDOW_NORMAL);
   
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
      particleFilter.initalise(frame, objectToTrack, 10);
      initalised = true;
    }
    particleFilter.update(frame);

    // Draw predicted location
    Point matchLocP = particleFilter.getLocation();
    rectangle(frame, matchLocP,
        Point(matchLocP.x + objectToTrack.cols , matchLocP.y + objectToTrack.rows),
        Scalar::all(100), 2, 8, 0);
    particleFilter.drawParticles(frame, Scalar::all(100));

    imshow("Tracking window", frame);
  }
  return 0;
}
