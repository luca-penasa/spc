#ifndef KEYPOINTS_EXTRACTOR_H
#define KEYPOINTS_EXTRACTOR_H

//std
#include <string>

//local
#include "keypoints.h"

//opencv
#include <opencv2/opencv.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

using namespace std;


///
/// \brief The KeypointsExtractor class
///
class KeypointsExtractor
{
public:
    void setFilename (const string filename);

    void loadImage();

    void compute();

    Keypoints::Ptr getDescriptors();

    void setScale(float scale);

private:
    typedef shared_ptr<KeypointsExtractor> Ptr;

    string filename_;

    cv::Mat image_, image_low_, descriptors_;

    vector<cv::KeyPoint> cv_keypoints_;

    float scale_;
};


#endif // KEYPOINTS_EXTRACTOR_H
