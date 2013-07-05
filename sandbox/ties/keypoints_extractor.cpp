#include "keypoints_extractor.h"


void KeypointsExtractor::setFilename (const string filename)
{
    filename_ = filename;
    scale_ = 0.3;
}

void KeypointsExtractor::loadImage()
{
    image_ = cv::imread(filename_, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
    cv::resize(image_, image_low_, cv::Size(), scale_, scale_, CV_INTER_AREA);
}

void KeypointsExtractor::compute()
{
    cout <<  "computing descriptors... ";

    cv::SIFT detector;
    cv::SIFT extractor;

    detector.detect(image_low_, cv_keypoints_);

    cout << "detected: " << cv_keypoints_.size() << " keys" << endl;

    extractor.compute(image_low_, cv_keypoints_, descriptors_);
    cout << "computed!" << endl;


    cout << descriptors_.cols << " per " << descriptors_.rows << endl;

    cout << "Found "<< *descriptors_.size.p << endl;

}




Keypoints::Ptr KeypointsExtractor::getDescriptors()
{
    Keypoints::Ptr keys = Keypoints::Ptr(new Keypoints);
    keys->descriptor_size_ = descriptors_.cols;

    keys->descriptors_.resize(descriptors_.cols * descriptors_.rows);

    for (int i = 0; i < descriptors_.rows; ++i)
    {
        for (int j = 0; j< descriptors_.cols; ++j)
        {
            float el = descriptors_.at<float>(i,j);
            keys->descriptors_.at(i*descriptors_.cols + j) = (unsigned char) el;
        }

    }

    for (int i = 0; i < cv_keypoints_.size(); ++i)
    {
        cv::KeyPoint cv_key = cv_keypoints_.at(i);
        Keypoint::Ptr key (new Keypoint);
        key->x_ = cv_key.pt.x * 1/scale_;
        key->y_ = cv_key.pt.y* 1/scale_;
        keys->keypoints_.push_back(key);
    }
    return keys;
}

void KeypointsExtractor::setScale(float scale)
{
    scale_= scale;
}


