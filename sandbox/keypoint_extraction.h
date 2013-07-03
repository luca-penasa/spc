#ifndef KEYPOINT_EXTRACTION_H
#define KEYPOINT_EXTRACTION_H


/// for the keypoint extraction part
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/nonfree/features2d.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <limits>

#include <memory>
#include <opencv2/core/types_c.h>

#include "mm_matches.h"

using namespace std;
struct Keypoint
{
public:
    typedef shared_ptr<Keypoint> Ptr;
    Keypoint() {}

    float x_,y_, scale_, orient_;
    //    unsigned char * descriptor_ptr_;
};



class Keypoints
{
public:

    typedef shared_ptr<Keypoints> Ptr;

    Keypoints (size_t desc_size = 128)
    {
        descriptor_size_ = desc_size;
    }

    size_t getNumberOfKeypoints()
    {
        return keypoints_.size();
    }

    int getSizeOfFeature()
    {
        return descriptor_size_;
    }

    template <typename ScalarT>
    auto getAllDescriptorAsStdVector() -> vector<ScalarT>
    {
        size_t len = descriptor_size_ * getNumberOfKeypoints();
        vector<ScalarT> fdesc (len);

        for (int i = 0; i < len; ++i)
            fdesc.at(i) = (ScalarT) descriptors_.at(i);


        return fdesc;
    }

    Keypoint::Ptr getKeypoint(int id)
    {
        if (id >= keypoints_.size())
            cout << "Wrong keypoint required!" << endl;
        return keypoints_.at(id);
    }

    void pushBack(Keypoint::Ptr point)
    {
        keypoints_.push_back(point);
    }

    Keypoints extractIndices(vector<int> ids)
    {
        Keypoints new_keys;
        for (auto i : ids)
        {
            new_keys.pushBack(this->getKeypoint(i));
        }
        return new_keys;
    }


    string filename_;
    vector <Keypoint::Ptr> keypoints_;

    //the actual container of the descriptors values!
    vector<unsigned char> descriptors_;

    size_t descriptor_size_;
};

class KeypointsReader
{
public:
    KeypointsReader ( string filename ): file_(filename, std::ios::in|std::ios::binary),
        keys_(new Keypoints)
    {
        filename_ = filename;

    }

    void readHeader()
    {
        file_.seekg(0); //sure is at beginning
        file_.read(reinterpret_cast<char *> (&n_keys_), sizeof(int));
        file_.read(reinterpret_cast<char *> (&feat_size_), sizeof(int));
        keys_->descriptor_size_ = feat_size_;
        //        keys_->resize(n_keys_);
    }

    void readKeypoints()
    {
        keys_->descriptors_.resize(n_keys_ * feat_size_);

        //be sure file is at the right position
        file_.seekg(2*sizeof(int)); // is the end of the header!
        float x, y, scale, orient;

        keys_->descriptor_size_ = feat_size_;


        for (int i = 0; i < n_keys_; ++i)
        {
            Keypoint::Ptr key (new Keypoint);
            file_.read(reinterpret_cast<char*>(&key->x_), sizeof(float));
            file_.read(reinterpret_cast<char*>(&key->y_), sizeof(float));
            file_.read(reinterpret_cast<char*>(&key->scale_), sizeof(float));
            file_.read(reinterpret_cast<char*>(&key->orient_), sizeof(float));
            file_.read(reinterpret_cast<char*>((&keys_->descriptors_[0]) + i * feat_size_), sizeof(char) * feat_size_);

            //            key->descriptor_ptr_ = (&keys_->descriptors_[0]) + i * feat_size_;

            keys_->pushBack(key);
        }

        file_.close();
    }

    void read()
    {
        readHeader();
        readKeypoints();
    }

    Keypoints::Ptr getKeypoints()
    {
        return keys_;
    }



    string filename_;
    ifstream file_;

    int n_keys_, feat_size_;
    Keypoints::Ptr keys_;

};

class KeypointsExtractor
{
public:

    void setFilename (const string filename)
    {
        filename_ = filename;       
        scale_ = 0.3;
    }

    void loadImage()
    {        
        image_ = cv::imread(filename_, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
        cv::resize(image_, image_low_, cv::Size(), scale_, scale_, CV_INTER_AREA);
    }

    void compute()
    {
        cout <<  "computing descriptors... ";

        int minHessian = 400;

        cv::SIFT detector;
        cv::SIFT extractor;

//        cv::FlannBasedMatcher matcher;

        detector.detect(image_low_, cv_keypoints_);
        extractor.compute(image_low_, cv_keypoints_, descriptors_);
        cout << "computed!" << endl;

        cout << descriptors_.cols << " per " << descriptors_.rows << endl;

        cout << "desc "<< *descriptors_.size.p << endl;

//        cout << "type " << descriptors_.type();

//        for (int i = 0; i < descriptors_.rows; ++i)
//        {
//            for (int j = 0; i < descriptors_.cols; ++j)
//            {
//                float el = descriptors_.at<float>(i,j);
//                cout << el << " ";
//            }

//            cout << endl;
//        }
    }

    Keypoints::Ptr getDescriptors()
    {
        Keypoints::Ptr keys = Keypoints::Ptr(new Keypoints);
        keys->descriptor_size_ = descriptors_.cols;

        keys->descriptors_.resize(descriptors_.cols * descriptors_.rows);


//        cout << "qua " << descriptors_.at(0) << endl;

//        cout << "full size " << descriptors_.cols * descriptors_.rows << endl;
//        keys->descriptors_.assign(descriptors_.data, descriptors_.data + descriptors_.cols * descriptors_.rows);

        for (int i = 0; i < descriptors_.rows; ++i)
        {
            for (int j = 0; j< descriptors_.cols; ++j)
            {
                float el = descriptors_.at<float>(i,j);

//                cout << i << " " << j;
                keys->descriptors_.at(i*descriptors_.cols + j) = (unsigned char) el;
            }

//            cout << endl;
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




    typedef shared_ptr<KeypointsExtractor> Ptr;

    string filename_;

    cv::Mat image_, image_low_, descriptors_;

    vector<cv::KeyPoint> cv_keypoints_;



    double scale_;
};


class MatchesFilter
{

public:
    enum FILTER_TYPE {MINIMUM, FIRST_NEAREST};

    MatchesFilter() {filter_type_ = MINIMUM; }

    void setInputMatches(vector<vector<Match> > multi_matches)
    {
        in_matches_ = multi_matches;
    }

    void setFilterType(FILTER_TYPE type)
    {
        filter_type_ = type;
    }

    void filter(vector<Match> &matches, float factor)
    {
        switch (filter_type_)
        {
            case MINIMUM:
            {
            float old_dist = std::numeric_limits<float>::max();
            for (int i = 0 ; i < in_matches_.size(); ++i)
            {
                Match match = in_matches_.at(i).at(0);
                float dist = match.distance_;
                if (old_dist > dist )
                    old_dist = dist;

            }

            float min_distance = old_dist;
            cout << "Found min distance: " << min_distance <<  endl;

            float discriminant_distance = factor * min_distance;
            for (int i = 0 ; i < in_matches_.size(); ++i)
            {
                Match match = in_matches_.at(i).at(0);
                if (match.distance_ < discriminant_distance)
                {
                    matches.push_back(match);
                }

            }



            break;
            }


            case FIRST_NEAREST:
            {

            //filtering!
            for (int i = 0; i < in_matches_.size(); ++i)
            {
                Match m1 = in_matches_.at(i).at(0);
                Match m2 = in_matches_.at(i).at(1);

//                int nn = in_matches_.at(0).size();
                if (m1.distance_ < m2.distance_ * factor)
                {

                    matches.push_back(m1);
                }

            }

                break;
            }
            default:
            {
                break;
            }
        }
    }


    vector<vector<Match> > in_matches_;
    FILTER_TYPE filter_type_;
};

#endif // KEYPOINT_EXTRACTION_H
