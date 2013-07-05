#ifndef KEYPOINT_EXTRACTION_H
#define KEYPOINT_EXTRACTION_H




#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <sstream>

#include <limits>

#include <memory>
#include <opencv2/core/types_c.h>

#include "matches.h"



using namespace std;

///
/// \brief The Keypoint struct
///
struct Keypoint
{
public:
    typedef shared_ptr<Keypoint> Ptr;
    Keypoint()
    {
        x_ = numeric_limits<float>::quiet_NaN();
        y_ = numeric_limits<float>::quiet_NaN();
        orient_ = numeric_limits<float>::quiet_NaN();
        scale_ = numeric_limits<float>::quiet_NaN();
    }

    float x_,y_, scale_, orient_;

};


///
/// \brief The Keypoints class
///
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






#endif // KEYPOINT_EXTRACTION_H
