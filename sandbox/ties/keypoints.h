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

///
/// \brief The KeypointsWriter class
///
class KeypointsWriter
{
public:
    KeypointsWriter() {}


    void setFilename(string fname)
    {
        filename_ = fname;
    }

    void setKeypoints(Keypoints::Ptr keys)
    {
        keys_ = keys;
    }

    int write()
    {
        Keypoints keys = *keys_;
        std::ofstream binaryFile(filename_.c_str(), std::ios::out|std::ios::binary|std::ios::trunc);
        int sz = keys.getNumberOfKeypoints();
        int szDesc = keys.getSizeOfFeature();

        binaryFile.write((char*) &sz ,sizeof(int));
        binaryFile.write((char*) &szDesc, sizeof(int));

        for (int i = 0; i < sz; ++i) {

            Keypoint k = *keys.getKeypoint(i);

            binaryFile.write(reinterpret_cast<char*>(&k.x_), sizeof(float));
            binaryFile.write(reinterpret_cast<char*>(&k.y_), sizeof(float));
            binaryFile.write(reinterpret_cast<char*>(&k.orient_), sizeof(float));
            binaryFile.write(reinterpret_cast<char*>(&k.scale_), sizeof(float));

            //also the descriptors must be written !
            for (int j = 0; j < szDesc; ++j) {
              unsigned char elem = keys.descriptors_.at(i*szDesc + j) ;
              binaryFile.write(reinterpret_cast<char*>(&elem), sizeof(char));
            }
        }

        binaryFile.close();
        //TODO: handle error return value
        return 1;
    }

 string filename_;

 Keypoints::Ptr keys_;
};


///
/// \brief The KeypointsReader class
///
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


bool compare_cv_keys_xy(const cv::KeyPoint &a, const cv::KeyPoint &b)
{

    if (a.pt.x == b.pt.x)
    {
        return (a.pt.y > b.pt.y);
    }
    else
    {
        return (a.pt.x > b.pt.x);
    }
}

bool are_equal_cv_keys_xy(const cv::KeyPoint &a, const cv::KeyPoint &b)
{

    return ((a.pt.x == b.pt.x) & (a.pt.y == b.pt.y));
}



///
/// \brief The KeypointsExtractor class
///
class KeypointsExtractor
{
public:

    void setFilename (const string filename)
    {
        remove_duplicates_ = true;
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

        cv::SIFT detector;
        cv::SIFT extractor;

        detector.detect(image_low_, cv_keypoints_);

        cout << "detected: " << cv_keypoints_.size() << " keys" << endl;

        extractor.compute(image_low_, cv_keypoints_, descriptors_);
        cout << "computed!" << endl;


        if (remove_duplicates_)
        {
//            removeDuplicates();
        }

        cout << descriptors_.cols << " per " << descriptors_.rows << endl;

        cout << "Found "<< *descriptors_.size.p << endl;

    }



    void removeDuplicates()
    {
        typedef std::pair<cv::KeyPoint, cv::Mat>  pairT;
        cout << "here" << endl;

        vector<pairT> pairs;
        for (int i = 0 ; i < cv_keypoints_.size() ; ++i)
        {
            pairT pair(cv_keypoints_[i], descriptors_.row(i));
            pairs.push_back(pair);
        }


        struct sort_pred {
            bool operator()(const pairT &a, const pairT &b)
            {
                if (a.first.pt.x == b.first.pt.x)
                {
                    return (a.first.pt.y > b.first.pt.y);
                }
                else
                {
                    return (a.first.pt.x > b.first.pt.x);
                }
            }
        };

        struct eq_pred {
            bool operator() (const pairT &a, const pairT & b)
            {
                return ((a.first.pt.x == b.first.pt.x) & (a.first.pt.y == b.first.pt.y));
            }
        };


        //now we sort the pair vector with a custom rule!
        std::sort(pairs.begin(), pairs.end(), sort_pred());

        //now remove duplicates
        pairs.erase(std::unique(pairs.begin(), pairs.end(), eq_pred()), pairs.end());

        cout << "SORTED AND EREASED" << endl;

        //put back descriptors and keypoints int the original places
        cv::Mat new_descs;
        vector<cv::KeyPoint> new_keys;

        for (pairT p: pairs)
        {
            new_descs.push_back(p.second);
            new_keys.push_back(p.first);
        }


        cv_keypoints_ = new_keys;
        descriptors_ = new_descs;


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
                //we force all de
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

    void setScale(float scale)
    {
        scale_= scale;
    }

    void setRemoveDuplicates(bool flag)
    {
        remove_duplicates_ = flag;
    }


    typedef shared_ptr<KeypointsExtractor> Ptr;

    string filename_;

    cv::Mat image_, image_low_, descriptors_;

    vector<cv::KeyPoint> cv_keypoints_;

    bool remove_duplicates_; //some alghorithms - ie SIFT - put out more than one keypoint with the same x,y (but with different scale for example)

    float scale_;
};

///
/// \brief The MatchesFilter class
///
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

///
/// \brief The MatchXY class
/// NOT USE THIS CLASS FOR OTHER PORPUSES THAN SORTING AND CLEANING FROM DUPLICATES!
/// it features an uncommon operator overloading that may lead to misunderstandings!
///
class MatchXY
{
public:
    MatchXY () {}

    MatchXY(float xa, float ya, float xb,float yb)
    {
        xA_ = xa;
        yA_ = ya;
        xB_ = xb;
        yB_ = yb;
    }

    float xA_, yA_, xB_, yB_;

};

const bool operator< (const MatchXY &a, const MatchXY &b )
{
    if (a.xA_ == b.xA_)
    {
        return (a.yA_ > b.yA_);
    }
    else
    {
        return (a.xA_ > b.xA_);
    }
}

const bool operator== (const MatchXY & a, const MatchXY &b)
{
    return ((a.xA_ == b.xA_) & (a.yA_ == b.yA_));
}


///
/// \brief The MatchesWriter class
///
class MatchesWriter
{
public:
    MatchesWriter() {filter_duplicates_ = true;}


    void setFilename(string fname)
    {
        filename_ = fname;
    }


    void setMatchesAndKeypoints(vector<Match>  matches, Keypoints keysA, Keypoints keysB)
    {
        keysA_ = keysA;
        keysB_ = keysB;
        matches_ = matches;

        toXYMatches();
        if (filter_duplicates_)
        {
            filterDuplicates();
        }
    }

    void toXYMatches()
    {
        for (Match m: matches_)
        {
            MatchXY mxy;
            float xa, ya, xb, yb;

            int ida, idb;

            ida = m.idA_;
            idb = m.idB_;


            Keypoint keya = *(keysA_.keypoints_.at(ida) );
            Keypoint keyb = *(keysB_.keypoints_.at(idb) );

            mxy.xA_ = keya.x_;
            mxy.yA_ = keya.y_;
            mxy.xB_ = keyb.x_;
            mxy.yB_ = keyb.y_;

            matches_xy_.push_back(mxy);

        }

    }

    void filterDuplicates()
    {
        // we should implement a better filter.
        // the problem here is that sift gives out more than a keypoint with a x,y coord couple.
        // we could keep (not the firs as we do here) but the match that exibit the lower distance in the feature space!

        //now we sort the pair vector with a custom rule!
        std::sort(matches_xy_.begin(), matches_xy_.end());

        //now remove duplicates
        matches_xy_.erase(std::unique(matches_xy_.begin(), matches_xy_.end()), matches_xy_.end());

    }

    int write()
    {
        stringstream s_stream;

        ofstream out(filename_.c_str());

        for (MatchXY m : matches_xy_)
        {

            s_stream.precision(6);

            s_stream << m.xA_  << " " << m.yA_ << " " << m.xB_ << " " << m.yB_ << endl;
        }

        out << s_stream.str();
        out.close();


        return 1;
    }



    string filename_ ;

    std::vector<Match>  matches_;
    vector<MatchXY> matches_xy_;
    Keypoints keysA_;
    Keypoints keysB_;

    bool filter_duplicates_;
};

#endif // KEYPOINT_EXTRACTION_H
