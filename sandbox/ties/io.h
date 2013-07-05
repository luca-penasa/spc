#ifndef IO_H
#define IO_H

//STD
#include <string>
#include <algorithm>

//LOCAL
#include "keypoints.h"

using namespace std;


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
            float distance = m.distance_;

            int ida, idb;

            ida = m.idA_;
            idb = m.idB_;


            Keypoint keya = *(keysA_.keypoints_.at(ida) );
            Keypoint keyb = *(keysB_.keypoints_.at(idb) );

            mxy.xA_ = keya.x_;
            mxy.yA_ = keya.y_;
            mxy.xB_ = keyb.x_;
            mxy.yB_ = keyb.y_;
            mxy.distance_ = distance;

            matches_xy_.push_back(mxy);

        }

    }

    void filterDuplicates()
    {
        //now we sort the pair vector with a custom rule!
        std::sort(matches_xy_.begin(), matches_xy_.end(), operator_minor_on_A);

        //now remove duplicates
        matches_xy_.erase(std::unique(matches_xy_.begin(), matches_xy_.end(), operator_equal_on_A), matches_xy_.end());

        //the same in the B-image side
        //now we sort the pair vector with a custom rule!
        std::sort(matches_xy_.begin(), matches_xy_.end(), operator_minor_on_B);

        //now remove duplicates
        matches_xy_.erase(std::unique(matches_xy_.begin(), matches_xy_.end(), operator_equal_on_B), matches_xy_.end());

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


#endif // IO_H
