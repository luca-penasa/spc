#include <sys/stat.h>


#include "matcher.h"
#include "flann_index.h"
#include "helpers.h"
#include "matches_filter.h"
#include "keypoints_extractor.h"
#include "io.h"

template <typename ScalarT>
Matcher<ScalarT>::Matcher() : image_matrix_(0)
{


    //ensure all the dirs are setted up
    cache_dir_ = "cache/tiepoints/";
    out_dir_ = "Homol";

    mkdir("cache/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    //make sure this dir exists
    mkdir(cache_dir_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    mkdir((cache_dir_ + "flann/").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    mkdir((cache_dir_ + "descriptors/").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);


}

template <typename ScalarT>
void Matcher<ScalarT>::setFilenames(vector<string> fnames)
{
    filenames_ = fnames;
    mask_.setNumberOfImages(fnames.size());
    mask_.setImageNames(filenames_);
    mask_.setFromMissingMatchFiles(out_dir_); //this is the default behavior
    image_matrix_ = ImageMatchesMatrix(fnames.size());
}

template <typename ScalarT>
void Matcher<ScalarT>::updateFinders()
{
    finders_.resize(filenames_.size());
#pragma omp parallel for
    for (int i = 0 ; i < filenames_.size(); ++i )
    {
        string fname = filenames_.at(i);

        //get the corresponding keypoints
        Keypoints::Ptr kpts = keypoints_.at(i);

        FlannIndexTypePtr finder (new FlannIndexType);
        finder->setInputKeypoints(kpts);

        //check if an index file exists!
        string index_name = cache_dir_ + "flann/" + fname + ".dat";

        if (fexists(index_name)) //load it!
        {
            finder->loadIndexFromFile(index_name);
            cout << "loading from " << index_name << endl;
        }
        else
        {
            finder->buildIndex();
            finder->saveIndexToFile(index_name);
        }

        finders_.at(i) = finder;
    }
}
template <typename ScalarT>
void Matcher<ScalarT>::updateMatches()
{

    image_matrix_.setNumberOfImages(filenames_.size());


#pragma omp parallel for
    for (int i = 0; i < filenames_.size(); i++)
    {
        for (int j = 0; j  < filenames_.size(); j++)
        {
            if (mask_.getElement(i, j)) //only if the mask requires it!
            {
                string ima = filenames_.at(i);
                string imb = filenames_.at(j);

                Keypoints::Ptr kps_a = keypoints_.at(i);
                Keypoints::Ptr kps_b = keypoints_.at(j);

                vector<vector<Match> > matches;

                if (kps_a->keypoints_.size() > kps_b->keypoints_.size())
                {
                    //this is the normal situation
                    FlannIndexTypePtr finder = finders_.at(i);
                    matches = finder->getMatchesMulti(kps_b, 2);
                }
                else //simply the inverse thing - we also need to tell the Match object the two ids are inverted!
                {
                    FlannIndexTypePtr finder = finders_.at(j);
                    matches = finder->getMatchesMulti(kps_a, 2);

                }

                vector<Match> good_matches;

                MatchesFilter filter;
                filter.setFilterType(MatchesFilter::FIRST_NEAREST);
                filter.setInputMatches(matches);
                filter.filter(good_matches, 0.2);

                cout << "IM A : " << ima << endl;
                cout << "IM B : " << imb << endl;
                cout << "Found " << good_matches.size() << " matches!" << endl;

                if (kps_a->keypoints_.size() > kps_b->keypoints_.size())
                    image_matrix_.setMatches(good_matches, i, j);
                else
                    image_matrix_.setMatches(good_matches, j, i);
            }


        }
    }
}

template <typename ScalarT>

void Matcher<ScalarT>::updateKeypoints()
{
    //for each filename we compute the keypoints with their desriptors
    keypoints_.resize(filenames_.size());

#pragma omp parallel for
    for (int i = 0; i < filenames_.size(); i++)
    {
        string file = filenames_.at(i);

        //check if the keypoints exist!
        //we need to also store in someway the type and the resolution at which they have been computed
        string desc_name = cache_dir_ + "descriptors/" + file + ".dat";

        Keypoints::Ptr keys;
        if (fexists(desc_name)) //load it!
        {
            cout << "loading keypoints from " << desc_name << endl;
            KeypointsReader reader(desc_name);
            reader.readHeader();
            reader.readKeypoints();
            keys = reader.getKeypoints();
        }
        else //build them
        {

            KeypointsExtractor extractor;
            extractor.setFilename(file);
            extractor.setScale(0.5);
            extractor.loadImage();
            extractor.compute();

            keys = extractor.getDescriptors();

            //and save them!
            KeypointsWriter w;
            w.setFilename(desc_name);
            w.setKeypoints(keys);
            w.write();

        }


        keypoints_.at(i) = keys;
    }
}

template <typename ScalarT>

void Matcher<ScalarT>::setCacheDir(string dir)
{
    cache_dir_ = dir;
}
template <typename ScalarT>

void Matcher<ScalarT>::setMatchingMask(ImageMatchesMask &mask)
{
    mask_ = mask;
}

template <typename ScalarT>
void Matcher<ScalarT>::writeOutHomolFiles()
{

    string main_dir_name = out_dir_ + "/";

    mkdir(main_dir_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);


    for (int i = 0; i < filenames_.size(); ++i)
    {
        string namea = filenames_.at(i);

        string this_pict_dir_name = main_dir_name + "Pastis" + namea + "/";
        mkdir(this_pict_dir_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);



        Keypoints::Ptr keya = keypoints_.at(i);

        for (int j =0 ; j < filenames_.size(); j++)
        {

            if ((i != j) & (mask_.getElement(i,j)))
            {

                string nameb = filenames_.at(j);

                cout << "WRITE " << namea << " vs " << nameb << endl;
                vector<Match> matches = image_matrix_.getMatches(i, j);


                Keypoints::Ptr keyb = keypoints_.at(j);


                string this_dat_filename = this_pict_dir_name + nameb + ".txt";
                MatchesWriter w;
                w.setFilename(this_dat_filename);
                w.setMatchesAndKeypoints(matches, *keya, *keyb);
                w.write();

            }
        }
    }
}


///// FORCED INSTANTATION

template class Matcher<unsigned char>;

