#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>


#include <flann/flann.hpp>

#include <sys/stat.h>

#include "keypoints.h"
#include "matches.h"
#include <time.h>


using namespace std;





bool fexists(string filename)
{
    FILE * file = fopen(filename.c_str(), "r");
    return file;
}







//int WritePastisResultFile (string filename, Matches::Ptr matches, Keypoints::Ptr Akeys, Keypoints::Ptr Bkeys)
//{

//    std::ofstream pastisResult(filename.c_str(), std::ios::out|std::ios::trunc);

//    for (Match::Ptr match: matches->matches_)
//    {
//        //        cout << "qua" << endl;
//        int ida = match->idA_;
//        int idb = match->idB_;

//        Keypoint::Ptr aKey = Akeys->getKeypoint(ida);
//        Keypoint::Ptr bKey = Bkeys->getKeypoint(idb);

//        pastisResult << aKey->x_ << " " << aKey->y_ << " "
//                     << bKey->x_ << " " << bKey->y_ << endl;
//    }
//    pastisResult.close();

//    return 1;
//}


string strip_extension(string fullname)
{
    int lastindex = fullname.find_last_of(".");
    string rawname = fullname.substr(0, lastindex);
    return rawname;
}

template <typename ScalarT>
class FlannIndex
{
public:

    typedef typename flann::L2<ScalarT> flannDistanceType;
    typedef flann::Index<flannDistanceType > flannIndexType;
    typedef shared_ptr<flannIndexType> flannIndexTypePtr;

    typedef std::shared_ptr<FlannIndex> Ptr;
public:
    FlannIndex() {}

    void buildIndex()
    {
        cout << "building index..." << endl;
        index_->buildIndex();
        cout << "built!" << endl;
    }

    void setInputKeypoints(Keypoints::Ptr kpoints)
    {
        keypoints_ = kpoints;

        desc_as_v_ = keypoints_->getAllDescriptorAsStdVector<ScalarT>();
        flann_descriptors_ = flann::Matrix<ScalarT>(&desc_as_v_[0], keypoints_->getNumberOfKeypoints(), keypoints_->getSizeOfFeature());

        flann::KDTreeIndexParams params(4);
        //        flann::AutotunedIndexParams params;


        index_ = flannIndexTypePtr( new flannIndexType(flann_descriptors_, params) );
        //        index_ = flannIndexTypePtr( new flannIndexType(flann_descriptors_, flann::AutotunedIndexParams()) );
    }

    vector<vector<Match> > getMatchesMulti(Keypoints::Ptr points, int nn)
    {
        last_query_ = points->getAllDescriptorAsStdVector<ScalarT>();
        flann::Matrix<ScalarT> query_flann (&last_query_[0], points->getNumberOfKeypoints(), points->getSizeOfFeature());

        vector<int> ids(points->getNumberOfKeypoints() * nn);
        vector<float> dists(points->getNumberOfKeypoints() * nn);

        flann::Matrix<int> flann_ids (ids.data(), points->getNumberOfKeypoints(), nn);
        flann::Matrix<float> flann_dists(dists.data(), points->getNumberOfKeypoints(), nn);


        cout << "data points: " << this->keypoints_->keypoints_.size() ;
        cout << " query points: " << points->getNumberOfKeypoints() << endl;


        clock_t begin, end;
        begin = clock();

        //        spars.cores = 3;
        index_->knnSearch(query_flann, flann_ids, flann_dists, nn, flann::SearchParams());

        end = clock();
        double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
        cout << time_spent << endl;


        vector<vector<Match> > matches;


        //putting in matches
        for (int i = 0; i < points->getNumberOfKeypoints() ; ++i)
        {
            vector<Match> this_matches;
            for (int j=0; j < nn; ++j)
            {
                Match match(ids.at(i*nn+j), i);
                match.distance_ = dists.at(i*nn + j);
                this_matches.push_back(match);
            }

            matches.push_back(this_matches);


        }

        cout << "Got " << matches.size() << " matches!" << endl;
        return matches;
    }

//    Matches::Ptr getMatches(Keypoints::Ptr points, float ratio = 0.3)
//    {
//        int nn = 2; //for each query the first two!
//        last_query_ = points->getAllDescriptorAsStdVector<ScalarT>();
//        flann::Matrix<ScalarT> query_flann (&last_query_[0], points->getNumberOfKeypoints(), points->getSizeOfFeature());

//        vector<int> ids(points->getNumberOfKeypoints() * nn);
//        vector<float> dists(points->getNumberOfKeypoints() * nn);

//        flann::Matrix<int> flann_ids (ids.data(), points->getNumberOfKeypoints(), nn);
//        flann::Matrix<float> flann_dists(dists.data(), points->getNumberOfKeypoints(), nn);


//        cout << "data points: " << this->keypoints_->keypoints_.size() ;
//        cout << " query points: " << points->getNumberOfKeypoints() << endl;


//        clock_t begin, end;
//        begin = clock();

//        //        spars.cores = 3;
//        index_->knnSearch(query_flann, flann_ids, flann_dists, nn, flann::SearchParams(128));

//        end = clock();
//        double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
//        cout << time_spent << endl;


//        Matches::Ptr matches (new Matches);


//        //filtering!
//        for (int i = 0; i < points->getNumberOfKeypoints(); ++i)
//        {
//            if (dists.at(i*2) < dists.at(i*2  + 1) * ratio)
//            {
//                Match::Ptr match(new Match(ids.at(i*2), i) );
//                matches->matches_.push_back(match);
//            }

//        }

//        cout << "Got " << matches->matches_.size() << " matches!" << endl;
//        return matches;
//    }


    void saveIndexToFile(string filename)
    {
        index_->save(filename);
    }

    void loadIndexFromFile(string filename)
    {
        index_ = flannIndexTypePtr ( new flannIndexType (flann_descriptors_, flann::SavedIndexParams(filename)) );
    }


    Keypoints::Ptr getTrainKeys()
    {
        return keypoints_;
    }

    flannIndexTypePtr index_;
    Keypoints::Ptr keypoints_;

    flann::Matrix<ScalarT> flann_descriptors_;

    vector<ScalarT> desc_as_v_;
    vector<ScalarT> last_query_;

    vector<vector<Keypoint> > matches_;

};

//template class FlannIndex<unsigned char>;
//template class FlannIndex<float>;


template <typename ScalarT>
class Matcher
{
public:

    typedef  FlannIndex<ScalarT> FlannIndexType;
    typedef typename FlannIndex<ScalarT>::Ptr FlannIndexTypePtr;

    Matcher() : image_matrix_(0)
    {


        //ensure all the dirs are setted up
        cache_dir_ = "cache/tiepoints/";

        mkdir("cache/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        //make sure this dir exists
        mkdir(cache_dir_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        mkdir((cache_dir_ + "flann/").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        mkdir((cache_dir_ + "descriptors/").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);


    }

    void setFilenames(vector<string> fnames)
    {
        filenames_ = fnames;
        mask_.setNumberOfImages(fnames.size());
        mask_.setUpperTriangularNoDiagonal(); //this is the default behavior
        image_matrix_ = ImageMatchesMatrix(fnames.size());
    }

    void updateFinders()
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

    void updateMatches()
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
//                        for (vector<Match> &match_v: matches)
//                            for (Match &m: match_v)
//                                m.switchInversion();
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

    void updateKeypoints()
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
                extractor.setScale(0.3);
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


    void setCacheDir(string dir)
    {
        cache_dir_ = dir;
    }

    void setMatchingMask(ImageMatchesMask &mask)
    {
        mask_ = mask;
    }


    void writeOutHomolFiles(string main_dir_name)
    {

        main_dir_name += "/" ; //be sure it ends like that

        mkdir(main_dir_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);


        for (int i = 0; i < filenames_.size(); ++i)
        {
            string namea = filenames_.at(i);

            string this_pict_dir_name = main_dir_name + "Pastis" + namea + "/";
            mkdir(this_pict_dir_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);



            Keypoints::Ptr keya = keypoints_.at(i);

            for (int j =0 ; j < filenames_.size(); j++)
            {

                if (i != j)
                {

                    string nameb = filenames_.at(j);

                    cout << "WRITE " << namea << " vs " << nameb << endl;
                    vector<Match> matches = image_matrix_.getMatches(i, j);

                    if (matches.size() != 0)
                    {
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
    }


    vector<string> filenames_;

    vector< typename FlannIndexType::Ptr > finders_;

    vector<Keypoints::Ptr> keypoints_;

    ImageMatchesMatrix image_matrix_;

    string cache_dir_;

    ImageMatchesMask mask_;




};


///
/// \brief test_create_save_load_flann
/// \param filenames
/// \return
///
int test_create_save_load_flann(vector<string> filenames)
{
      string fname = filenames.at(0);
      string flannName = fname + ".flann";
      string keysName = fname + ".keys";

      //compute keys
      KeypointsExtractor extractor;
      extractor.setFilename(fname);
      extractor.loadImage();
      extractor.compute();
      Keypoints::Ptr keys = extractor.getDescriptors();

      //save keys
      KeypointsWriter w;
      w.setFilename(keysName);
      w.setKeypoints(keys);
      w.write();


      // create index and save it
      FlannIndex<unsigned char> flann;
      flann.setInputKeypoints(keys);
      flann.buildIndex();
      flann.saveIndexToFile(flannName);

      //now realod keys
      KeypointsReader r(keysName);
      r.readHeader();
      r.readKeypoints();
      Keypoints::Ptr reloaded_keys = r.getKeypoints();

      //try to setup w flann with saved things
      FlannIndex<unsigned char> newflann;
      newflann.setInputKeypoints(reloaded_keys);
      newflann.loadIndexFromFile(flannName);





}



///
/// \brief test_simple_match_2_images_and_save A SIMPLE TEST CASE OF MY CLASSES
/// \param filenames
/// \return
///
int test_simple_match_2_images_and_save(vector<string> filenames)
{

    KeypointsExtractor extractor;
    extractor.scale_ = 0.5;
    extractor.setFilename(filenames.at(0));
    extractor.loadImage();
    extractor.compute();
    Keypoints::Ptr keys = extractor.getDescriptors();

    KeypointsWriter w;
    w.setFilename("k1.dat");
    w.setKeypoints(keys);
    w.write();

    //try reloading
    KeypointsReader r("k1.dat");
    r.read();

    keys = r.getKeypoints();


    KeypointsExtractor extractor2;
    extractor2.scale_ = 0.5;
    extractor2.setFilename(filenames.at(1));
    extractor2.loadImage();
    extractor2.compute();
    Keypoints::Ptr keys2 = extractor2.getDescriptors();

    w.setFilename("k2.dat");
    w.setKeypoints(keys2);
    w.write();

    KeypointsReader r2("k2.dat");
    r2.read();

    keys2 = r2.getKeypoints();

    FlannIndex<unsigned char> index;
    index.setInputKeypoints(keys);
    index.buildIndex();

    vector<vector<Match> > matches  = index.getMatchesMulti(keys2, 2);

    MatchesFilter filter;
    filter.setFilterType(MatchesFilter::FIRST_NEAREST);
    filter.setInputMatches(matches);

    std::shared_ptr<vector<Match> > good_matches(new  vector<Match>);

    filter.filter(*good_matches, 0.2);

    //try to write out the good matches

    MatchesWriter writer;
    writer.setFilename("matches.txt");
    writer.setMatchesAndKeypoints(*good_matches, *keys, *keys2);
    writer.write();

    cout << "Found " << good_matches->size() << " good matches" << endl;

    return 1;
}


int main(int argc, char ** argv)
{
    //we pass a list of files containing keypoints
    vector<string> filenames;

    for (int i = 1; i < argc; ++i)
        filenames.push_back(argv[i]);


    //// TESTS
//    test_simple_match_2_images_and_save(filenames);

//    test_create_save_load_flann(filenames);

    ///////////////////////////////////////////////////

    //// ACTUAL CODE
    Matcher<unsigned char> matcher;
    matcher.setFilenames(filenames);
    matcher.updateKeypoints();
    matcher.updateFinders();
    matcher.updateMatches();
    matcher.writeOutHomolFiles(string("Homol_my"));


        return 1;
}
