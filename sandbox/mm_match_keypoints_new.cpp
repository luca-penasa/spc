#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>


#include <flann/flann.hpp>

#include <sys/stat.h>

#include "keypoint_extraction.h"
#include "mm_matches.h"


using namespace std;





bool fexists(string filename)
{
    FILE * file = fopen(filename.c_str(), "r");
    return file;
}







int WritePastisResultFile (string filename, Matches::Ptr matches, Keypoints::Ptr Akeys, Keypoints::Ptr Bkeys)
{

    std::ofstream pastisResult(filename.c_str(), std::ios::out|std::ios::trunc);

    for (Match::Ptr match: matches->matches_)
    {
        //        cout << "qua" << endl;
        int ida = match->idA_;
        int idb = match->idB_;

        Keypoint::Ptr aKey = Akeys->getKeypoint(ida);
        Keypoint::Ptr bKey = Bkeys->getKeypoint(idb);

        pastisResult << aKey->x_ << " " << aKey->y_ << " "
                     << bKey->x_ << " " << bKey->y_ << endl;
    }
    pastisResult.close();

    return 1;
}


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
        index_->knnSearch(query_flann, flann_ids, flann_dists, nn, flann::SearchParams(128));

        end = clock();
        double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
        cout << time_spent << endl;


        vector<vector<Match> > matches;


        //filtering!
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

    Matches::Ptr getMatches(Keypoints::Ptr points, float ratio = 0.3)
    {
        int nn = 2; //for each query the first two!
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
        index_->knnSearch(query_flann, flann_ids, flann_dists, nn, flann::SearchParams(128));

        end = clock();
        double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
        cout << time_spent << endl;


        Matches::Ptr matches (new Matches);


        //filtering!
        for (int i = 0; i < points->getNumberOfKeypoints(); ++i)
        {
            if (dists.at(i*2) < dists.at(i*2  + 1) * ratio)
            {
                Match::Ptr match(new Match(ids.at(i*2), i) );
                matches->matches_.push_back(match);
            }

        }

        cout << "Got " << matches->matches_.size() << " matches!" << endl;
        return matches;
    }


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

    Matcher() {}

    void setFilenames(vector<string> fnames)
    {
        filenames_ = fnames;
    }

    void updateFinders()
    {
        //        #pragma omp parallel for
        for (int i = 0 ; i < filenames_.size(); ++i )
        {
            string fname = filenames_.at(i);

            KeypointsReader reader(fname);
            reader.read();

            Keypoints::Ptr kpts = reader.getKeypoints();
            keypoints_.push_back(kpts);

            FlannIndexTypePtr finder (new FlannIndexType);
            finder->setInputKeypoints(kpts);

            //check if an index file exists!
            string index_name = fname + ".flannindex";

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

            finders_.push_back(finder);
        }
    }

    void updateMatches()
    {
#pragma omp parallel for
        for (int i = 0; i < filenames_.size(); ++i)
        {
            FlannIndexTypePtr finder = finders_.at(i);


            for (int j = 0; j  < i; ++j)
            {



                FlannIndexTypePtr finder_b = finders_.at(j);
                Keypoints::Ptr kps = keypoints_.at(j);


                Matches::Ptr matches = finder->getMatches(kps);
                cout << "IM A : " << filenames_.at(i) << endl;
                cout << "IM B : " << filenames_.at(j) << endl;
                cout << "Found " << matches->matches_.size() << " matches!" << endl;
            }
        }
    }

    vector<string> filenames_;

    vector< typename FlannIndexType::Ptr > finders_;

    vector<Keypoints::Ptr> keypoints_;


};

#include <time.h>

int main(int argc, char ** argv)
{
    //we pass a list of files containing keypoints
    vector<string> filenames;

    for (int i = 1; i < argc; ++i)
        filenames.push_back(argv[i]);

//    KeypointsReader reader(filenames.at(0));
//    KeypointsReader reader2(filenames.at(1));

//    reader.read();
//    reader2.read();


    KeypointsExtractor extractor;
    extractor.scale_ = 0.5;
    extractor.setFilename(filenames.at(0));
    extractor.loadImage();
    extractor.compute();
    Keypoints::Ptr keys = extractor.getDescriptors();

    KeypointsExtractor extractor2;
    extractor2.scale_ = 0.5;
    extractor2.setFilename(filenames.at(1));
    extractor2.loadImage();
    extractor2.compute();
    Keypoints::Ptr keys2 = extractor2.getDescriptors();

//    Keypoints pt = *keys;
//    for (int i = 0;  pt.getNumberOfKeypoints(); ++i)
//    {
//        for (int j = 0; j < 128; ++j)
//        {
//            cout << (int)pt.descriptors_.at(i*128+j) << " ";
//        }
//        cout << endl;

//    }

//    Keypoints::Ptr keys = reader.getKeypoints();
//    Keypoints::Ptr keys2 = reader2.getKeypoints();




    FlannIndex<unsigned char> index;
    index.setInputKeypoints(keys);
    index.buildIndex();

    vector<vector<Match> > matches  = index.getMatchesMulti(keys2, 2);

    MatchesFilter filter;
    filter.setFilterType(MatchesFilter::FIRST_NEAREST);
    filter.setInputMatches(matches);

    vector<Match> good_matches;

    filter.filter(good_matches, 0.1);

    cout << "Found " << good_matches.size() << " good matches" << endl;

    /////////////////////////////// ONE TEST
//    string filename = filenames.at(0);
//            string filename2 = filenames.at(1);

//    KeypointsReader reader(filename);
//            KeypointsReader reader2(filename2);


//    reader.read();

//    reader2.read();




//    Keypoints::Ptr kpoints = reader.getKeypoints();

//    vector<float> data = kpoints->getAllDescriptorAsStdVector<float>();

//    flann::Matrix<float> trainSet(&(data[0]), kpoints->keypoints_.size(), 128);
//    flann::Index< flann::L2<float> > index(trainSet, flann::KDTreeIndexParams(4));
//    index.buildIndex();

//    //        Keypoints::Ptr kpoints2 = reader2.getKeypoints();

//    vector<float> v = kpoints->getAllDescriptorAsStdVector<float>();
//    //        for (float c:v)
//    //            cout << c << endl;


//    //        cout << "building index" << endl;
//    FlannIndex<float> index2;
//    index2.setInputKeypoints(kpoints);
//    index2.buildIndex();

    //        cout << "done" << endl;


    //        Matches::Ptr match = index->getMatches(kpoints2);

    //        //print out some matches
    //        for (Match::Ptr m : match->matches_)
    //        {
    //            std::cout << m->idA_ << " " << m->idB_ << endl;
    //        }

    //        WritePastisResultFile("prova.txt", match, kpoints, kpoints2);

    ////////////////////////// ONE TEST



//    Matcher<unsigned char>  matcher;
//    matcher.setFilenames(filenames);
//    matcher.updateFinders();

//    matcher.updateMatches();


    //    vector<int> ids;
    //    vector<unsigned char> descs;
    //    vector<Keypoint> keys;


    //    std::cout << "Loading all files in memory...: " << std::endl;
    //    ReadAllBinaryKeyFiles(filenames, ids, descs, keys);

    ////    //print out some of them:
    ////    for (int i = 0; i < 6000; ++i )

    ////            {
    ////                std::cout << " kID: " << i << " Filename: " << filenames.at(ids.at(i));
    ////                std::cout << " id: " << ids.at(i) << std::endl;
    ////            }


    //    std::cout << "Found a total of " << keys.size() << " keypoints with descriptors" << std::endl;

    //    //reinterpret them as integers!
    //    std::vector<int> desc_float(descs.size());

    //    for (int i = 0; i < descs.size(); i++) {
    //        desc_float[i] = (int) descs[i];
    ////        std::cout << desc_float[i] << std::endl;
    //    }

    //    std::cout << "Building flann index..." << std::endl;
    //    //now build up a flann structure
    //    flann::Matrix<int> flann_descriptors(desc_float.data(), keys.size(), 128);
    //    flann::Index< flann::L2<int> > index(flann_descriptors, flann::KDTreeIndexParams(4));
    //    index.buildIndex();
    //    std::cout << "Built..." << std::endl;

    //    //Number of neighbors
    //    int nn = 5; // the first one is clearly itself! then we may have some autoreferring matches (should not!)
    //                // 5 should be enough

    //    //where to put results!
    //    std::vector<int> indices(keys.size() * nn);
    //    std::vector<float> distances(keys.size() * nn);
    //    std::vector<bool> use_mask;
    //    use_mask.assign(keys.size() * nn, false);

    //    //get them in a flann fashion
    //    flann::Matrix<int> flann_indices(&indices[0], keys.size(), nn);
    //    flann::Matrix<float> flann_dists(&distances[0], keys.size(), nn);

    //    std::cout << "Searching in the feature domain... " << std::endl;
    //    index.knnSearch(flann_descriptors, flann_indices, flann_dists, nn, flann::SearchParams(128));

    //    std::vector<KpMatch> matches;

    //    std::cout << "Clening out the results..." << std::endl;
    //    // now we need to filter aways the auto referring matches
    //    // actually we create a mask that will be used after
    //    for (int global_id1 = 0 ; global_id1 < keys.size(); ++global_id1)
    //    {

    //        vector<int> two_good_local_ids;
    //        vector<int> two_good_global_ids;

    //        int local_id1 = global_id1 * nn;

    //        for (int i = 1; i < nn ; ++i ) //on the number of required neighbors per key
    //        {
    //            int local_id2 = local_id1 + i;
    //            int global_id2 = indices.at(local_id2);



    //            if (ids.at(global_id1) != ids.at(global_id2))
    //            {
    //                two_good_global_ids.push_back(global_id2);
    //                two_good_local_ids.push_back(local_id2);

    //            }

    //            if (two_good_global_ids.size() == 2) //once we have 2 good is fine
    //            {
    //                // we filter them on another criteria
    //                if(distances[two_good_local_ids.at(0)] < distances[two_good_local_ids.at(1)] * ratio)
    //                {
    //                    KpMatch kpm;
    //                    kpm.id1 = two_good_global_ids.at(0); //we save here the two global ids... so we can know in which image it must go
    //                    kpm.id2 = two_good_global_ids.at(1);
    //                    matches.push_back(kpm);
    //                }
    //                break;
    //            }
    //        }
    //    }


    //    std::cout << "Found a total of " << matches.size() << " good matches" << std::endl;
    //    //now we need to sort out the different images

    //    corrMatrix matrix (filenames);


    //    for (auto kpoint : matches)
    //    {
    //        int i,j; //images identifiers
    //        i = ids.at(kpoint.id1);
    //        j = ids.at(kpoint.id2);

    //        //create the matrix structure of correspondences
    //        MatchXY match;
    //        float x1, y1, x2, y2;

    //        match.x1 = keys.at(kpoint.id1).x;
    //        match.y1 = keys.at(kpoint.id1).y;

    //        match.x2 = keys.at(kpoint.id2).x;
    //        match.y2 = keys.at(kpoint.id2).y;

    //        matrix.pushBackMatch(match, i, j); //push back this match for the given image!
    //    }

    //    for (int i = 0; i < filenames.size(); ++i)
    //    {
    //        string father_name = strip_extension(filenames.at(i));
    //        string dirname = "LBPp-Match-" + father_name;
    //        mkdir( dirname.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); //be sure the dir exists

    //        for (int j = 0; j < filenames.size(); ++j)
    //        {
    //            string son_name = strip_extension(filenames.at(j));

    //            //get all matches for this couple
    //            vector<MatchXY> couple_matches = matrix.getMatchesForImages(i, j);
    //            if (couple_matches.size() > 50)
    //            {
    //                cout << "Im1: "<< father_name << endl;
    //                cout << "Im2: "<< son_name << endl;
    //                cout << "Nmatches: " << couple_matches.size() << endl;



    //            }
    //            if (father_name != son_name)
    //            {
    //                string result_name = dirname + "/" + son_name + ".result";
    //               WritePastisResultFile(result_name, couple_matches);
    //            }

    //        }
    //    }


    //    return 1;
}
