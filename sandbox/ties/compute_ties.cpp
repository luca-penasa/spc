#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>


#include <flann/flann.hpp>

#include <sys/stat.h>

#include "keypoints.h"
#include "matches.h"
#include "matcher.h"
#include "io.h"
#include "matches_filter.h"
#include "keypoints_extractor.h"

#include <time.h>


using namespace std;


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
