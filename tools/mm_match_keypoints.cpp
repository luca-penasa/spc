#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <flann/flann.hpp>

#include <sys/stat.h>



using namespace std;


double ratio = 0.3;

///THE CODE HERE COMES IN PART FROM THE ONE FOUND IN GITHUB (MicMacTools repo)

typedef struct {
    int id1, id2;
} KpMatch;

typedef struct {
    float x, y, scale, orient;
} Keypoint;

typedef struct{
    float x1, y1, x2, y2;
} MatchXY;


string strip_extension(string fullname)
{
    int lastindex = fullname.find_last_of(".");
    string rawname = fullname.substr(0, lastindex);
    return rawname;
}

class corrMatrix
{
public:
    corrMatrix(vector<string> im_names)
    {
       int n_images = im_names.size();
       matches_xy.resize(n_images * n_images);
       stride = n_images;
       names = im_names;

    }

    void pushBackMatch(MatchXY & match, int i, int j)
    {
        int position = i * stride + j;
        matches_xy.at(position).push_back(match);
    }

    vector<MatchXY> getMatchesForImages(int i, int j)
    {
        int position = i * stride + j;
        return matches_xy.at(position);
    }

    string getNameOfImage(int i)
    {
        return names.at(i);
    }

private:
    vector<vector<MatchXY>> matches_xy;
    int stride;
    vector<string> names;
};

typedef std::vector<unsigned char> descriptor;

/* Read binary sift file */
int ReadBinaryKeyFile
(const string filename, std::vector<unsigned char> & descriptors, std::vector<Keypoint> & keypoints) {
    /***********************************************
    * NO check:                                    *
    * it's intended to be use in MicMac            *
    * SO WE TRUST Pastis and siftpp_tgi output     *
    * and we also assume users don't delete their  *
    * files                                        *
    ************************************************/
    int number, vectorLen;
    std::ifstream binaryFile(filename, std::ios::in|std::ios::binary);
    /* Read header: a number and the vector_lenght (obviously it is 128) */
    binaryFile.read(reinterpret_cast<char*>(&number), sizeof(int));
    binaryFile.read(reinterpret_cast<char*>(&vectorLen), sizeof(int));
    // TODO: if != 128: throw error / return 0
    descriptors.resize(number * vectorLen);
    keypoints.resize(number);

    for (int i = 0; i < number; ++i) {
        binaryFile.read( reinterpret_cast<char*>(&((keypoints)[i].x)), sizeof(float) );
        binaryFile.read( reinterpret_cast<char*>(&((keypoints)[i].y)), sizeof(float) );
        binaryFile.read( reinterpret_cast<char*>(&((keypoints)[i].scale)), sizeof(float) );
        binaryFile.read( reinterpret_cast<char*>(&((keypoints)[i].orient)), sizeof(float) );
        binaryFile.read( reinterpret_cast<char*>(&((descriptors)[i*128])), sizeof(char)*128 );
    }

    binaryFile.close();
    return number;
}

int WriteBinaryKeyFile
(const char* filename, std::vector< std::vector<float> >& descriptors, std::vector<Keypoint>& keypoints){
    std::ofstream binaryFile(filename, std::ios::out|std::ios::binary|std::ios::trunc);
    int sz = keypoints.size();
    int szDesc = 128;
    binaryFile.write((char*) &sz ,sizeof(int));
    binaryFile.write((char*) &szDesc, sizeof(int));

    for (int i = 0; i < sz; ++i) {
        binaryFile.write(reinterpret_cast<char*>(&keypoints[i].x), sizeof(float));
        binaryFile.write(reinterpret_cast<char*>(&keypoints[i].y), sizeof(float));
        binaryFile.write(reinterpret_cast<char*>(&keypoints[i].orient), sizeof(float));
        binaryFile.write(reinterpret_cast<char*>(&keypoints[i].scale), sizeof(float));
        for (int j = 0; j < 128; ++j) {
          unsigned char elem = 512.0f * descriptors[i][j];
          binaryFile.write(reinterpret_cast<char*>(&elem), sizeof(char));
        }
    }

    binaryFile.close();
    //TODO: handle error return value
    return 0;
}

int WritePastisResultFile
(string filename, std::vector<MatchXY>& matches){

    std::ofstream pastisResult(filename.c_str(), std::ios::out|std::ios::trunc);

    for (auto match: matches)
    {
        pastisResult << match.x1 << " " << match.y1 << " "
            << match.x2 << " " << match.y2 << std::endl;
    }
   pastisResult.close();
   //TODO: handle an error return value
   return 0;
}


int ReadAllBinaryKeyFiles
(vector<string> filenames, std::vector<int> &identifiers,  std::vector<unsigned char> & descriptors, std::vector<Keypoint> & keypoints)
{
    int curr_id = 0; //an id for each file so that filenames.at(id) returns the name

    for (auto fname :filenames)
    {
        std::cout << "Reading " << fname << " Assigned id " << curr_id <<  std::endl;
        vector<unsigned char> curr_descs;
        std::vector<Keypoint>  curr_kpoints;
        std::vector<int> curr_ids;


        ReadBinaryKeyFile(fname, curr_descs, curr_kpoints);

        int n_keys = curr_kpoints.size();

        curr_ids.assign(n_keys, curr_id++); //fill the current id vector

        //then append everithing
        std::copy (curr_descs.begin(), curr_descs.end(), std::back_inserter(descriptors));
        std::copy (curr_ids.begin(), curr_ids.end(), std::back_inserter(identifiers));
        std::copy (curr_kpoints.begin(), curr_kpoints.end(), std::back_inserter(keypoints));
    }

//    for (int i = 0 ;i < identifiers.size(); ++i)
//        std::cout << i << std::endl;

    return 1;
}



int main(int argc, char ** argv)
{
    //we pass a list of files containing keypoints
    vector<string> filenames;

    for (int i = 1; i < argc; ++i)
        filenames.push_back(argv[i]);

//    for (auto i : filenames)
//        std::cout << "Found File: " << i << std::endl;


    vector<int> ids;
    vector<unsigned char> descs;
    vector<Keypoint> keys;


    std::cout << "Loading all files in memory...: " << std::endl;
    ReadAllBinaryKeyFiles(filenames, ids, descs, keys);


    for (int i = 0 ; i < 10; ++i)
    {
        for (int j = 0; j < 128; ++j)
        {
            cout << (int) descs.at(i * 128 + j) << " ";

        }

        cout << endl;
    }

//    //print out some of them:
//    for (int i = 0; i < 6000; ++i )

//            {
//                std::cout << " kID: " << i << " Filename: " << filenames.at(ids.at(i));
//                std::cout << " id: " << ids.at(i) << std::endl;
//            }


    std::cout << "Found a total of " << keys.size() << " keypoints with descriptors" << std::endl;

    //reinterpret them as integers!
    std::vector<int> desc_float(descs.size());

    for (int i = 0; i < descs.size(); i++) {
        desc_float[i] = (int) descs[i];
//        std::cout << desc_float[i] << std::endl;
    }

    std::cout << "Building flann index..." << std::endl;
    //now build up a flann structure
    flann::Matrix<int> flann_descriptors(desc_float.data(), keys.size(), 128);
    flann::Index< flann::L2<int> > index(flann_descriptors, flann::KDTreeIndexParams(4));
    index.buildIndex();
    std::cout << "Built..." << std::endl;

    //Number of neighbors
    int nn = 5; // the first one is clearly itself! then we may have some autoreferring matches (should not!)
                // 5 should be enough

    //where to put results!
    std::vector<int> indices(keys.size() * nn);
    std::vector<float> distances(keys.size() * nn);
    std::vector<bool> use_mask;
    use_mask.assign(keys.size() * nn, false);

    //get them in a flann fashion
    flann::Matrix<int> flann_indices(&indices[0], keys.size(), nn);
    flann::Matrix<float> flann_dists(&distances[0], keys.size(), nn);

    std::cout << "Searching in the feature domain... " << std::endl;
    index.knnSearch(flann_descriptors, flann_indices, flann_dists, nn, flann::SearchParams(128));

    std::vector<KpMatch> matches;

    std::cout << "Clening out the results..." << std::endl;
    // now we need to filter aways the auto referring matches
    // actually we create a mask that will be used after
    for (int global_id1 = 0 ; global_id1 < keys.size(); ++global_id1)
    {

        vector<int> two_good_local_ids;
        vector<int> two_good_global_ids;

        int local_id1 = global_id1 * nn;

        for (int i = 1; i < nn ; ++i ) //on the number of required neighbors per key
        {
            int local_id2 = local_id1 + i;
            int global_id2 = indices.at(local_id2);



            if (ids.at(global_id1) != ids.at(global_id2))
            {
                two_good_global_ids.push_back(global_id2);
                two_good_local_ids.push_back(local_id2);

            }

            if (two_good_global_ids.size() == 2) //once we have 2 good is fine
            {
                // we filter them on another criteria
                if(distances[two_good_local_ids.at(0)] < distances[two_good_local_ids.at(1)] * ratio)
                {
                    KpMatch kpm;
                    kpm.id1 = two_good_global_ids.at(0); //we save here the two global ids... so we can know in which image it must go
                    kpm.id2 = two_good_global_ids.at(1);
                    matches.push_back(kpm);
                }
                break;
            }
        }
    }


    std::cout << "Found a total of " << matches.size() << " good matches" << std::endl;
    //now we need to sort out the different images

    corrMatrix matrix (filenames);


    for (auto kpoint : matches)
    {
        int i,j; //images identifiers
        i = ids.at(kpoint.id1);
        j = ids.at(kpoint.id2);

        //create the matrix structure of correspondences
        MatchXY match;
        float x1, y1, x2, y2;

        match.x1 = keys.at(kpoint.id1).x;
        match.y1 = keys.at(kpoint.id1).y;

        match.x2 = keys.at(kpoint.id2).x;
        match.y2 = keys.at(kpoint.id2).y;

        matrix.pushBackMatch(match, i, j); //push back this match for the given image!
    }

    for (int i = 0; i < filenames.size(); ++i)
    {
        string father_name = strip_extension(filenames.at(i));
        string dirname = "LBPp-Match-" + father_name;
        mkdir( dirname.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); //be sure the dir exists

        for (int j = 0; j < filenames.size(); ++j)
        {
            string son_name = strip_extension(filenames.at(j));

            //get all matches for this couple
            vector<MatchXY> couple_matches = matrix.getMatchesForImages(i, j);
            if (couple_matches.size() > 50)
            {
                cout << "Im1: "<< father_name << endl;
                cout << "Im2: "<< son_name << endl;
                cout << "Nmatches: " << couple_matches.size() << endl;



            }
            if (father_name != son_name)
            {
                string result_name = dirname + "/" + son_name + ".result";
                WritePastisResultFile(result_name, couple_matches);
            }

        }
    }


    return 1;
}
