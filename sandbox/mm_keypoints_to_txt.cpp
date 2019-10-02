#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <flann/flann.hpp>

#include <sys/stat.h>

#include "fileIO.h"

using namespace std;



int main(int argc, char ** argv)
{
    //we pass a list of files containing keypoints
    vector<string> filenames;

    for (int i = 1; i < argc; ++i)
        filenames.push_back(argv[i]);

    vector<unsigned char> * descriptors = new vector<unsigned char>;
    vector<Keypoint> * keypoints = new vector<Keypoint>;

    ReadBinaryKeyFile(filenames.at(0).c_str(), descriptors, keypoints);

    for (int i = 0; i < keypoints->size(); ++i)
    {
        Keypoint kpt = keypoints->at(i);
        cout << i << " " << kpt.x << " " << kpt.y << " " << kpt.scale << " " << kpt.orient << "  ";
        for (int j = 0; j < 128; ++j)
        {
            int value =  (int) *(&(descriptors->at(i *128)) + j);
            cout << value << " " ;
        }

        cout << endl;
    }



    return 1;
}
