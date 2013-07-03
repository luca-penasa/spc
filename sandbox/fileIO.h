#ifndef __mmt__fileIO__
#define __mmt__fileIO__

#include <iostream>

#include <vector>

typedef struct {
    int id1, id2;
} KpMatch;

typedef struct {
    float x, y, scale, orient;
} Keypoint;

int ReadBinaryKeyFile
(const char* filename, std::vector<unsigned char>* descriptors, std::vector<Keypoint>* keypoints);

int WriteBinaryKeyFile
(const char* filename, std::vector< std::vector<float> >& descriptors, std::vector<Keypoint>& keypoints);

int WritePastisResultFile
(const char* filename, std::vector<KpMatch>& matches, std::vector<Keypoint>& kpTrain, std::vector<Keypoint>& kpQuery);

#endif
