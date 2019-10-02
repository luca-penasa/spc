#include <fstream>

#include "fileIO.h"
/* Read binary sift file */
int ReadBinaryKeyFile
(const char* filename, std::vector<unsigned char>* descriptors, std::vector<Keypoint>* keypoints) {
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
    descriptors->resize(number * vectorLen);
    keypoints->resize(number);
   
    for (int i = 0; i < number; ++i) {
        binaryFile.read(reinterpret_cast<char*>(&((*keypoints)[i].x)), sizeof(float));
        binaryFile.read(reinterpret_cast<char*>(&((*keypoints)[i].y)), sizeof(float));
        binaryFile.read(reinterpret_cast<char*>(&((*keypoints)[i].scale)), sizeof(float));
        binaryFile.read(reinterpret_cast<char*>(&((*keypoints)[i].orient)), sizeof(float));
        binaryFile.read(reinterpret_cast<char*>(&((*descriptors)[i*128])), sizeof(char)*128);
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
(const char* filename, std::vector<KpMatch>& matches, std::vector<Keypoint>& kpTrain, std::vector<Keypoint>& kpQuery){
    std::ofstream pastisResult(filename, std::ios::out|std::ios::trunc);
    for (int i = 0; i < matches.size(); i++) {
        pastisResult << kpTrain[matches[i].id1].x << " " << kpTrain[matches[i].id1].y << " "
            << kpQuery[matches[i].id2].x << " " << kpQuery[matches[i].id2].y << std::endl; 
    }
   pastisResult.close();
   //TODO: handle an error return value
   return 0;
}
 
