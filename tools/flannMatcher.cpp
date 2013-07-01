/************************************************************
 * A sift features matcher based on Flann and pluggable     *
 * into the IGN photogrammetric pipeline.                   *
 *                                                          *
 * Author: Romain Janvier <romain.janvier*AT*hotmail.fr     *
 * Freely inspired by Noah Snavely's Bundler KeyMatcher     *
 * and Nghia Ho improvements.                               *
 *                                                          *
 * Licence: GPL-2+                                          *
 ************************************************************/

#include "fileIO.h"

#include <iostream>
#include <vector>

#include <flann/flann.hpp>

#include <time.h>

int main(int argc, const char *argv[])
{
    double ratio = 0.3;
    const char* fileTrain =  argv[1];
    std::vector<unsigned char> descsTrain;
    std::vector<Keypoint> kpTrain;

    const char* fileQuery = argv[2];
    std::vector<unsigned char> descsQuery;
    std::vector<Keypoint> kpQuery;

    const char* outFile = argv[3];

    std::vector<KpMatch> matches;

    /* Reading files */
    int numKeysTrain = ReadBinaryKeyFile(fileTrain, &descsTrain, &kpTrain);
    int numKeysQuery = ReadBinaryKeyFile(fileQuery, &descsQuery, &kpQuery);

    std::cout << "Data points " << numKeysTrain
       << " / Query points " << numKeysQuery << std::endl;
  
   /* convert unsigned char to float, as flann doesn't "support" unsigned chars */
   // FIXME: Not really DRY...
   // FIXME: Memory management: Del char vectors / use pointers ?
   std::vector<float> fDescsTrain(numKeysTrain * 128);
  
   for (int i = 0; i < numKeysTrain * 128; i++) {
       fDescsTrain[i] = descsTrain[i];
   }

   std::vector<float> 
fDescsQuery(numKeysQuery * 128);
   for (int i = 0; i < numKeysQuery * 128; i++) {
       fDescsQuery[i] = descsQuery[i];
   }
   
   /* Number of neighbors */
   int nn = 2;

   flann::Matrix<float> trainSet(&fDescsTrain[0], numKeysTrain, 128);
   flann::Index< flann::L2<float> > index(trainSet, flann::KDTreeIndexParams(4));
   index.buildIndex();
   /* Results */
   std::vector<int> indicesRes(numKeysQuery * nn);
   std::vector<float> distsRes(numKeysQuery * nn);
   /* Query */
   flann::Matrix<float> querySet(&fDescsQuery[0], numKeysQuery, 128);
   flann::Matrix<int> indices(&indicesRes[0], numKeysQuery, nn);
   flann::Matrix<float> dists(&distsRes[0], numKeysQuery, nn);
   std::cout << "searching... ";

    clock_t begin, end;

    begin = clock();    
   index.knnSearch(querySet, indices, dists, nn, flann::SearchParams(128));    
    end = clock();

    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;

    std::cout << "in " << time_spent << " s"  << std::endl;


   
   /* Filter candidates with dist ratio */
   for (int i = 0; i < numKeysQuery; i++) {
       if(distsRes[i*2]< distsRes[i*2 + 1] * ratio) {
           KpMatch kpm;
           kpm.id1 = indicesRes[i*2];
           kpm.id2 = i;
           matches.push_back(kpm);
       }
   }

   std::cout << "match(es): " << matches.size() << std::endl;
   
   /* write a result file */
   WritePastisResultFile(outFile, matches, kpTrain, kpQuery);
   /*delete [] &fDescsQuery;
   delete [] &fDescsTrain;
   delete [] querySet.ptr();
   delete [] trainSet.ptr();
   delete [] &kpTrain;
   delete [] &kpQuery;*/
   return 0;
}
