#ifndef IMAGE_MATCHES_MASK_H
#define IMAGE_MATCHES_MASK_H

//STD
#include <vector>
#include <string>
#include <iostream>

#include "helpers.h"

using namespace std;
///
/// \brief The ImageMatchesMask class
///
class ImageMatchesMask
{
public:
    ImageMatchesMask() {}

    void setNumberOfImages(size_t n)
    {
        n_images_ = n;
        mask_.assign(n_images_ * n_images_, false);
    }


    void setImageNames(vector<string> names)
    {
        images_names_ = names;
    }


    void setElement(int i, int j, bool val);


    bool getElement(int i, int j);


    void setUpperTriangularNoDiagonal();

    void setFromMissingMatchFiles(string directory);


private:
    vector<bool> mask_;

    size_t n_images_;

    vector<string> images_names_;

};

#endif // IMAGE_MATCHES_MASK_H
