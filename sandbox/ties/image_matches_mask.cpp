#include "image_matches_mask.h"



void ImageMatchesMask::setNumberOfImages(size_t n)
{
    n_images_ = n;
    mask_.assign(n_images_ * n_images_, false);
}

void ImageMatchesMask::setElement(int i, int j, bool val)
{
    mask_.at(i*n_images_ + j) = val;
}

bool ImageMatchesMask::getElement(int i, int j)
{
    return mask_.at(i*n_images_ + j);
}

void ImageMatchesMask::setUpperTriangularNoDiagonal()
{
    for (int i = 0; i < n_images_ ; ++i)
        for (int j = 0; j < i; ++j)
            mask_.at(i*n_images_ + j) = true;


}

