#ifndef IMAGE_MATCHES_MASK_H
#define IMAGE_MATCHES_MASK_H

//STD
#include <vector>
//#include <string>

using namespace std;
///
/// \brief The ImageMatchesMask class
///
class ImageMatchesMask
{
public:
    ImageMatchesMask() {}

    void setNumberOfImages(size_t n);


    void setElement(int i, int j, bool val);


    bool getElement(int i, int j);


    void setUpperTriangularNoDiagonal();


    vector<bool> mask_;

    size_t n_images_;

};

#endif // IMAGE_MATCHES_MASK_H
