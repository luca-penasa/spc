#include "image_matches_mask.h"



void ImageMatchesMask::setFromMissingMatchFiles(string directory)
{
    if (images_names_.size() == 0)
        cout << "you must specify a list of images names for setting up the matrix in this way" << endl;
    else
    {
        directory += "/"; //ensure is ending this way
        for (int i = 0 ; i < images_names_.size(); ++i)
        {
            string namea = images_names_.at(i);
            string this_pict_dir_name = directory + "Pastis" + namea + "/";

            for (int j = 0 ; j < images_names_.size(); ++j)
            {
                string nameb = images_names_.at(j);
                if (i > j)
                {
                    string this_match_filename = this_pict_dir_name + nameb + ".txt";
                    if (!fexists(this_match_filename))
                    {
                        setElement(i, j, true);
                    }
                    else
                    {
                        setElement(i,j, false);
                    }
                }

            }
        }


    }
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

