#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#include <boost/filesystem.hpp>
#include <spc/core/logging.h>
namespace spc
{
namespace fs
{

    typedef boost::filesystem::path Path;


    Path appendPostfix(const Path &inpath, const std::string &postfix , const std::string delimiter = "_")
    {
        if (postfix == "")
        {
            DLOG(WARNING) << "adding NO postfix to filename";
            return inpath;
        }

        std::string filename = inpath.stem().string();
        std::string newfilename = filename + delimiter + postfix + inpath.extension().string();

        return inpath.parent_path() / newfilename;

    }

    bool existsFile(const Path &filename)
    {
        return boost::filesystem::exists(filename);
    }

}// end namespace fs
} // end spc
#endif // FILESYSTEM_H
