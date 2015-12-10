#include "regex.h"

#include <spc/core/logging.h>
#include <boost/regex.hpp>

#include <boost/filesystem.hpp>

std::vector<std::string> spc::filter_filenames_regex(const std::vector<std::string> &in_list, const std::string &reg_string)
{
    using  pairT = std::pair<std::string, std::string> ;
    std::vector<pairT> fnames;

    for (auto f : in_list)
        fnames.push_back(pairT( f,   boost::filesystem::path(f).filename().string()));

    std::vector<std::string> cleaned;
    boost::regex regex  (reg_string);

    for (auto s: fnames)
    {
        if (boost::regex_match(s.second,regex))
            cleaned.push_back(s.first);
    }
    return cleaned;
}


