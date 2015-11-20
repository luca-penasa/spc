#include "strings.h"
#include <boost/tokenizer.hpp>
namespace spc
{




std::vector<std::string> splitStringAtSeparator(const std::string &line, const std::string &separator)
{

    std::vector<std::string> out;

    boost::char_separator<char> sep(separator.c_str());
    boost::tokenizer<boost::char_separator<char>> tokens(line, sep);

    // Assign tokens to a string vector

    out.assign(tokens.begin(), tokens.end());

    return out;
}

std::vector<std::string> splitLines(const std::string &muliline_text)
{

    std::vector<std::string> out;
    std::istringstream iss(muliline_text);
    std::string line;

    while (std::getline(iss, line))
    {
        out.push_back(line);
    }

    return out;
}

} // end nspace
