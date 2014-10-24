#include "strings.h"
#include <boost/tokenizer.hpp>
namespace spc
{
std::string stripExtension(const std::string &file_name)
{

    for (int i = file_name.length(); i > 0; --i) {
        if (file_name[i] == '.') {
            return file_name.substr(0, i);
        }
    }
    return file_name;
}

std::string getBaseFileName(const std::string &file_name)
{
    std::string file_name2
        = stripExtension(file_name); // remove extension if there is any
    for (int i = 0; i < file_name2.length(); ++i) {
        if (file_name2[i] == '_') {
            return file_name2.substr(0, i);
        }
    }
    return file_name2;
}

std::string addSubscript(const std::string &file_name, const std::string &text)
{
    std::string without_extension = stripExtension(file_name);
    std::string new_file_name = without_extension + text;
    return new_file_name;
}

std::vector<std::string> splitStringAtSeparator(const std::string &line, const std::string &separator)
{

    std::vector<std::string> out;

    boost::char_separator<char> sep(separator.c_str());
    boost::tokenizer<boost::char_separator<char>> tokens(line, sep);

    // Assign tokens to a string vector

    out.assign(tokens.begin(), tokens.end());

    return out;
}

} // end nspace
