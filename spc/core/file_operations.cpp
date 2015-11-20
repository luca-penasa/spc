#include "file_operations.h"

#include <spc/core/logging.h>

#include <spc/core/regex.h>

#include <boost/filesystem.hpp>

#include <spc/core/timing.h>

#ifdef _WIN32
#include <direct.h>
#define getcwd _getcwd // stupid MSFT "deprecation" warning
//#else
//    #include <unistd.h>
#endif

namespace spc {

std::string getBaseFileName(const std::string& file_name)
{
    std::string file_name2 = stripExtension(file_name); // remove extension if there is any
    for (int i = 0; i < file_name2.length(); ++i) {
        if (file_name2[i] == '_') {
            return file_name2.substr(0, i);
        }
    }
    return file_name2;
}

std::string concatenateFilenameWithSeparator(const std::string& file_name,
    const std::string& text,
    const std::string& sep /*= "_"*/)
{
    // strip extension if any
    std::string without_extension = stripExtension(file_name);
    std::string new_file_name = without_extension + sep + text;
    return new_file_name;
}

std::string stripExtension(const std::string& file_name)
{
    for (int i = file_name.length(); i > 0; --i) {
        if (file_name[i] == '.') {
            return file_name.substr(0, i);
        }
    }

    // if no "." were found just return the full name
    return file_name;
}

std::string getExtension(const std::string& file_name)
{
    for (int i = file_name.length(); i > 0; --i) {
        if (file_name[i] == '.') {
            return file_name.substr(i, file_name.size());
        }
    }

    // if no "." were found just return an empty string
    return std::string();
}

bool fileExists(const std::string& fname)
{
    return boost::filesystem::exists(fname);
}

std::string getCurrentDirectory()
{
    char buffer[10000];
    char* answer = getcwd(buffer, sizeof(buffer));
    std::string s_cwd;
    if (answer)
        s_cwd = answer;
    return s_cwd;
}

std::vector<std::string> list_files(const std::string& dir_name)
{
    std::vector<std::string> list;

    // list all files in current directory.
    // You could put any file path in here, e.g. "/home/me/mwah" to list that
    // directory
    boost::filesystem::path p(dir_name);
    boost::filesystem::directory_iterator end_itr;
    // cycle through the directory
    for (boost::filesystem::directory_iterator itr(p); itr != end_itr; ++itr) {
        // If it's not a directory, list it. If you want to list directories too,
        // just remove this check.
        if (boost::filesystem::is_regular_file(itr->path())) {
            // assign current file name to current_file and echo it out to the
            // console.
            std::string current_file = itr->path().string();
            list.push_back(current_file);
        }
    }

    return list;
}

std::vector<std::string> list_files_regexp(const std::string& dir,
    const std::string& regex)
{
    DLOG(INFO) << "... listing....";

    auto files = list_files(dir);

    DLOG(INFO) << "Found " << files.size() << " in dir";

    //	for (auto s: files)
    //	{
    //		LOG(INFO) << s;
    //	}

    DLOG(INFO) << "... going to do regexp filtering....";
    auto out = filter_regex_match(files, regex);

    DLOG(INFO) << "Regex here: found " << out.size() << " matches";

    return out;
}

std::string applyTimeStampOnFileName(const std::string &file_name)
{
    std::string no_ext = stripExtension(file_name);

    std::string ext = getExtension(file_name);

    std::string out =  concatenateFilenameWithSeparator(no_ext, getTimeStamp());

    if (!ext.empty())
        out += ext;

    return out;
}

} // end spc nspace
