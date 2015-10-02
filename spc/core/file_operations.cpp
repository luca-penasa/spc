#include "file_operations.h"
#include <dirent.h>


#include <spc/core/logging.h>

#include <spc/core/regex.h>

#include <boost/filesystem.hpp>

#ifdef _WIN32
	#include <direct.h>
	#define getcwd _getcwd // stupid MSFT "deprecation" warning
//#else
//    #include <unistd.h>
#endif


bool spc::fileExists(const std::string &fname)
{
	return boost::filesystem::exists(fname);
}


std::string spc::getCurrentDirectory()
{
	char buffer[10000];
	char *answer = getcwd(buffer, sizeof(buffer));
	std::string s_cwd;
	if (answer)
		s_cwd = answer;
	return s_cwd;
}


std::vector<std::string> spc::list_files(const std::string &dir_name)
{

	std::vector<std::string> list;
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (dir_name.c_str())) != NULL)
	{
		while ((ent = readdir (dir)) != NULL)
		{
			list.push_back(ent->d_name);
		}
		closedir (dir);
		return list;
	}
}


std::vector<std::string> spc::list_files_regexp(const std::string &dir, const std::string &regex)
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

	DLOG(INFO) << "Found " << out.size() << " matches";

	return out;
}
