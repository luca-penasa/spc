#ifndef HELPERS_H
#define HELPERS_H


#include <dirent.h>
#include <vector>
#include <string>

using namespace std;

int getFilesInDir(const string directory, vector<string> & list);

bool fexists(string filename);

string strip_extension(string fullname);

#endif // HELPERS_H
