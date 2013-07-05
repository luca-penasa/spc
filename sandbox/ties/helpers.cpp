#include "helpers.h"

int getFilesInDir(const string directory, vector<string> & list)
{
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (directory.c_str())) != NULL) {
      /* print all the files and directories within directory */
      while ((ent = readdir (dir)) != NULL) {
        printf ("%s\n", ent->d_name);
      }
      closedir (dir);
    } else {
      /* could not open directory */
      perror ("");
      return -1;
    }
}



bool fexists(string filename)
{
    FILE * file = fopen(filename.c_str(), "r");
    return file;
}



string strip_extension(string fullname)
{
    int lastindex = fullname.find_last_of(".");
    string rawname = fullname.substr(0, lastindex);
    return rawname;
}

