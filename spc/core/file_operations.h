#ifndef SPC_FILE_OPERATIONS_H
#define SPC_FILE_OPERATIONS_H


#include <string>
#include <vector>



namespace spc
{


std::string getCurrentDirectory();

//! says if the file fname exists.
bool fileExists(const std::string &fname);


//! get a list of all the files in the given directory
std::vector<std::string> list_files(const std::string &dir_name);

std::vector<std::string> list_files_regexp(const std::string  &dir, const std::string &regex);



}//end nspace

#endif // FILE_OPERATIONS_H
