#ifndef SPC_FILE_OPERATIONS_H
#define SPC_FILE_OPERATIONS_H


#include <string>
#include <vector>



namespace spc
{


/** \brief Get the part before "_" of a string.
 * \param[in] file_name string filename
 * \return the filename the _* part
 */
std::string getBaseFileName(const std::string &file_name);

/** \brief Concatenate a string with another string , separating the two with a separator you can chose
 * \param[in] file_name string filename
 * \param[in] text string to be added after the separator
 * \return the filename as filename_text
 */
std::string concatenateFilenameWithSeparator(const std::string &file_name, const std::string &text, const std::string &sep = "_");

std::string getExtension(const std::string& file_name);

std::string applyTimeStampOnFileName(const std::string &file_name);

/** \brief Remove extension from string representing a file name.
 * \param[in] file_name string filename
 * \return the filename without extension or the original filename if no "." were found
 */
std::string stripExtension(const std::string &file_name);


/**
 * @brief getCurrentDirectory
 * @return the current running directory, it uses c++ cwd()
 */
std::string getCurrentDirectory();

/**
 * @brief fileExists
 * @param fname the full/relative path of a file to be checked for existence
 * @return true if the file exists a the path fiven in fname
 */
bool fileExists(const std::string &fname);


/**
 * @brief list_files
 * @param dir_name the directory name
 * @return a list of all the files in the given directory
 * @note not sure what it would happen if the dir does not exists. Probably just an empty vector
 * @note taken from https://gist.github.com/vivithemage/9517678
 */
std::vector<std::string> list_files(const std::string &dir_name);


/**
 * @brief list_files_regexp
 * @param dir the path
 * @param regex a string representing the regexp to be used for filtering
 * @return a list of matching files in the directory
 */
std::vector<std::string> list_files_regexp(const std::string  &dir, const std::string &regex);



}//end nspace

#endif // FILE_OPERATIONS_H
