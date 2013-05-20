#ifndef SPC_STRINGS_H
#define SPC_STRINGS_H



#include <string>


/** \brief Remove extension from string representing a file name.
 * \param[in] file_name string filename 
 * \return the filename without extension
 */
std::string 
stripExtension(const std::string& file_name);

/** \brief Get the part before "_" of a string.
 * \param[in] file_name string filename 
 * \return the filename the _* part
 */
std::string 
getBaseFileName(const std::string& file_name);

/** \brief Add a string to a string , separating the two with a _
 * \param[in] file_name string filename 
 * \param[in] text string to be added after a "_"
 * \return the filename as filename_text
 */
std::string 
addSubscript(const std::string &file_name, const std::string &text);



#endif
