#pragma once
#ifndef SPC_STRINGS_H
#define SPC_STRINGS_H

#include <string>
#include <vector>
#include <sstream>

namespace spc
{





/**
 * @brief splitStringAtSeparator use boost::tokenizer to split a string in a vector of string
 * @param line the string to split
 * @param separator the splitted version of the original line
 * @return
 */
std::vector<std::string> splitStringAtSeparator(const std::string &line, const std::string &separator = " ");


/**
 * @brief splitLines split a multiline string in lines components
 * @param muliline_text
 * @return the vector of string, one line per element
 */
std::vector<std::string> splitLines(const std::string & muliline_text);


/** from here
 * http://stackoverflow.com/questions/11521183/return-fixed-length-stdstring-from-integer-value
 **/
std::string fixedLength(const int i, const int length);


} // end namespace
#endif
