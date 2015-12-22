#pragma once
#ifndef SPC_REGEX_H
#define SPC_REGEX_H



#include <vector>
#include <string>


namespace spc
{


//! filter a list of strings by matching regex
std::vector<std::string> filter_filenames_regex(const std::vector<std::string> & in_list ,const  std::string & reg_string);



//void test_regexp(const std::string & pattern);

}// end nspace
#endif // REGEX_H
