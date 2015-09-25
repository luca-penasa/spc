#ifndef SPC_REGEX_H
#define SPC_REGEX_H



#include <vector>
#include <string>


namespace spc
{


//! filter a list of strin by matching regex
std::vector<std::string> filter_regex_match(const std::vector<std::string> & in_list ,const  std::string & reg_string);


}// end nspace
#endif // REGEX_H
