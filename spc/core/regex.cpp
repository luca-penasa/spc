
#include "regex.h"
#include <boost/xpressive/xpressive.hpp>
#include <spc/core/logging.h>


std::vector<std::string> spc::filter_regex_match(const std::vector<std::string> &in_list, const std::string &reg_string)
{

	std::vector<std::string> cleaned;
	boost::xpressive::sregex regex =  boost::xpressive::sregex::compile (reg_string);

	for (auto s: in_list)
	{
		if (boost::xpressive::regex_match(s, regex))
		{
			cleaned.push_back(s);
		}
	}

	return cleaned;
}
