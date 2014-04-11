#ifndef SPC_SVMLIB_HELPER_H
#define SPC_SVMLIB_HELPER_H

#include <vector>
#include <fstream>
#include <sstream>

#include <boost/algorithm/string/trim.hpp>

namespace spc
{

///
/// \brief writeToSVMlibFile writes a series of nested vectors (fields for SVM
/// in svm-copatible format)
/// \param fields
/// \param filename
///
void writeToSVMlibFile(const std::vector
                       <std::vector<std::vector<float> > > &fields,
                       const std::string filename);

} // end nspace

#endif // SPC_SVMLIB_HELPER_H
