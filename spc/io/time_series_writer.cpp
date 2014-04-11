#include "time_series_writer.h"

#include <fstream>
#include<sstream>

#include <boost/algorithm/string.hpp>
#include <boost/interprocess/sync/file_lock.hpp>
#include <boost/filesystem.hpp>


#include <spc/common/common.h>

namespace spc
{
template<typename ScalarT>
TimeSeriesWriter<ScalarT>::TimeSeriesWriter(): precision_(6), separator_(" ")
{
}

template<typename ScalarT>
int
TimeSeriesWriter<ScalarT>::writeAsciiAsSparse()
{
    int n_rows = in_series_->getNumberOfSamples();

    std::ofstream file;
    file.open (filename_.c_str());

    // Mandatory lock file


   boost::interprocess::file_lock  lock = boost::interprocess::file_lock (filename_.c_str());
//    if (lock.try_lock ())
//      PCL_DEBUG ("[pcl::PCDWriter::setLockingPermissions] File %s locked succesfully.\n", file_name.c_str ());
//    else
//      PCL_DEBUG ("[pcl::PCDWriter::setLockingPermissions] File %s could not be locked!\n", file_name.c_str ());

    namespace fs = boost::filesystem;
    fs::permissions (fs::path (filename_), fs::add_perms | fs::set_gid_on_exe);


    std::ostringstream stream;
    stream.precision (precision_);
    stream.imbue (std::locale::classic ());


    std::vector<ScalarT> x = in_series_->getX();
    std::vector<ScalarT> y = in_series_->getY();

    for (int i = 0; i < n_rows; ++i)
    {

        ScalarT val_x,val_y;
        val_x = x.at(i);
        val_y = y.at(i);

        if (pcl_isnan(val_x))
            stream << "nan";
        else
            stream << val_x;

        // the separator!
        stream << separator_.c_str();

        if (pcl_isnan(val_y))
            stream << "nan";
        else
            stream << val_y;


        //stream now contains the line
        std::string result = stream.str ();
        boost::trim (result);
        stream.str ("");
        file << result << std::endl; //new line

    }

    file.close();

    fs::permissions (fs::path (filename_), fs::remove_perms | fs::set_gid_on_exe);
    lock.unlock ();

}

template class TimeSeriesWriter<float>;
template class TimeSeriesWriter<double>;
}//end nspace
