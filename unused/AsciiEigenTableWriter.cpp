#include "AsciiEigenTableWriter.h"

//#include <pcl/console/print.h>
#include <vector>
#include <boost/algorithm/string.hpp>

#include <boost/filesystem.hpp>

namespace spc
{

namespace io
{

void AsciiEigenTableWriter::setInput(const EigenTable::Ptr object)
{
    if (!object->isAsciiSerializable()) {
        LOG(ERROR) <<
                    "Cannot export selected object to ascii";
        return;
    }

    table_ = object;
}

void AsciiEigenTableWriter::writeContent(std::ostringstream &stream) const
{
    // now the data
    size_t row, col;
    row = table_->mat().rows();
    col = table_->mat().cols();

    for (size_t r = 0; r < row; ++r) {
        for (size_t c = 0; c < col; ++c) {
            stream << boost::lexical_cast<std::string>(table_->mat()(r, c));
            stream << separator_.c_str();
        }
        stream << std::endl;
    }
}

void AsciiEigenTableWriter::writeHeader(std::ostringstream &stream) const
{
    std::vector<std::string> names = table_->getScalarColumnsNames();

    if (names.empty())
        return;

    for (std::string name : names) {
        stream << name;
        stream << separator_.c_str();
    }
    stream << std::endl;

}

void AsciiEigenTableWriter::openAndLockFile()
{
    ofilestream_.open(filename_.c_str());

    // Mandatory lock file
    lock_ = boost::interprocess::file_lock(filename_.c_str());
    namespace fs = boost::filesystem;

    fs::permissions(fs::path(filename_),
                    fs::add_perms | fs::set_gid_on_exe);
}

void AsciiEigenTableWriter::closeAndUnlockFile()
{
    ofilestream_.close();
    namespace fs = boost::filesystem;

    fs::permissions(fs::path(filename_),
                    fs::remove_perms | fs::set_gid_on_exe);
    lock_.unlock();
}

void AsciiEigenTableWriter::write()
{

    if (filename_.empty()) {
        LOG(ERROR) << "empty path and filename";
        return;
    }


    // now a string stream
    std::ostringstream stream;
    stream.precision(precision_);
    stream.imbue(std::locale::classic());


    if (write_headers_)
        writeHeader(stream);

    writeContent(stream);

    // stream now contains the line
    std::string result = stream.str();
    boost::trim(result);
    stream.str("");
    openAndLockFile();


    ofilestream_ << result << std::endl;

    closeAndUnlockFile();

    return;
}


}//end io
}//end spc

