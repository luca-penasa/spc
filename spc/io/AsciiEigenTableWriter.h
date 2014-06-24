#ifndef ASCIIWRITERBASE_H
#define ASCIIWRITERBASE_H

#include <spc/elements/macros_ptr.h>
#include <boost/interprocess/sync/file_lock.hpp>
//#include <sstream>
#include <fstream>
#include <spc/elements/EigenTable.h>

namespace spc
{

namespace io
{

class AsciiEigenTableWriter
{
public:

    void setInput(const EigenTable::Ptr object);

    virtual void writeContent(std::ostringstream &stream) const;

    virtual void writeHeader(std::ostringstream &stream) const;

    void openAndLockFile();

    void closeAndUnlockFile();

    virtual void write();

    void setOutputFilename(const std::string &filename)
    {
        filename_ = filename;
    }

    void setASCIIPrecision(const int precision)
    {
        precision_ = precision;
    }

    void setSeparator(const std::string &separator)
    {
        separator_ = separator;
    }

    void setWriteHeaders(const bool w)
    {
        write_headers_ = w;
    }

private:
    EigenTable::Ptr table_;
    std::string filename_;
    int precision_ = 6;
    std::string separator_ = " ";
    bool write_headers_ = true;

    std::ofstream ofilestream_;

    boost::interprocess::file_lock lock_;
};

} // end nspace spc::io
} // ennd nspace spc

#endif // ASCIIWRITERBASE_H
