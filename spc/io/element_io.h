#ifndef ELEMENT_IO_H
#define ELEMENT_IO_H

#include <Eigen/Dense>
#include <fstream>

#include <cereal/archives/xml.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>

#include <spc/io/eigen_serialization.hpp>
#include <spc/io/cereal_types.hpp>

#include <spc/elements/MovableElement.h>
#include <boost/filesystem.hpp>

namespace spc {

namespace io {


enum ARCHIVE_TYPE {XML, JSON, SPC}; // spc is cereal-binary

/**
     * @brief serializeToFile
     * @param element
     * @param filename
     * @param cereal_outname
     * @return
     * Note the filename must be with no extension. Extension is automatically appended depending on the type
     */
int serializeToFile(const spcObject::Ptr element,
                    std::string filename,
                    const ARCHIVE_TYPE &type = SPC);

/**
     * @brief deserializeFromFile get a filename and gives back an scpSerializableObject::Ptr
     * @param filename with extension, we will guess the right archive from that
     * @return
     */
spcObject::Ptr deserializeFromFile(const std::string filename);


int serializeToStream(const spcObject::Ptr element,
                      std::ostream &stream,
                      const ARCHIVE_TYPE &type = SPC);

spcObject::Ptr deserializeFromStream(std::istream &stream,
                                     const ARCHIVE_TYPE &type = SPC);

int serializeToString(const spcObject::Ptr element,
                      std::string &string,
                      const ARCHIVE_TYPE &type = SPC);

spcObject::Ptr deserializeFromString(std::string &string,
                                     const ARCHIVE_TYPE &type = SPC);

} // end io namespace


} //end nspace

#endif // ELEMENT_IO_H
