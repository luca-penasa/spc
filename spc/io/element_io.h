#ifndef ELEMENT_IO_H
#define ELEMENT_IO_H

#include <Eigen/Dense>
#include <fstream>

#include <spc/elements/SerializableObject.h>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>

#include <spc/common/eigen_serialization.hpp>
#include <spc/common/cereal_types.hpp>

#include <spc/elements/movable_element.h>
#include <spc/elements/SerializableObject.h>
#include <spc/elements/ModificableElement.h>

#include <boost/filesystem.hpp>

namespace spc {

class ElementsIO: public ModificableElement
{

public:
    enum ARCHIVE_TYPE {XML, JSON, SPC}; // spc is cereal-binary

    /**
     * @brief serializeToFile
     * @param element
     * @param filename
     * @param cereal_outname
     * @return
     * Note the filename must be with no extension. Extension is automatically appended depending on the type
     */
    static int serializeToFile(const spcSerializableObject::Ptr element,
                               std::string filename,
                               const ARCHIVE_TYPE &type = SPC);

    /**
     * @brief deserializeFromFile get a filename and gives back an scpSerializableObject::Ptr
     * @param filename with extension, we will guess the right archive from that
     * @return
     */
    static spcSerializableObject::Ptr deserializeFromFile(const std::string filename);


    static int serializeToStream(const spcSerializableObject::Ptr element,
                                 std::ostream &stream,
                                 const ARCHIVE_TYPE &type = SPC);

    static spcSerializableObject::Ptr deserializeFromStream(std::istream &stream,
                                                            const ARCHIVE_TYPE &type = SPC);

    static int serializeToString(const spcSerializableObject::Ptr element,
                                 std::string &string,
                                 const ARCHIVE_TYPE &type = SPC);

    static spcSerializableObject::Ptr deserializeFromString(std::string &string,
                                                            const ARCHIVE_TYPE &type = SPC);

};


} //end nspace

#endif // ELEMENT_IO_H
