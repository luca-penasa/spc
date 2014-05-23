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
    ElementsIO(): archive_type_(JSON)
    {

    }

    enum ARCHIVE_TYPE {XML, JSON, SPC}; // spc is cereal-binary

    /**
     * @brief serializeToFile
     * @param element
     * @param filename
     * @param cereal_outname
     * @return
     * Note the filename must be with no extension. Extension is automatically appended depending on the type
     */
    int serializeToFile(const spcSerializableObject::Ptr element,
                        std::string filename,
                        const std::string cereal_outname = "SPC Dataset");

    /**
     * @brief deserializeFromFile get a filename and gives back an scpSerializableObject::Ptr
     * @param filename
     * @return
     */
    spcSerializableObject::Ptr deserializeFromFile(const std::string filename);

    spcGetMacro(ArchiveType, archive_type_, ARCHIVE_TYPE)
    spcSetMacro(ArchiveType, archive_type_, ARCHIVE_TYPE)

    private:
        ARCHIVE_TYPE archive_type_;

};


} //end nspace

#endif // ELEMENT_IO_H
