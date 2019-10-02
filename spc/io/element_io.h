#ifndef ELEMENT_IO_H
#define ELEMENT_IO_H
#include <spc/core/macros_ptr.h>
#include <spc/core/spc_eigen.h>
#include <spc/core/SerializableInterface.h>

//#include <spc/elements/MovableElement.h>
#include <boost/filesystem.hpp>

//#include <boost/bimap.hpp>

namespace spc {

namespace io {

enum ARCHIVE_TYPE {
    XML,
    JSON,
    SPC,
    ASCII
}; // spc is cereal-binary



std::string type_to_extension (const ARCHIVE_TYPE &type);


ARCHIVE_TYPE extension_to_type(const std::string & extension);

/**
     * @brief serializeToFile
     * @param element
     * @param filename
     * @param cereal_outname
     * @param timestamp_file if true a timestamp will be added to the filename
     * @param sidefile_string if not null a sidefile will be written together with the file, containing the string in sidefile_string (multiline allowed)
     * @return
     * Note the filename must be with no extension. Extension is automatically
 * appended depending on the type
     */
int serializeToFile(const ISerializable::Ptr element, std::string filename,
                    const ARCHIVE_TYPE& type = SPC, const bool timestamp_file = false,
                    const std::string& sidefile_string = std::string(),
                    const int progressive_number = -1,
                    const size_t number_of_digits = 3 );


// same but for many objects, integer postfix added tot he name
int serializeToFile(const std::vector<ISerializable::Ptr> elements, std::string filename,
                    const ARCHIVE_TYPE& type = SPC, const bool timestamp_file = false,
                    const std::string& sidefile_string = std::string(),
                    const size_t number_of_digits = 3);

/**
     * @brief deserializeFromFile get a filename and gives back an
     * scpSerializableObject::Ptr
     * @param filename with extension, we will guess the right archive from that
     * @return
     */
ISerializable::Ptr deserializeFromFile(const std::string filename);

int serializeToStream(const ISerializable::Ptr element, std::ostream& stream,
                      const ARCHIVE_TYPE& type = SPC);

ISerializable::Ptr deserializeFromStream(std::istream& stream,
                                         const ARCHIVE_TYPE& type = SPC);

int serializeToString(const ISerializable::Ptr element, std::string& string,
                      const ARCHIVE_TYPE& type = SPC);

ISerializable::Ptr deserializeFromString(std::string& string,
                                         const ARCHIVE_TYPE& type = SPC);

//static std::vector<int> parseLoadableFiles(int argc, char **argv);

//void testXMLMatrixWrite();
} // end io namespace

} // end nspace

#endif // ELEMENT_IO_H
