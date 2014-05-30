#include "element_io.h"

#include <cereal/types/polymorphic.hpp>
#include <pcl/console/print.h>

namespace spc
{
namespace io
{

int serializeToFile(const ElementBase::Ptr element, std::string filename,
                    const ARCHIVE_TYPE &type)
{
    if (!element->isSPCSerializable()) {
        pcl::console::print_error(
            "Trying to serialize an unserializable object. Nothing Done.\n");
        return -1;
    }

    std::string extension; // we could put them in a map maybe

    if (type == XML)
        extension = ".xml";

    else if (type == JSON)
        extension = ".json";

    else if (type == SPC)
        extension = ".spc";

    filename += extension;

    std::ofstream os(filename); // open file as ofstream
    serializeToStream(element, os, type);

    return 1;
}

ElementBase::Ptr deserializeFromFile(const std::string filename)
{

    spc::ElementBase::Ptr ptr; // a null pointer
    if (!boost::filesystem::exists(filename)) {
        pcl::console::print_error("Trying to deserialize a non existent-file. "
                                  "Null Pointer returned\n ");
        return ptr;
    }

    boost::filesystem::path path = (filename);
    std::string extension = path.extension().c_str();

    ARCHIVE_TYPE matched_type;

    if (!((extension == ".xml") || (extension == ".json")
          || (extension == ".spc"))) {
        pcl::console::print_error(
            "Extension not supported. Returned null pointer.\n");
        return ptr;
    }

    if (extension == ".xml")
        matched_type = XML;
    else if (extension == ".json")
        matched_type = JSON;
    else if (extension == ".spc")
        matched_type = SPC;

    std::ifstream is(filename); // open filename

    return deserializeFromStream(is, matched_type);
}

int serializeToStream(const ElementBase::Ptr element, std::ostream &stream,
                      const ARCHIVE_TYPE &type)
{

    if (type == XML) {
        cereal::XMLOutputArchive archive(stream);
        archive(cereal::make_nvp("SPCDataset", element));
    }

    if (type == JSON) {
        cereal::JSONOutputArchive archive(stream);
        archive(cereal::make_nvp("SPCDataset", element));
    }

    if (type == SPC) {
        cereal::BinaryOutputArchive archive(stream);
        archive(cereal::make_nvp("SPCDataset", element));
    }
}

ElementBase::Ptr deserializeFromStream(std::istream &stream,
                                     const ARCHIVE_TYPE &type)
{
    ElementBase::Ptr ptr;

    if (type == XML) {
        cereal::XMLInputArchive archive(stream);
        archive(cereal::make_nvp("SPCDataset", ptr));
    } else if (type == JSON) {
        cereal::JSONInputArchive archive(stream);
        archive(cereal::make_nvp("SPCDataset", ptr));
    } else if (type == SPC) {
        cereal::BinaryInputArchive archive(stream);
        archive(cereal::make_nvp("SPCDataset", ptr));
    }

    return ptr;
}

int serializeToString(const ElementBase::Ptr element, std::string &string,
                      const io::ARCHIVE_TYPE &type)
{
    std::stringstream sstream;
    serializeToStream(element, sstream, type);
    string = sstream.str();
    return 1;
}

ElementBase::Ptr deserializeFromString(std::string &string,
                                     const io::ARCHIVE_TYPE &type)
{
    std::stringstream sstream("");
    sstream.write(string.data(), string.size());
    return deserializeFromStream(sstream, type);
}
}
}
