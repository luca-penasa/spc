#include "element_io.h"

#include <cereal/types/polymorphic.hpp>




namespace spc
{

int ElementsIO::serializeToFile(const spcSerializableObject::Ptr element, std::string filename, const ARCHIVE_TYPE &type)
{
    if (!element->isSPCSerializable())
    {
        pcl::console::print_error("Trying to serialize an unserializable object. Nothing Done.\n");
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

spcSerializableObject::Ptr ElementsIO::deserializeFromFile(const std::string filename)
{

    spc::spcSerializableObject::Ptr ptr; // a null pointer
    if (!boost::filesystem::exists( filename ))
    {
        pcl::console::print_error("Trying to deserialize a non existent-file. Null Pointer returned\n ");
        return ptr;
    }

    boost::filesystem::path path = (filename);
    std::string extension = path.extension().c_str();

    ARCHIVE_TYPE matched_type;

    if (!((extension == ".xml") || (extension == ".json") || (extension == ".spc")))
    {
        pcl::console::print_error("Extension not supported. Returned null pointer.\n");
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

int ElementsIO::serializeToStream(const spcSerializableObject::Ptr element, std::ostream &stream, const ARCHIVE_TYPE &type)
{

    if (type == XML)
    {
        cereal::XMLOutputArchive archive(stream);
        archive(cereal::make_nvp("SPCDataset",element));
    }

    if (type == JSON)
    {
        cereal::JSONOutputArchive archive(stream);
        archive(cereal::make_nvp("SPCDataset", element));
    }

    if (type == SPC)
    {
        cereal::BinaryOutputArchive archive(stream);
        archive(cereal::make_nvp("SPCDataset", element));
    }
}

spcSerializableObject::Ptr ElementsIO::deserializeFromStream(std::istream &stream, const ARCHIVE_TYPE &type)
{
    spcSerializableObject::Ptr ptr;

    if (type == XML)
    {
        cereal::XMLInputArchive archive(stream);
        archive(cereal::make_nvp("SPCDataset", ptr));
    }
    else if (type == JSON)
    {
        cereal::JSONInputArchive archive(stream);
        archive(cereal::make_nvp("SPCDataset", ptr));
    }
    else if (type == SPC)
    {
        cereal::BinaryInputArchive archive(stream);
        archive(cereal::make_nvp("SPCDataset", ptr));
    }

    return ptr;
}

int ElementsIO::serializeToString(const spcSerializableObject::Ptr element, std::string &string, const ElementsIO::ARCHIVE_TYPE &type)
{
    std::stringstream sstream;
    serializeToStream(element, sstream, type);
    string = sstream.str() ;
    return 1;
}

spcSerializableObject::Ptr ElementsIO::deserializeFromString(std::string &string, const ElementsIO::ARCHIVE_TYPE &type)
{

    //    std::stringstream sstream(string.c_str());
    //    return deserializeFromStream(sstream, type);

    std::stringstream sstream("");
    sstream.write(string.data(), string.size());


    //    sstream.write(&(*string.begin()), string.size()) ;
    //    std::cout << string.c_str() << std::endl;

    return deserializeFromStream(sstream, type);
}



}
