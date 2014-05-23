#include "element_io.h"

#include <cereal/types/polymorphic.hpp>




namespace spc
{

int ElementsIO::serializeToFile(const spcSerializableObject::Ptr element, std::string filename, const std::string cereal_outname)
{
    if (!element->isSPCSerializable())
    {
        pcl::console::print_error("Trying to serialize an unserializable object. Nothing Done.\n");
        return -1;
    }

    if (archive_type_ == XML)
    {
        filename += ".xml";
        std::ofstream os(filename); // open filename
        cereal::XMLOutputArchive archive(os);
        archive(cereal::make_nvp(cereal_outname.c_str(),element));
    }

    if (archive_type_ == JSON)
    {
        filename += ".json";
        std::ofstream os(filename); // open filename
        cereal::JSONOutputArchive archive(os);
        archive(cereal::make_nvp(cereal_outname.c_str(), element));
    }

    if (archive_type_ == SPC)
    {
        filename += ".spc";
        std::ofstream os(filename); // open filename
        cereal::BinaryOutputArchive archive(os);
        archive(cereal::make_nvp(cereal_outname.c_str(), element));
    }

    return 1;



}

spcSerializableObject::Ptr ElementsIO::deserializeFromFile(const std::string filename)
{

    spcSerializableObject::Ptr ptr;

    if (!boost::filesystem::exists( filename ))
    {
        pcl::console::print_error("Trying to deserialize a non existent-file. Null Pointer returned\n ");
        return ptr;
    }

    boost::filesystem::path path = (filename);
    std::string extension = path.extension().c_str();

    if (!((extension == ".xml") || (extension == ".json") || (extension == ".spc")))
    {
        pcl::console::print_error("Extension not supported. Returned null pointer.\n");
        return ptr;
    }

    std::ifstream is(filename); // open filename


    if (extension == ".xml")
    {
        cereal::XMLInputArchive archive(is);
        archive(cereal::make_nvp("SPC Dataset", ptr));
    }
    else if (extension == ".json")
    {
        cereal::JSONInputArchive archive(is);
        archive(cereal::make_nvp("SPC Dataset", ptr));
    }
    else if (extension == ".spc")
    {
        cereal::BinaryInputArchive archive(is);
        archive(cereal::make_nvp("SPC Dataset", ptr));
    }

    return ptr;

}



}
