#include "element_io.h"

#include <cereal/types/polymorphic.hpp>
//#include <pcl/console/print.h>

//#include <cereal/archives/xml.hpp>
//#include <cereal/archives/binary.hpp>
//#include <cereal/archives/json.hpp>

#include <spc/core/cerea_requested_types.hpp>
#include <cereal/types/vector.hpp>
#include <spc/io/eigen_serialization.hpp>
#include <spc/io/cereal_types.hpp>
#include <spc/core/file_operations.h>

#include <spc/core/timing.h>
//#include <pcl/console/parse.h>
#include <spc/elements/ElementBase.h>
#include <fstream>
#include <spc/core/strings.h>

namespace spc {
namespace io {

int serializeToFile(const ISerializable::Ptr element, std::string filename,
                    const ARCHIVE_TYPE& type /*= SPC*/, const bool timestamp_file /*= false*/,
                    const std::string& sidefile_string /*= std::string()*/,
                    const int progressive_number /*= -1*/,
                    const size_t number_of_digits /*= 3*/ )

    {
        CHECK(element != NULL) << "Cannot serialize element. ptr is null";

        if (!element->isSerializable()) {
            LOG(ERROR) << "The file cannot be deserialized";
            return -1;
        }

        std::string extension; // we could put them in a map maybe

        extension = type_to_extension(type);

        // strip any possible existent extension
        filename = spc::stripExtension(filename);

        if (progressive_number >= 0) {
            std::string progressiven = spc::fixedLength(progressive_number, number_of_digits);
            filename = spc::concatenateFilenameWithSeparator(filename, progressiven);
        }

        // add the new one
        filename += extension;

        // if requested apply the timestamp
        if (timestamp_file)
            filename = applyTimeStampOnFileName(filename);

        std::ofstream os(filename); // open file as ofstream

        // do actual serialization
        try {
            serializeToStream(element, os, type);
        }
        catch (const std::exception& exc) {
            LOG(ERROR) << "Some problem serializing the file: " << exc.what();
            return -1;
        }

        // if requested create the sidefile
        if (!sidefile_string.empty()) {
            std::string sidefile_name = filename;
            sidefile_name += ".sidefile";

            std::ofstream outfile;

            if (spc::fileExists(sidefile_name))
                outfile.open(sidefile_name, std::ios_base::app);
            else
                outfile.open(sidefile_name);

            outfile << sidefile_string;
            outfile.close();
        }

        LOG(INFO) << "File Saved to: " << filename;

        return 1;
    }

    int serializeToFile(const std::vector<ISerializable::Ptr> elements,
        std::string filename,
        const ARCHIVE_TYPE& type,
        const bool timestamp_file,
        const std::string& sidefile_string,
        const size_t number_of_digits)
    {
        size_t number = 0;
        for (ISerializable::Ptr el : elements) {
            serializeToFile(el, filename, type, timestamp_file, sidefile_string, number++, number_of_digits);
        }
    }

    ISerializable::Ptr deserializeFromFile(const std::string filename)
    {

        spc::ElementBase::Ptr ptr; // a null pointer
        if (!boost::filesystem::exists(filename)) {
            LOG(ERROR) << "Trying to deserialize a non existent-file. Null Pointer returned";
            return NULL;
        }

        boost::filesystem::path path(filename);
        std::string extension(path.extension().string());

        ARCHIVE_TYPE matched_type;

        if (!((extension == ".xml") || (extension == ".json")
                || (extension == ".spc"))) {
            LOG(ERROR) << "Extension not supported. Returned null pointer";
            return ptr;
        }

        matched_type = extension_to_type(extension);

        std::ifstream is(filename); // open filename

        setlocale(LC_NUMERIC, "C");

        //    is.imbue(std::locale("C"));

        return deserializeFromStream(is, matched_type);
    }

    int serializeToStream(const ISerializable::Ptr element, std::ostream& stream,
        const ARCHIVE_TYPE& type)
    {

        stream.imbue(std::locale("C"));

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

            if (type == ASCII) {
                if (element->isAsciiSerializable()) {
                    if (element->toAsciiStream(stream) != 1) {
                        LOG(WARNING) << "some error occured while serializing to ascii. see log";
                        return -1;
                    }
                }
                else {
                    LOG(WARNING) << "ascii serialization to a non-ascii-serializable element. nothing done";
                    return -1;
                }
            }
        }
        return 1;
    }

    ISerializable::Ptr deserializeFromStream(std::istream& stream,
        const ARCHIVE_TYPE& type)
    {
        ISerializable::Ptr ptr;

        {
            try {
                if (type == XML) {
                    cereal::XMLInputArchive archive(stream);
                    archive(cereal::make_nvp("SPCDataset", ptr));
                }
                else if (type == JSON) {
                    cereal::JSONInputArchive archive(stream);
                    archive(cereal::make_nvp("SPCDataset", ptr));
                }
                else if (type == SPC) {
                    cereal::BinaryInputArchive archive(stream);
                    archive(cereal::make_nvp("SPCDataset", ptr));
                }
            }

            catch (const std::exception& exc) {
                LOG(ERROR) << "Some problem deserializing the file: " << exc.what();
                return NULL;
            }
        }

        return ptr;
    }

    int serializeToString(const ISerializable::Ptr element, std::string& string,
        const io::ARCHIVE_TYPE& type)
    {
        std::stringstream sstream;
        serializeToStream(element, sstream, type);
        string = sstream.str();
        return 1;
    }

    ISerializable::Ptr deserializeFromString(std::string& string,
        const io::ARCHIVE_TYPE& type)
    {
        std::stringstream sstream("");
        sstream.write(string.data(), string.size());
        return deserializeFromStream(sstream, type);
    }

    std::string type_to_extension(const ARCHIVE_TYPE& type)
    {
        if (type == XML)
            return ".xml";
        else if (type == JSON)
            return ".json";
        else if (type == SPC)
            return ".spc";
        else if (type == ASCII)
            return ".txt";
    }

    ARCHIVE_TYPE extension_to_type(const std::string& extension)
    {
        if ((extension == ".xml") | (extension == "xml"))
            return XML;
        else if ((extension == ".json") | (extension == "json"))
            return JSON;
        else if ((extension == ".spc") | (extension == "spc"))
            return SPC;
        else if ((extension == ".txt") | (extension == "txt"))
            return ASCII;
    }

    //std::vector<int> parseLoadableFiles(int argc, char **argv)
    //{
    //    std::vector<int> spc_ids;
    //    spc_ids = pcl::console::parse_file_extension_argument(argc, argv, "spc");

    //    std::vector<int> xml_ids;
    //    xml_ids = pcl::console::parse_file_extension_argument(argc, argv, "xml");

    //    std::vector<int> json_ids;
    //    json_ids = pcl::console::parse_file_extension_argument(argc, argv, "json");

    //    spc_ids.insert( spc_ids.end(), xml_ids.begin(), xml_ids.end() );
    //    spc_ids.insert( spc_ids.end(), json_ids.begin(), json_ids.end() );

    //    return spc_ids;
    //}
}
}
