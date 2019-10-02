#include <spc/io/element_io.h>
#include <iostream>

#include <spc/core/logging.h>
using namespace spc;

INITIALIZE_EASYLOGGINGPP

int main(int argc, char ** argv)
{
	START_EASYLOGGINGPP(argc, argv);


    if (argc != 2)
    {
        std::cout << "error in command line options" << std::endl;
        return -1;
    }

    std::string fname (argv[1]);


    boost::filesystem::path p (fname);
    std::cout << "found file:" <<  p.filename() << std::endl;

    std::cout << "laoding..." << std::endl;
    ISerializable::Ptr obj;
    obj = spc::io::deserializeFromFile(fname);


    boost::filesystem::path newpath = p.parent_path() / p.stem();

    std::cout << "renamed to " << newpath.string() + ".spc" <<  std::endl;


    spc::io::serializeToFile(obj,newpath.string(), spc::io::SPC);

    std::cout << "file saved!" << std::endl;


    return 1;
}
