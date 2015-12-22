#pragma once
#ifndef GFLAGGING_H
#define GFLAGGING_H
#include <gflags/gflags.h>

#include <fstream>
#include <spc/core/file_operations.h>

#include <spc/core/timing.h>

// some convenience functions to be used in tools
namespace spc
{


// in the form 2015-11-11.10:46:40

/**
 * @brief saveToolArgs save in an incremental sidefile in the current dir a remainder of the used args
 * @param additional_metadata can be used to add infos in the header (e.g. the name of the tool called)
 * @return
 */
int saveToolArgs(const std::string &additional_metadata = std::string()) {
    std::string fname = "spc_commands_args.txt";
    std::ofstream outfile;

    if (spc::fileExists(fname))
        outfile.open(fname, std::ios_base::app);
    else
        outfile.open(fname);

    outfile << "#\n-------------------\n#" << getTimeStamp() << "#\n-------------------\n\n" <<  google::CommandlineFlagsIntoString();
    outfile.close();

    return 0;
}


}// end spc nspace

#endif
