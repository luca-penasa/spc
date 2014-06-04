#include <spc/elements/ICalDataDB.h>
#include <pcl/console/print.h>

//void spc::DataDB::fromFile(const std::string filename)
//{
//    std::ifstream in(filename.c_str());
//    if (!in.is_open()) {
//        pcl::console::print_error("Error loading file\n");
//        return;
//    }

//    std::vector<std::string> fields;

//    this->clear();

//    std::string line;

//    std::getline(in, line);

//    // Tokenize the line
//    typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
//    boost::char_separator<char> sep(" ");
//    tokenizer tokens(line, sep);

//    // Assign tokens to a string vector
//    fields.assign(tokens.begin(), tokens.end());

//    pcl::console::print_info("found %i fields\n", fields.size());

//    // first line is the header!

//    std::vector<std::string> vec;
//    while (in.eof() != 1) {
//        // Always get the line -> so while advances
//        std::getline(in, line);

//        if (line.size() == 0) {
//            continue;
//        }

//        typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
//        boost::char_separator<char> sep(" ");
//        tokenizer tokens(line, sep);

//        // Assign tokens to a string vector
//        vec.clear();
//        vec.assign(tokens.begin(), tokens.end());

//        CorePoint::Ptr cp(new CorePoint);

//        size_t col_count = 0;
//        spcForEachMacro(std::string fieldname, fields)
//        {

//            std::string word = vec.at(col_count++);

//            if ((fieldname == "intensity") | (fieldname == "distance")
//                | (fieldname == "angle")) {
//                cp->setValue(fieldname, (float)atof(word.c_str()));
//            } else if ((fieldname == "cloud_id") | (fieldname == "core_id")
//                       | (fieldname == "nn")) {
//                cp->setValue(fieldname, atoi(word.c_str()));
//            }
//        }

//        db_.push_back(cp);
//    }
//}
