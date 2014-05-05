#ifndef CALIBRATION_DATA_DB_H
#define CALIBRATION_DATA_DB_H

#include <spc/common/common_includes.h>

#include <spc/calibration/CorePointData.h>

#include <spc/common/std_helpers.hpp>

#include <fstream>

#include <boost/tokenizer.hpp>



namespace spc
{

class CalibrationDataDB
{
public:

    typedef boost::shared_ptr<CalibrationDataDB> Ptr;
    typedef const boost::shared_ptr<CalibrationDataDB> ConstPtr;

    //! empty constructor
    CalibrationDataDB() {}

    void pushBack(CorePointData::Ptr data)
    {
        db_.push_back(data);
    }

    size_t size() const
    {
        return db_.size();
    }

    std::vector<CorePointData::Ptr> getDataDB()
    {
        return db_;
    }

    size_t getNumberOfDifferentClouds() const
    {
        return this->getVectorOfUniqueClouds().size();
    }


    void printOutStuff()
    {
        BOOST_FOREACH(CorePointData::Ptr entry, db_)
                std::cout << *entry << std::endl;
    }

    std::vector<CorePointData::Ptr> getDataForCorePointID(size_t core_point_id)
    {
        if (core_ids_indices_list_.empty())
            buildCoreIdFastAccessor();


        return core_ids_indices_list_.at(core_point_id);
    }

    void buildCoreIdFastAccessor()
    {
        core_ids_indices_list_.resize(getVectorOfUniqueCorePoints().size());


        BOOST_FOREACH(CorePointData::Ptr core, db_)
        {
            size_t current_id = core->value<size_t>("core_id");
            core_ids_indices_list_.at(current_id).push_back(core);
        }
    }

    std::vector<size_t> getVectorOfUniqueCorePoints()
    {
        // first get the core points ids as a vector list
        std::vector<size_t> core_points_ids;

        BOOST_FOREACH( CorePointData::Ptr core, db_ )
        {
            size_t current_id = core->value<size_t>("core_id");
            if(!spc::element_exists<size_t>(core_points_ids, current_id))
                core_points_ids.push_back( current_id);
        }

        return core_points_ids;
    }

    std::vector<size_t> getVectorOfUniqueClouds() const
    {
        // first get the core points ids as a vector list
        std::vector<size_t> cloud_ids;

        BOOST_FOREACH( CorePointData::ConstPtr core, db_ )
        {
            size_t current_id = core->value<size_t>("cloud_id");
            if(!spc::element_exists<size_t>(cloud_ids, current_id))
                cloud_ids.push_back( current_id);
        }

        return cloud_ids;
    }



    void writeFullData(std::ostringstream &stream)
    {
        stream << "distance intensity angle cloud_id core_point_id n_neighbors " << std::endl;

        BOOST_FOREACH(CorePointData::Ptr meas, db_)
                meas->writeLine(stream);
    }

    void clear()
    {
        db_.clear();
        core_ids_indices_list_.clear();
    }

    void fromFile(const std::string filename)
    {
        std::ifstream in(filename.c_str());
        if (!in.is_open())
        {
            pcl::console::print_error("Error loading file\n");
            return;
        }


        std::vector< std::string > fields;

        this->clear();


        std::string line;

        std::getline(in, line);

        //Tokenize the line
        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
        boost::char_separator<char> sep(" ");
        tokenizer tokens(line, sep);

        //Assign tokens to a string vector
        fields.assign(tokens.begin(), tokens.end());

        pcl::console::print_info("found %i fields\n", fields.size());

        //first line is the header!

        std::vector< std::string > vec;
        while ( in.eof() != 1 )
        {
            //Always get the line -> so while advances
            std::getline(in, line);

            if (line.size() == 0)
            {
                continue;
            }


            typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
            boost::char_separator<char> sep(" ");
            tokenizer tokens(line, sep);

            //Assign tokens to a string vector
            vec.clear();
            vec.assign(tokens.begin(), tokens.end());



            CorePointData::Ptr cp (new CorePointData);

            size_t col_count = 0;
            BOOST_FOREACH(std::string fieldname, fields)
            {

                std::string word = vec.at(col_count++);


                if ((fieldname == "intensity") |
                        (fieldname == "distance") |
                        (fieldname == "angle"))

                {
                    cp->setValue(fieldname, (float) atof(word.c_str()));
                }
                else if ((fieldname == "cloud_id") |
                         (fieldname == "core_id") |
                         (fieldname == "nn"))
                {
                    cp->setValue(fieldname, (size_t) atoi(word.c_str()));
                }
            }

            db_.push_back(cp);

        }

    }

    CalibrationDataDB getValidDataOnly() const
    {
        CalibrationDataDB new_db;
        BOOST_FOREACH(const CorePointData::Ptr data, db_)
        {
            if(data->isValid())
                new_db.pushBack(data);
        }

        return new_db;
    }

    void writeToAsciiFile(const std::string filename )
    {

        std::ofstream file;
        file.open (filename.c_str());

        std::ostringstream stream;
        stream.precision (6);
        stream.imbue (std::locale::classic ());

        this->writeFullData(stream);

        std::string result = stream.str ();
        boost::trim (result);
        stream.str ("");
        file << result << std::endl;

        file.close();

    }

private:
    std::vector<CorePointData::Ptr> db_;

    std::vector<std::vector<CorePointData::Ptr> > core_ids_indices_list_;
};

}//end nspace

#endif
