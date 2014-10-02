#ifndef SAMPLEDDATA_H
#define SAMPLEDDATA_H

//#include <spc/elements/EigenTable.h>
#include <spc/io/element_io.h>
#include <spc/calibration/Observations.h>
#include <spc/calibration/HelperMethods.h>

namespace spc
{

class SampledData
{
public:
    SampledData();


    void readFile(const std::string &fname)
    {
        ISerializable::Ptr o =  spc::io::deserializeFromFile(fname);

        table_ = spcDynamicPointerCast<EigenTable> (o);

        table_ = table_->getWithStrippedNANs({"distance", "intensity", "angle", "intensity_std"});

        std::cout << "Following columns found:\n" << std::endl;
        for (int i = 0; i < table_->getNumberOfColumns(); ++i)
        {
            std::cout << table_->getColumnName(i) << std::endl;
        }

        obs_ = table2observations(*table_);

        d_ = table_->mat().col(table_->getColumnId("distance"));
        a_ = table_->mat().col(table_->getColumnId("angle"));
        i_ = table_->mat().col(table_->getColumnId("intensity"));



        cloud_ids_ = table_->mat().col(table_->getColumnId("cloud_id")).cast<int>();
        unique_ids_ = unique(cloud_ids_);
    }


    EigenTable::Ptr table_;

    Eigen::VectorXf d_;
    Eigen::VectorXf a_;
    Eigen::VectorXf i_;
    Eigen::VectorXi cloud_ids_;
    Eigen::VectorXi unique_ids_;

    std::vector<Observation> obs_;

};

}//end nspace

#endif // SAMPLEDDATA_H
