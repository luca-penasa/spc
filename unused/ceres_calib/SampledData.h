#ifndef SAMPLEDDATA_H
#define SAMPLEDDATA_H

//#include <spc/elements/EigenTable.h>
#include <spc/io/element_io.h>
#include <spc/ceres_calibration/Observations.h>
#include <spc/ceres_calibration/HelperMethods.h>

#include <spc/elements/calibration/DataHolder.h>

namespace spc
{

class SampledData
{
public:
    SampledData();


    void readFile(const std::string &fname)
    {
        ISerializable::Ptr o =  spc::io::deserializeFromFile(fname);

        data_ = spcDynamicPointerCast<calibration::DataHolder> (o);
        data_->updateCloudIDsInObservations();

        obs_ = data_->getAllObservations();

//        table_ = table_->getWithStrippedNANs({"distance", "intensity", "angle", "intensity_std"});

//        // add a constant to intensities
////        table_->mat().col(table_->getColumnId("intensity")) =  table_->mat().col(table_->getColumnId("intensity")).array() + 10000;

//        std::cout << "Following columns found:\n" << std::endl;
//        for (int i = 0; i < table_->getNumberOfColumns(); ++i)
//        {
//            std::cout << table_->getColumnName(i) << std::endl;
//        }

//        obs_ = table2observations(*table_);

//        d_ = table_->mat().col(table_->getColumnId("distance"));
//        a_ = table_->mat().col(table_->getColumnId("angle"));
//        i_ = table_->mat().col(table_->getColumnId("intensity"));



//        cloud_ids_ = table_->mat().col(table_->getColumnId("cloud_id")).cast<int>();
//        unique_ids_ = unique(cloud_ids_);
    }


//    EigenTable::Ptr table_;

    calibration::DataHolder::Ptr data_;

    Eigen::VectorXf d_;
    Eigen::VectorXf a_;
    Eigen::VectorXf i_;
    Eigen::VectorXi cloud_ids_;
//    Eigen::VectorXi unique_ids_;

    std::vector<calibration::Observation::Ptr> obs_;

};

}//end nspace

#endif // SAMPLEDDATA_H
