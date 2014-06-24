#ifndef SPC_CALIBRATION_DATA_FILTER_H
#define SPC_CALIBRATION_DATA_FILTER_H

#include <spc/elements/EigenTable.h>
namespace spc
{

class IntensityCalibrationDataFilter
{

public:
    void setData(const EigenTable::Ptr data)
    {
        data_ = data;
    }

    std::vector<size_t> getGoodIds(std::vector<size_t> good_ids = std::vector
                                   <size_t>()) const
    {
        std::cout << "here" << std::endl;

        if (!data_) {
            pcl::console::print_error("No dataset as input.\n");
            return good_ids;
        }

        size_t n_rows = data_->getNumberOfRows();

        std::cout << "here" << std::endl;
        Eigen::VectorXf n_neighbors = data_->column("n_neighbors");

        for (size_t i = 0; i < n_rows; ++i) {
            if (n_neighbors(i) >= min_n_neighbors_)
                good_ids.push_back(i);
        }

        return good_ids;
    }

    EigenTable::Ptr applyFilter() const
    {
        std::vector<size_t> good_ids = getGoodIds();

        EigenTable::Ptr good(new EigenTable(*data_, true));

        size_t counter = 0;
        for (size_t id : good_ids)
            good->row(counter++) = data_->row(id);

        addWeightsToTable(good);

        return good;
    }

    void setMinNNeighbors(const size_t &n)
    {
        min_n_neighbors_ = n;
    }

    static void addWeightsToTable(const EigenTable::Ptr table)
    {
        // provide space
        table->addNewComponent("intensity_w");
        table->addNewComponent("eigen_w");

        table->column("intensity_w")
            = table->column("intensity_std")
                  .array()
                  .square()
                  .inverse()
                  .matrix(); // do the square before. its a std
        table->column("eigen_w")
            = table->column("eigen_ratio").cwiseInverse().matrix(); // just the
                                                                    // inverse.
                                                                    // its a
                                                                    // variance
    }

protected:
    EigenTable::Ptr data_;

    size_t min_n_neighbors_ = 3;
};

} // end nspace

#endif
