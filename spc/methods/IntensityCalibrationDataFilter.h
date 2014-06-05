#ifndef SPC_CALIBRATION_DATA_FILTER_H
#define SPC_CALIBRATION_DATA_FILTER_H

#include <spc/methods/IntensityCalibrationDataEstimator.h>

namespace spc
{

class CalibrationDataFilter
{
public:
    SPC_OBJECT(CalibrationDataFilter)

    CalibrationDataFilter()
    {
    }
    void setInputCalibrationSamplesDB(SamplesDB data)
    {
        db_ = data;
    }

    void fixUniqueNormals()
    {

        std::vector<size_t> unique_cores = db_.getVectorOfUniqueSamples();
        spcForEachMacro(size_t id, unique_cores)
        {

            // just some verbosity
            if (id % 100 == 0) {
                pcl::console::print_info("core # %i\n", id);
            }

            std::vector<Sample::Ptr> current_core_points
                = db_.getDataForSampleID(id);

            size_t best_id
                = estimateBestSampleForNormals(current_core_points);

            // extract the normal for the best core point
            Eigen::Vector3f best_normal = current_core_points.at(best_id)->variantPropertyValue
                                          <Eigen::Vector3f>("normal");

            // now we force ALL the other core points to have the same normal
            spcForEachMacro(Sample::Ptr core_meas, current_core_points)
            {
                core_meas->variantPropertyValue<Eigen::Vector3f>("normal") = best_normal;
            }
        }
    }

    //! secondary parameter is the scattering angle for now
    /** \note this will overwrite any existing parameter with the same name
     */
    void recomputeScatteringAngles()
    {
        spcForEachMacro(Sample::Ptr core, db_.getSamplesDB())
        {
            Eigen::Vector3f normal = core->variantPropertyValue<Eigen::Vector3f>("normal");
            Eigen::Vector3f ray = core->variantPropertyValue<Eigen::Vector3f>("ray");
            // overwrite old measure
            core->variantPropertyValue("angle")
                = CalibrationDataEstimator::getMinimumAngleBetweenVectors(
                    normal, ray);
        }
    }

    // this is a temptative to get the best normal for a given core point
    size_t estimateBestSampleForNormals(const std::vector
                                           <Sample::Ptr> &data)
    {
        std::vector<size_t> n_neighbors = SamplesDB::extractPropertyAsVector
            <size_t>(data, "n_neighbors");
        std::vector<size_t>::iterator it
            = std::max_element(n_neighbors.begin(), n_neighbors.end());
        size_t position = std::distance(n_neighbors.begin(), it);
        return position;
    }

private:
    SamplesDB db_;
};

} // end nspace

#endif
