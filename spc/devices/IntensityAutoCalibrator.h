#ifndef INTENSITYAUTOCALIBRATOR_H
#define INTENSITYAUTOCALIBRATOR_H

#include <vector>
#include <string>

#include <spc/elements/generic_cloud.h>

#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>

#include <pcl/io/pcd_io.h>
#include <spc/methods/compute_eigen_indices.h>

#include <spc/common/geometry.h>
#include <spc/elements/plane.h>

#include <numeric>
#include <algorithm>

#include <boost/any.hpp>

#include <boost/spirit/home/support/detail/hold_any.hpp>


namespace spc
{



class CorePointMeasurements
{
public:
    typedef  std::map<std::string, boost::any> DataHolderType;
    typedef  std::pair<std::string, boost::any> PairType;

    //!empty const
    CorePointMeasurements() {}

    boost::any & value(const std::string name)
    {
        return datadb[name];
    }

    DataHolderType getDB()
    {
        return datadb;
    }

    DataHolderType getDB() const //const version
    {
        return datadb;
    }

    void writeLine(std::ostringstream &stream)
    {
        stream    << boost::any_cast<float> (this->value("avg_distance")) << " "
                  << boost::any_cast<float> (this->value("avg_intensity")) << " "
                  << boost::any_cast<float> (this->value("scattering_angle")) << " "
                  << boost::any_cast<size_t> (this->value("n_neighbors")) << std::endl;
    }



private:
    DataHolderType datadb;


};


std::ostream& operator<<(std::ostream& os, const CorePointMeasurements& obj)
{
    // write obj to stream
    BOOST_FOREACH (CorePointMeasurements::PairType iter, obj.getDB())
    {
        os << iter.first << ": ";

        if (iter.second.type() == typeid(std::vector<int>))
        {
            std::vector<int>  ids = boost::any_cast<std::vector<int> > (iter.second);

            BOOST_FOREACH (int i, ids)
                    os << i << " " ;

            os << std::endl;
        }

        if (iter.second.type() == typeid(std::vector<float>))
        {
            std::vector<float>  ids = boost::any_cast<std::vector<float> > (iter.second);

            BOOST_FOREACH (float i, ids)
                    os << i << " " ;

            os << std::endl;
        }

        if (iter.second.type() == typeid(float))
        {
            float  data = boost::any_cast<float> (iter.second);
            os << data << std::endl;
        }

        if (iter.second.type() == typeid(int))
        {
            int  data = boost::any_cast<int> (iter.second);
            os << data << std::endl;
        }

        if (iter.second.type() == typeid(size_t) )
        {
            size_t  data = boost::any_cast<size_t> (iter.second);
            os << data << std::endl;
        }

        if (iter.second.type() == typeid(std::string) )
        {
            std::string  data = boost::any_cast<std::string> (iter.second);
            os << data << std::endl;
        }

        if (iter.second.type() == typeid(Eigen::Vector3f))
        {
            Eigen::Vector3f  data = boost::any_cast<Eigen::Vector3f> (iter.second);
            os << data.transpose() << std::endl;
        }
    }
    return os;
}



class CalibrationDataDB
{
public:
    //! empty constructor
    CalibrationDataDB() {}

    void pushBack(CorePointMeasurements data)
    {
        db_.push_back(data);
    }

    void printOutStuff()
    {
        BOOST_FOREACH(CorePointMeasurements entry, db_)
                std::cout << entry << std::endl;
    }

    CalibrationDataDB getDataForCorePointID(size_t core_point_id)
    {
        CalibrationDataDB new_db;
        BOOST_FOREACH(CorePointMeasurements core, db_)
        {

            if (boost::any_cast<size_t> (core.value("core_point_id")) == core_point_id)
            {
                new_db.pushBack(core);
            }

        }

        return new_db;
    }

    CalibrationDataDB refineNormalEstimations() const
    {
        CalibrationDataDB new_db;


    }

    void writeFullData(std::ostringstream &stream)
    {
        BOOST_FOREACH(CorePointMeasurements meas, db_)
        {
            meas.writeLine(stream);
        }
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
    std::vector<CorePointMeasurements> db_;
};




class IntensityAutoCalibrator
{
public:
    IntensityAutoCalibrator();

    void setInputClouds(std::vector<std::string> cloud_names);


    void setInputCorePoints(std::string core_points_name);


    int compute()
    {
        //load the core points cloud
        this->loadCorePointsCloud();

        //set up a db
        db_ = CalibrationDataDB(); //ensure is clean, we should reset instead

        current_cloud_id_ = 0;
        BOOST_FOREACH (std::string fname, input_fnames_)
        {
            current_cloud_name_ = fname;
            pcl::console::print_info("started working on %s ", fname.c_str());

            //load the cloud and create what we need
            this->loadCloudAndCreateSearcher(fname);

            //now we cycle on the core points for the current cloud
            for (size_t i = 0; i < core_points_cloud_->size(); ++i)
            {
                CorePointMeasurements measurement = this->computeCorePointParameters(i, search_radius_);
                db_.pushBack(measurement);
            }
            current_cloud_id_ += 1;
        }
    }

    CalibrationDataDB getCalibrationDB();


    void setSearchRadius(const float rad);



    //on current cloud!
    CorePointMeasurements computeCorePointParameters(const size_t core_point_id,
                                                     const float search_radius )
    {
        CorePointMeasurements out;

        //get the point coordinates from core point cloud
        pcl::PointXYZI point = core_points_cloud_->at(core_point_id);

        std::vector<float> dists;
        std::vector<int> ids;

        current_cloud_searcher_->radiusSearch(point, search_radius, ids, dists );


        //compute normal and average distance

        float nx;
        float ny;
        float nz;
        float lam0;
        float lam1;
        float lam2;

        spc::computePointNormal (*current_point_cloud_, ids,
                                 nx, ny, nz, lam0, lam1, lam2);

        if (ids.size() <= 3)
        {
            nx = std::numeric_limits<float>::quiet_NaN();
            ny = std::numeric_limits<float>::quiet_NaN();
            nz = std::numeric_limits<float>::quiet_NaN();
            lam0 = std::numeric_limits<float>::quiet_NaN();
            lam1 = std::numeric_limits<float>::quiet_NaN();
            lam2 = std::numeric_limits<float>::quiet_NaN();

        }

        Eigen::Vector4f _c; //tmp_var
        pcl::compute3DCentroid(*current_point_cloud_, ids, _c);

        if (ids.size() == 0)
        {
            _c(0) = std::numeric_limits<float>::quiet_NaN();
            _c(1) = std::numeric_limits<float>::quiet_NaN();
            _c(2) = std::numeric_limits<float>::quiet_NaN();
        }


        Eigen::Vector3f c (_c(0),_c(1),_c(2));

        auto n_vec = Eigen::Vector3f(nx, ny, nz);
        auto lambdas = Eigen::Vector3f(lam0, lam1, lam2);

        //number and which neighbors used for this dataset
        out.value("n_neighbors") = ids.size();
        out.value("neighbors") = ids;
        out.value("neighbors_dists") = dists;

        //normal and goodnees of fit
        out.value("normal") = n_vec;
        out.value("lambdas") = lambdas;

        //the local centroid
        out.value("local_centroid") = c;

        Eigen::Vector3f pos = Eigen::Vector3f(current_sensor_center_(0), current_sensor_center_(1), current_sensor_center_(2));

        Eigen::Vector3f ray = c - pos;

        //position of sensor
        out.value("sensor_position") = pos;

        // ray from sensor to the center of mass of the core point
        out.value("ray") = ray;

        // the id of the core point
        out.value("core_point_id") = core_point_id;

        // cloud on which it was computed
        out.value("cloud_name") = current_cloud_name_;

        //a progressive id for this cloud
        out.value("cloud_id") = current_cloud_id_;

        //the average distance of the core point (its center of mass) from the sensor
        out.value("avg_distance") = ray.norm();

        // the local average intensity
        out.value("avg_intensity") = this->getAverageIntensity(*current_point_cloud_, ids);

        //the scattering angle
        out.value("scattering_angle") = IntensityAutoCalibrator::getMinimumAngleBetweenVectors(n_vec, ray);

        return out;

    }


    static float getMinimumAngleBetweenVectors(const Eigen::Vector3f x_, const Eigen::Vector3f y_)
    {
        //ensure they are normalized
        Eigen::Vector3f x = x_ / x_.norm();
        Eigen::Vector3f y = y_ / y_.norm();

        float cosTheta = x.dot(y);
        float theta = acos(std::min(fabs(cosTheta),1.0)) ;

        theta *= 180.0/M_PI;

        return theta;
    }

    static float getAverageIntensity(const pcl::PointCloud<pcl::PointXYZI> & cloud, std::vector<int> ids)
    {
        if (ids.size() == 0)
            return std::numeric_limits<float>::quiet_NaN();

        float sum = 0;
        BOOST_FOREACH(int id, ids)
                sum += cloud.at(id).intensity ;

        sum /= ids.size();

        return sum;
    }

    void loadCloudAndCreateSearcher(const std::string fname)
    {
        pcl::console::print_info("loading cloud\n");

        pcl::PCLPointCloud2::Ptr in_cloud (new pcl::PCLPointCloud2);

        Eigen::Vector4f center;
        Eigen::Quaternionf orientation;

        //LOAD
        pcl::io::loadPCDFile(fname, *in_cloud, center, orientation);

        //convert to a PointCloud type object
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2(*in_cloud, *cloud);

        pcl::console::print_info("cloud loaded\n");
        //TODO check that the intensity fields exists, please

        pcl::console::print_info("creating flann index\n");
        //now create the searcher
        pcl::search::FlannSearch<pcl::PointXYZI>::Ptr searcher (new pcl::search::FlannSearch<pcl::PointXYZI>);
        searcher->setInputCloud(cloud);

        pcl::console::print_info("flann index ok\n");
        current_point_cloud_ = cloud;
        current_cloud_searcher_ = searcher;

        current_sensor_center_ = center;
        current_sensor_orientation_ = orientation;
    }

    void loadCorePointsCloud()
    {
        pcl::console::print_info("loading the core points cloud\n");
        pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile(input_core_points_fname_, *in_cloud);
        core_points_cloud_ = in_cloud;
        pcl::console::print_info("core point cloud ok\n");
    }

    size_t getNumberOfClouds() const
    {
        return input_fnames_.size();
    }



private:
    //! list of pcd files to use for calibration
    std::vector<std::string> input_fnames_;

    //! the core points pcd file
    std::string input_core_points_fname_;

    //! here will be store the core points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr core_points_cloud_;

    //! these pointers will be dynamically changed during execution
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_point_cloud_;

    //! the current searcher - updated during execution
    pcl::search::FlannSearch<pcl::PointXYZI>::Ptr current_cloud_searcher_;

    //! current sensor position
    Eigen::Vector4f current_sensor_center_;

    //! current sensor orientation - not really needed here
    Eigen::Quaternionf current_sensor_orientation_;


    float search_radius_;

    size_t min_n_points_;

    CalibrationDataDB db_;

    int current_cloud_id_;

    std::string current_cloud_name_;




};


}
#endif // INTENSITYAUTOCALIBRATOR_H
