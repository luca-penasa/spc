
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>

#include <spc/common/common.h>
#include <spc/common/strings.h>
#include <spc/common/io_helper.h>

#include <spc/methods/linear_interpolator.h>

#include <math.h>

using namespace pcl;
using namespace pcl::console;
using namespace std;


//A strange point type to be used here:
struct PointXYZScalar
    {
        PCL_ADD_POINT4D;
        float Sc4laR897;




        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    POINT_CLOUD_REGISTER_POINT_STRUCT   (PointXYZScalar,
                         (float, x, x)
                         (float, y, y)
                         (float, z, z)
                         (float, Sc4laR897, Sc4laR897)


    )

int main(int argc, char* argv[])
{
    ////////// PARSE THINGS /////////////////
    //cloud filename
    vector<int> incloud_id = parse_file_extension_argument(argc, argv, ".pcd");
    if (incloud_id.size() == 0)
    {
        print_error("You should pass a point cloud to be corrected\n");
        return -1;
    }
    string incloud_pathname = argv[incloud_id[0]]; //first one is input

    //tseries filename
    vector<int> tseries_id = parse_file_extension_argument(argc, argv, ".txt");
    if (tseries_id.size() == 0)
    {
        print_error("A .txt file with correction curve is needed\n");
        return -1;
    }
    string tseries_pathname = argv[tseries_id[0]]; //first one is input


    //int field name
    string intensity_field_name = string("intensity"); //default value
    parse_argument(argc, argv, "-if", intensity_field_name);

    Eigen::Vector3f center;
    center.fill(0.0);

    //center
    parse_3x_arguments(argc, argv, "-c", center(0), center(1), center(2));

    //save full correction curve?
    bool save_full_correction_curve = false;
    parse_argument(argc, argv, "-s", save_full_correction_curve);

    //derived filename etc
    boost::filesystem::path full_in_path(incloud_pathname);

    std::string filename = full_in_path.filename().c_str();

    filename = spc::stripExtension(filename);

    ////////// PRINT INFOS //////////////////
    //TODO


    ////////// LOAD THINGS //////////////////
    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2);

    if (pcl::io::loadPCDFile(incloud_pathname, *cloud) < 0)
    {
        print_error("Cannot find %s point cloud file\n", incloud_pathname.c_str());
        return -1;
    }

    //load the time-series
    vector< vector<float> > tseries;
    if (spc::loadCSVTimeSeries(tseries_pathname, string(" "), tseries) < 0)
    {
        print_error("Cannot find %s correction curve file\n", tseries_pathname.c_str());
        return -1;
    }



    ////////// DO SOME CHECKS ////////////////
    if (getFieldIndex(*cloud, intensity_field_name.c_str()) < 0)
    {
        print_error("Cannot find %s scalar field\n", intensity_field_name.c_str());
        return -1;
    }

    if ((getFieldIndex(*cloud, "x") < 0) || (getFieldIndex(*cloud, "y") <0) || (getFieldIndex(*cloud, "z") <0))
    {
        print_error("Cannot find x,y,z geometric fields\n");
        return -1;
    }


    ///////// TRANSFORM TO RIGHT FORMAT ////////////
    cloud->fields[getFieldIndex(*cloud, intensity_field_name.c_str())].name = "Sc4laR897";
    PointCloud<PointXYZScalar>::Ptr pcl_cloud (new PointCloud<PointXYZScalar>);

    fromROSMsg(*cloud, *pcl_cloud);

    ///////// GET DATA AND INFOS FROM TSERIES ////////////
    vector <float> correction_d = tseries[0];
    vector <float> correction_i = tseries[1];

    float step = correction_d[1] - correction_d[0];
    float min_d = *std::min_element(correction_d.begin(), correction_d.end());
    float max_d = *std::min_element(correction_d.begin(), correction_d.end());

    //////// COMPUTE DISTANCES FOR THE CLOUD /////////////
    vector<float> distances(pcl_cloud->size());
    std::transform(pcl_cloud->begin(), pcl_cloud->end(), distances.begin(),
                                           [&](PointXYZScalar & point){return sqrt(
                                              (point.x - center(0)) * (point.x - center(0)) +
                                              (point.y - center(1)) * (point.y - center(1)) +
                                              (point.z - center(2)) * (point.z - center(2)) ) ;});

    float min_d_cloud = *std::min_element(distances.begin(), distances.end());
    float max_d_cloud = *std::min_element(distances.begin(), distances.end());

    /////// PRINT INFOS ON WHAT YOUR ARE GOING TO DO //////////////
    // TODO - you are going to detrend a cloud with min, max using a curve with min-max - could be risky.

    ///////// LINEAR INTERPOLATION ////////////////
    spc::LinearInterpolator<float> interpolator;
    interpolator.setXY(correction_i, step, min_d);
    interpolator.setNewX(distances);
    interpolator.setDegreeForOutOfBoundsInterpolator(1); //linear
    interpolator.compute();
    auto interpolated_intensities = interpolator.getNewY();

    //eventually save the correction curve
    if (save_full_correction_curve)
    {
        print_highlight("Saving full correction curve - this may take a while.\n");
        vector< vector<float> > data = {distances, interpolated_intensities};


        string save_fn;
        ///// SAVE RESULTS //////
        if (full_in_path.parent_path().empty())
            save_fn = filename + "_full_correction_curve.txt";
        else
            save_fn = full_in_path.parent_path().c_str() + string("/") + filename + "_full_correction_curve.txt";

        spc::saveAsCSV(save_fn, " ", data, 10);


    }

    //////// APPLY CORRECTION TO THE CLOUD ////////////
    size_t counter = 0;
    for (auto &point : *pcl_cloud)
        point.Sc4laR897 /= interpolated_intensities[counter++];

    //////// CONVERT BACK CLOUD /////////////////
    sensor_msgs::PointCloud2::Ptr out_cloud (new sensor_msgs::PointCloud2);
    toROSMsg(*pcl_cloud, *out_cloud);
    //change scalar field name
    out_cloud->fields[getFieldIndex(*out_cloud, "Sc4laR897")].name = "corrected_int";

    //also for the original cloud
    cloud->fields[getFieldIndex(*cloud, "Sc4laR897")].name = intensity_field_name.c_str();

    /////// MERGE ORIGINAL DATASET WITH THIS CLOUD /////////
    sensor_msgs::PointCloud2::Ptr complete_out_cloud (new sensor_msgs::PointCloud2);
    concatenateFields(*cloud, *out_cloud, *complete_out_cloud);


    string save_fn;
    ///// SAVE RESULTS //////
    if (full_in_path.parent_path().empty())
        save_fn = filename + "_distance_corrected.pcd";
    else
        save_fn = full_in_path.parent_path().c_str() + string("/") + filename + "_distance_corrected.pcd";

    spc::savePCDBinaryCompressed(save_fn, *complete_out_cloud);

//    std::cout << full_in_path.parent_path().c_str() + "/" + filename + "_distance_corrected.pcd" << std::endl;





    return 1;
}
