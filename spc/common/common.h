#ifndef SPC_COMMON_H
#define SPC_COMMON_H

#include <vector>
#include <cmath>
#include <iostream>
#include <utility>

#include <spc/methods/rbf_interpolator.h>

#include <Eigen/Core>

#include <boost/foreach.hpp>

#include <pcl/io/pcd_io.h>

#include <spc/common/common_includes.h>

void dotest();

#ifdef PCL_VER_LESS_1_7
	#include <sensor_msgs/PointCloud2.h>

	namespace pcl
	{	
		typedef sensor_msgs::PointCloud2 PCLPointCloud2;
		typedef sensor_msgs::PointField PCLPointField;
		typedef std_msgs::Header PCLHeader;


		#define fromPCLPointCloud2 fromROSMsg
		#define toPCLPointCloud2 toROSMsg
	}

#else
	#include <pcl/PCLPointCloud2.h>
#endif


#if defined(_WIN32)
 
#include <stdint.h>
 
typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;


 
#endif


//#ifdef PCL_VER_1_6_OR_OLDER
//
//#include <pcl/ros/conversions.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/PointField.h	
//typedef sensor_msgs::PointCloud2 PCLCloud;
//typedef sensor_msgs::PointField PCLScalarField;
//#define FROM_PCL_CLOUD pcl::fromROSMsg
//#define TO_PCL_CLOUD pcl::toROSMsg
//
//#else //Version 1.7.0 or newer
//
//#include <pcl/PCLPointCloud2.h>
//#include <pcl/PCLPointField.h>
//typedef pcl::PCLPointCloud2 PCLCloud;
//typedef pcl::PCLPointField PCLScalarField;
//#define FROM_PCL_CLOUD pcl::fromPCLPointCloud2
//#define TO_PCL_CLOUD pcl::toPCLPointCloud2
//
//#endif







namespace spc
{

template<typename Stype>
std::string asString(const Stype & number)
{
    return boost::lexical_cast<std::string>(number);
}

template <typename nType>
std::vector<nType> subdivideRange(const nType start, const nType end,
                                  const nType step);

template <typename nType>
std::vector<int> get_nans_id(const std::vector<nType> vector)
{
    std::vector<int> nans_id;
    for (int i = 0; i < vector.size(); ++i) {
        if (pcl_isnan(vector[i])) {
            nans_id.push_back(i);
        }
    }
    return nans_id;
}

template <typename nType>
std::vector<nType> erease_ids(const std::vector<nType> &Vector,
                              const std::vector<int> Ids)
{
    // sort indexes
    std::vector<int> ids = Ids;
    std::vector<nType> vector = Vector;
    sort(ids.begin(), ids.end());

    // remove from the higher to lower, so indexes are not changed
    for (int i = ids.size() - 1; i >= 0; i--) {
        vector.erase(vector.begin() + ids[i]);
    }
    return vector;
}

template <typename nType>
void fill_nan_rbf(std::vector<nType> &x, std::vector<nType> &y)
{
    // get nans ids
    std::vector<int> nans_id = get_nans_id(y);

     std::vector<nType> x_stripped;
         std::vector<nType> y_stripped;

    for (auto &elem : nans_id) {
    // remove this ids from the original dataset
   x_stripped = erease_ids(x, nans_id);
    y_stripped = erease_ids(y, nans_id);
    }
    // now create a rbf interpolator
    spc::RBFInterpolator<nType> rbf;
    rbf.setInputPoints(x_stripped);
    rbf.setInputValues(y_stripped);
    rbf.updateAll();

    for (auto &elem : nans_id) {
      int this_id = elem;
        nType this_x_position = x[this_id];
        Eigen::Matrix<nType, Eigen::Dynamic, 1> x_point;
        x_point.resize(1);
        x_point[0] = this_x_position;
        y[this_id] = rbf.evaluateRbf(x_point);
    }

    // now for each nan evaluate the rbf
}

template <typename Stype>
std::vector<int> getSortingIds(const std::vector<Stype> &v)
{
    //    typedef typename std::vector<Stype>::const_iterator myiter;

    //    using myiter = decltype(v.begin());

    typedef typename std::vector<Stype>::const_iterator myiter;
    typedef typename std::pair<size_t, myiter> pairT;

    //    using pairT = std::pair<size_t, myiter>;

    std::vector<pairT> order;

    size_t n = 0;
    BOOST_FOREACH(const Stype & e, v)
    {
        order.push_back(std::make_pair(n++, (myiter) & e));
    }
    std::sort(order.begin(), order.end(), [](pairT const &a, pairT const &b) {
        return *(a.second) < *(b.second);
    });

    std::vector<int> out(order.size());
    std::transform(order.begin(), order.end(), out.begin(),
                   [](pairT &pair) { return pair.first; });
    return out;
}

template <typename Stype>
std::vector<int> getFirstNSortingIds(const std::vector<Stype> &v, const int N)
{
    std::vector<int> all_ids = getSortingIds(v);
    std::vector<int> out(all_ids.begin(), all_ids.begin() + N);
    return out;
}

template <typename Stype>
std::vector<int> getLastNSortingIds(const std::vector<Stype> &v, const int N)
{

    std::vector<int> all_ids = getSortingIds(v);
    std::vector<int> out(all_ids.end() - N, all_ids.end());
    //    std::transform(end(all_ids) - N, end(all_ids), begin(out), [](int
    // &val){return val;} );

    return out;
}

} // end namespace
#endif
