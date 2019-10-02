#ifndef FLANN_BUG_H
#define FLANN_BUG_H

#include <spc/elements/ElementBase.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

#include <spc/methods/common.h>
#include <pcl/search/impl/flann_search.hpp>
#include <boost/make_shared.hpp>


class CloudProxy
{
public:
    CloudProxy(pcl::PCLPointCloud2::Ptr cloud)
    {
        cloud_ = cloud;
    }

    void updateSearcher()
    {
        pcl::search::FlannSearch
            <pcl::PointXYZ>::Ptr s(new pcl::search::FlannSearch<pcl::PointXYZ>);

        s->setInputCloud(this->getAsXYZ());

        searcher_ = s;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr getAsXYZ()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr out (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*cloud_, *out);

        return out;
    }

    pcl::search::FlannSearch<pcl::PointXYZ>::Ptr getSearcher()
    {
        return searcher_;
    }

    pcl::PCLPointCloud2::Ptr cloud_;

    pcl::search::FlannSearch<pcl::PointXYZ>::Ptr searcher_;
};


#endif // FLANN_BUG_H
