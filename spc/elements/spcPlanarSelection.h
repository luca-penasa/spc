#ifndef SPCPLANARSELECTION_H
#define SPCPLANARSELECTION_H

#include <spc/elements/element_base.h>
#include <spc/elements/spcPCLCloud.h>
#include <spc/elements/plane.h>
#include <pcl/io/pcd_io.h> //for debug only

#include <boost/serialization/shared_ptr.hpp>
#include <boost/foreach.hpp>

namespace spc
{



class spcPlanarSelection: public spcElementBase
{
public:

    typedef typename boost::shared_ptr<spcPlanarSelection> Ptr;
    typedef typename boost::shared_ptr<const spcPlanarSelection> ConstPtr;


    typedef typename pcl::PointCloud<pcl::PointXYZ> cloudT;

    spcPlanarSelection();

    spcPlanarSelection(const spcPlanarSelection & el)
    {
        verts_3d_ = el.getVertices();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr getVertices() const
    {
        return verts_3d_;
    }

    void setVertices(pcl::PointCloud<pcl::PointXYZ>::Ptr verts)
    {
        verts_3d_ = verts;
        updateProjectionPlane();
        updatePolyVertices();
    }

    float getMaxDistance() const
    {
        return max_distance_;
    }

    void setMaxDistance(float d)
    {
        max_distance_ = d;
    }

    void setInputCloud (spcGenericCloud::Ptr cloud)
    {
        pcl::console::print_info("Settet input cloud! with %i points\n", cloud->size());
        in_cloud_ = cloud;
        updateProjectedCloud();
        updateIndices();
    }

    std::vector<int> getIndices()
    {
        return indices_;
    }

    void updateIndices()
    {
        pcl::console::print_debug("Updating indices \n");

        for (int i = 0 ; i < projected_cloud_.size(); ++i)
        {
            if (projected_cloud_.at(i).z <= max_distance_)
            {
                //                pcl::console::print_debug("less than min distance \n");
                pcl::PointXY p;

                p.x = projected_cloud_.at(i).x;
                p.y = projected_cloud_.at(i).y;
                if (isPointInPoly(p, verts_2d_) == 1)
                {
                    //                    pcl::console::print_debug("found point inside \n");
                    indices_.push_back(i);
                }
            }
            //            else
            //                pcl::console::print_debug("too distant\n");
        }
    }

    pcl::PointCloud<pcl::PointXYZ> getInside(spcGenericCloud::Ptr in_cloud)
    {
        setInputCloud(in_cloud);
        updateProjectedCloud();
        updatePolyVertices();
        updateIndices();

        std::vector<int> indices = getIndices();

        std::cout << "found " <<  indices.size() << " valid indices\n" << std::endl;

        // now filter out

        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.resize(indices.size());

        BOOST_FOREACH(int id, indices)
        {
            Vector3f point = in_cloud->getPoint(id);
            pcl::PointXYZ p;
            p.x = point(0);
            p.y = point(1);
            p.z = point(2);
            cloud.push_back(p);
        }

        return cloud;


    }



protected:

    void updateProjectedCloud()
    {
        pcl::console::print_debug("Updating projected cloud.\n");
        projected_cloud_ = in_cloud_->applyTransform(proj_plane_.get2DArbitraryRefSystem());
        pcl::console::print_info("projected cloud has %i points\n", projected_cloud_.size());
        pcl::io::savePCDFileBinary("/home/luca/tmp.pcd", projected_cloud_);
    }

    void updateProjectionPlane()
    {
        pcl::console::print_info("Updating projection plane.\n");
        proj_plane_.positionFromCentroid(*verts_3d_);
        spcNormal3D n;
        n.normalFromBestFit(*verts_3d_);
        proj_plane_.setNormal(n.getNormal());
        pcl::console::print_info("Normal now is %f %f %f.\n", n.getNormal()(0), n.getNormal()(1), n.getNormal()(2));

    }




    /// http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
    int isPointInPoly(const pcl::PointXY P,
                      const std::vector<pcl::PointXY>& polyVertices)
    {
        int nvert = polyVertices.size();
        int i, j, c = 0;
        for (i = 0, j = nvert-1; i < nvert; j = i++) {
            if ( ((polyVertices[i].y>P.y) != (polyVertices[j].y>P.y)) &&
                 (P.x < (polyVertices[j].x-polyVertices[i].x) * (P.y-polyVertices[i].y) / (polyVertices[j].y-polyVertices[i].y) + polyVertices[i].x) )
                c = !c;
        }
        //        pcl::console::print_debug("Tested in poly: %i", c);
        return c;

    }





    void updatePolyVertices()
    {

        pcl::console::print_info("updating poly vertices\n");
        verts_2d_.clear();

        //         = verts_3d_.applyTransform(proj_plane_.get2DArbitraryRefSystem());

        pcl::console::print_info("there are %i initial poly verts\n", verts_3d_->size());
        pcl::PointCloud<pcl::PointXYZ> projected = spcPCLCloud<pcl::PointXYZ>(verts_3d_).applyTransform(proj_plane_.get2DArbitraryRefSystem());


        for (int i = 0 ; i < projected.size(); ++i)
        {
            pcl::PointXY p2d;
            pcl::PointXYZ p3d;
            p3d = projected.at(i);

            p2d.x = p3d.x;
            p2d.y = p3d.y;

            verts_2d_.push_back(p2d);
        }

        pcl::console::print_info("there are %i poly verts\n", verts_2d_.size());
    }



    ///
    /// \brief m_points the points defining a polyline in 3D
    ///
    pcl::PointCloud<pcl::PointXYZ>::Ptr verts_3d_;
    ///
    /// \brief in_cloud_ it the input cloud to be segmented
    ///
    spcGenericCloud::Ptr in_cloud_;

    /// things that will be auto-updated
    std::vector<pcl::PointXY> verts_2d_;

    /// plane on which to perform the projection
    spcPlane proj_plane_;

    /// a distance limit from the projective plane
    float max_distance_;

    /// good indexes (inside)
    std::vector<int> indices_;

    /// the input cloud projected in the 2d ref system
    /// z is the distance from the proj_plane_
    pcl::PointCloud<pcl::PointXYZ> projected_cloud_;



protected:



    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(verts_3d_);
        ar & BOOST_SERIALIZATION_NVP(max_distance_);
    }






};


}//end nspace

#endif // SPCPLANARSELECTION_H
