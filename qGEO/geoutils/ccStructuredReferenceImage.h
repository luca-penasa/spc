#ifndef CCSTRUCTUREDREFERENCEIMAGE_H
#define CCSTRUCTUREDREFERENCEIMAGE_H

#include <vector>
#include <ccPointCloud.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>

template <typename scalarT>
struct BoundingBox3D
{
    BoundingBox3D()
    {
        min_x = -1;
        min_y = -1;
        min_z = -1;

        max_x = 1;
        max_y = 1;
        max_z = 1;
    }

    scalarT min_x;
    scalarT min_y;
    scalarT min_z;
    scalarT max_x;
    scalarT max_y;
    scalarT max_z;
};

struct Point3D
{
    Point3D(int a, int b, int c) {x = a; y = b; z = c;}

    union{

        int point[3];

        struct
        {
            int x;
            int y;
            int z;
        };
    };

};




//! contain all the informations needed for mapping a given point in the image space and vice versa
/** we keep here for each point of a cloud the pixel's x-y-z coordinate in the image space
 */
class CloudToImageMapping
{
public:
    //! vector of the ids in the original cloud
    typedef std::vector<int> idsT;
    typedef Point3D pointT;


    CloudToImageMapping() {}

    //! a list of the original ids, they will be normally ordered integers, but in some cases this could be useful
    idsT m_ids;

    //! each id in m_ids is connected with only ONE Point in the 2D domain, and this mapping is stored here
    std::vector< pointT > m_mapping;
};



class ccStructuredReferenceImage
{
public:

    ccStructuredReferenceImage();

    typedef double scalarT;
    typedef std::vector<scalarT> pixelPositionsT ;
    typedef BoundingBox3D<scalarT> bbT;


    void setXStep(scalarT step) { m_x_step = step; }
    void setYStep(scalarT step) { m_y_step = step; }
    void setZStep(scalarT step) { m_z_step = step; }

    void setBoundingBox( bbT box ) { m_bb = box; }
    void setBoundingBoxFromCloud(ccPointCloud *cloud );

    int updateAllPixelsCentersFromBB ();

    Point3D computeMappingForPoint( CCVector3 * point );

    CloudToImageMapping computeMappingForCloud(ccPointCloud * cloud);

    vtkSmartPointer<vtkImageData> mapToImage(const CloudToImageMapping map, const ccPointCloud * cloud);

private:

    int computePixelsCenters(pixelPositionsT &vector_of_centers, scalarT &min, scalarT &max, scalarT &step);
    int findNearestNeighborBF(pixelPositionsT positions, scalarT query_position, int & nearest, scalarT & sq_distance);
    int findNearestNeighborFast(const scalarT query_position, int & nearest, const scalarT min, const scalarT step);




    scalarT m_x_step;
    scalarT m_y_step;
    scalarT m_z_step;

    bbT m_bb;

    pixelPositionsT m_x_pixels_centers;
    pixelPositionsT m_y_pixels_centers;
    pixelPositionsT m_z_pixels_centers;


};

#endif // CCSTRUCTUREDREFERENCEIMAGE_H
