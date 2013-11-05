#include "ccStructuredReferenceImage.h"
#include <cmath>
#include <limits>

#include <iostream>



ccStructuredReferenceImage::ccStructuredReferenceImage()
{
}

int
ccStructuredReferenceImage::computePixelsCenters(pixelPositionsT &vector_of_centers, scalarT &min, scalarT &max, scalarT &step)
{
    if (max < min)
    {
        return -1;
    }

    if (step <= 0)
    {
        return -1;
    }

    vector_of_centers.clear(); //reset

    //first pixel will always be centered on min
    scalarT range = max - min;
    int n_pixels = ceil( (double) range / (double) step); //approximate to next integer

    vector_of_centers.resize(n_pixels);

    for (int i=0; i < n_pixels; ++i)
    {
        vector_of_centers.at(i) = min + step*i;
    }

    return 1;

}

int
ccStructuredReferenceImage::updateAllPixelsCentersFromBB ()
{
    if(!this->computePixelsCenters(m_x_pixels_centers, m_bb.min_x, m_bb.max_x, m_x_step))
        return -1;
    if(!this->computePixelsCenters(m_y_pixels_centers, m_bb.min_y, m_bb.max_y, m_y_step))
        return -1;
    if(!this->computePixelsCenters(m_z_pixels_centers, m_bb.min_z, m_bb.max_z, m_z_step))
        return -1;

    return 1;
}

void
ccStructuredReferenceImage::setBoundingBoxFromCloud( ccPointCloud * cloud )
{
    //PointCoordinateType is float!
    PointCoordinateType mins[3];
    PointCoordinateType maxs[3];
    cloud->getBoundingBox(mins, maxs);

    bbT bb;
    //Also cast the PointCoordinateType to the scalarT used in this class
    bb.min_x = (scalarT) mins[0];
    bb.min_y = (scalarT) mins[1];
    bb.min_z = (scalarT) mins[2];

    bb.max_x = (scalarT) maxs[0];
    bb.max_y = (scalarT) maxs[1];
    bb.max_z = (scalarT) maxs[2];

    //and set the BB
    setBoundingBox(bb);
}

Point3D ccStructuredReferenceImage::computeMappingForPoint( CCVector3 * point )
{
// note that we does not check if the point is within or not the current BB!
// points outside BB will be mapped to the pixels at borders!
    int x_nn, y_nn, z_nn;

    findNearestNeighborFast( (*point)[0], x_nn, m_bb.min_x, m_x_step);
    findNearestNeighborFast( (*point)[1], y_nn, m_bb.min_y, m_y_step);
    findNearestNeighborFast( (*point)[2], z_nn, m_bb.min_z, m_z_step);

    return Point3D(x_nn, y_nn, z_nn);

}

CloudToImageMapping ccStructuredReferenceImage::computeMappingForCloud(ccPointCloud *cloud)
{
    CloudToImageMapping mapping;
    int n_points = cloud->size();

    // create the vector of indices
    for (int i = 0; i < n_points; ++i)
    {
        mapping.m_ids.push_back(i);
    }

    //now for each point do the mapping
    for (int i = 0; i < n_points; ++i)
    {
        CCVector3 point;
        cloud->getPoint(i, point);
        mapping.m_mapping.push_back( computeMappingForPoint( &point) );
    }

    return mapping;
}


int
ccStructuredReferenceImage::findNearestNeighborBF(pixelPositionsT positions, scalarT query_position, int & nearest, scalarT & sq_distance)
{
    nearest = 0;
    sq_distance = 1e50;

    for (int i = 0; i < positions.size(); ++i)
    {
        scalarT new_distance = positions.at(i) - query_position;
        new_distance *= new_distance;

        if (new_distance < sq_distance)
        {
            sq_distance = new_distance;
            nearest = i;
        }
    }
    return 1;

}

int
ccStructuredReferenceImage::findNearestNeighborFast(const scalarT query_position, int & nearest, const scalarT min, const scalarT step)
{
    //id = ( position - min ) / step
    nearest = round( (query_position - min) * 1/step );

    return 1;
}

vtkSmartPointer<vtkImageData>
ccStructuredReferenceImage::mapToImage(const CloudToImageMapping map, const ccPointCloud * cloud)
{

    vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();

    image->SetDimensions(m_x_pixels_centers.size() + 1, m_y_pixels_centers.size() +1 , m_z_pixels_centers.size() +1);
    image->SetNumberOfScalarComponents(1);
    image->SetScalarTypeToInt();
    image->AllocateScalars();
    int extent[6];
    image->GetExtent(extent);

    std::cout << "Extent" << std::endl;
    for (int i = 0; i < 6; ++i)
    {
        std::cout << extent[i] << std::endl;
    }

    for (int t = 0; t < map.m_ids.size(); ++t)
    {
        //int id = map.m_ids.at(t);
        int i, j, k;
        i = map.m_mapping.at(t).x;
        j = map.m_mapping.at(t).y;
        k = map.m_mapping.at(t).z;

        std::cout << i << " " << j << " " << k << std::endl;

        int* pixel = static_cast<int*>(image->GetScalarPointer(i,j,k));
        *pixel = 1;


    }

    std::cout << "FINISHED" << std::endl;

    return image;

}
