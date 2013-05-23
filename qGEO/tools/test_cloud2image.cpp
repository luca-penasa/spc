#include <ccStructuredReferenceImage.h>
#include <iostream>

#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageWriter.h>
#include <vtkDataSetWriter.h>

int main()
{
    ccStructuredReferenceImage referencer;
    referencer.setXStep(1);
    referencer.setYStep(1);
    referencer.setZStep(1);

    std::cout << "Here" << std::endl;

    CCLib::ChunkedPointCloud * cloud = new CCLib::ChunkedPointCloud;


    int test_n = 30;
    //cloud->resize(1);
    cloud->reserve(2*test_n*2*test_n);


    int count = 1;

    //create a cloud
    for (int i = -test_n; i < test_n; ++i)
    {

            for (int k = -test_n; k < test_n; ++k)
            {
                CCVector3 point;
                point.x = i;
                point.y = k;
                point.z = (i + k)/2 +0.02*i*k + 0.2*i*i;

                cloud->addPoint(point);

                //std::cout << i << j << k << " -> " << count << std::endl;
                count++;


            }
    }



    std::cout << "Using cloud with " << count - 1 << " points" << std::endl;

    referencer.setBoundingBoxFromCloud(cloud);

    referencer.updateAllPixelsCentersFromBB();


    std::cout << "MAPPING..." << std::endl;
    CloudToImageMapping map = referencer.computeMappingForCloud(cloud);

    std::cout << "To Image..." << std::endl;
    vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();

    image = referencer.mapToImage(map, cloud);

    vtkSmartPointer<vtkDataSetWriter> w = vtkSmartPointer<vtkDataSetWriter>::New();
    w->SetInput(image);
    w->SetFileName("/home/luca/Desktop/tmp.vtk");
    w->Write();





//    for (int i = 0; i < 20; ++i)
//    {
//        std::cout << map.m_mapping.at(i).x << " " << map.m_mapping.at(i).y << " " << map.m_mapping.at(i).z << std::endl;
//    }

}
