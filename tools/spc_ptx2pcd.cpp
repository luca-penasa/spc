 /* 
	*  Author: Justin Rosen
	*  Email: jmylesrosen@gmail.com
	*  Web: www.justinrosen.com
	*/
 
 // C++
 // ---------------------------------------------------------------------------
 #include <stdio.h>
 #include <stdlib.h>
 
 #include <iostream>
 #include <fstream>
 #include <string>
 
 using namespace std;
 
 // PCL
 // ---------------------------------------------------------------------------
 #include <pcl/point_types.h>
 #include <pcl/point_cloud.h>
 #include <pcl/io/pcd_io.h>
 
 #include "pcl/common/transforms.h"
 #include "pcl/common/eigen.h"
 #include "pcl/common/angles.h"
 
 #include <pcl/search/kdtree.h>
 #include <pcl/features/normal_3d.h>
 #include <spc/core/logging.h>
 using namespace pcl;
 using namespace pcl::search;
 
 // Eigen
 // ---------------------------------------------------------------------------
 
 #include <spc/core/spc_eigen.h>
 
 using namespace Eigen;
 
 
 // PTX File Format
 // ---------------------------------------------------------------------------
 
 /*
	*   Number of points per column
	*   Number of points per row
	*   X Y Z (Scanner location)
	*   R3 R3 R3
	*   R3 R3 R3
	*   R3 R3 R3 (3x3 rotation matrix - typically identity)
	*   R4 R4 R4 R4
	*   R4 R4 R4 R4
	*   R4 R4 R4 R4
	*   R4 R4 R4 R4 (4x4 rotation matrix - typically identity)
	*   X Y Z Intensity R G B (the coordinate of the point, the intensity of the reflection, and the c X Y Z Intensity R G B
	*   X Y Z Intensity R G B
	*   ...
	*   Repeat N Scans
	*   ...
	*/
 
 // Custom Point Types
 // ---------------------------------------------------------------------------
 struct PointXYZIRGB
 {
	 PCL_ADD_POINT4D;
	 union
	 {
		 struct
		 {
			 float intensity;
		 };
		 float data_i[4];
		 
		 union
		 {
			 struct
			 {
				 uint8_t b;
				 uint8_t g;
				 uint8_t r;
				 uint8_t _unused;
			 };
			 float rgb;
		 };
		 uint32_t rgba;
	 };
	 
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 } EIGEN_ALIGN16;
 
 struct PointXYZIRGBNormal
 {
	 PCL_ADD_POINT4D;
	 PCL_ADD_NORMAL4D;
	 union
	 {
		 struct
		 {
			 float intensity;
			 float curvature;
		 };
		 float data_c[4];
		 
		 union
		 {
			 struct
			 {
				 uint8_t b;
				 uint8_t g;
				 uint8_t r;
				 uint8_t _unused;
			 };
			 float rgb;
		 };
		 uint32_t rgba;
	 };
	 
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 } EIGEN_ALIGN16;
 
 POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRGB,
																		(float, x, x)
																		(float, y, y)
																		(float, z, z)
																		(float, intensity, intensity)
																		(float, rgb, rgb))
 
 POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRGBNormal,
																		(float, x, x)
																		(float, y, y)
																		(float, z, z)
																		(float, normal_x, normal_x)
																		(float, normal_y, normal_y)
																		(float, normal_z, normal_z)
																		(float, curvature, curvature)
																		(float, intensity, intensity)
																		(float, rgb, rgb))
 
 // Application
 // ---------------------------------------------------------------------------
 int ptx2pcd(string ptxPath, string pcdPath){
	 ifstream ptxFile;
	 
	 PointCloud<PointXYZIRGB> cloud;
	 PointCloud<PointXYZINormal> cloudi;
	 
	 int width;
	 int height;
	 
	 string line;
	 Vector3f loc;
	 Vector3f rot;
	 Matrix3f mx3;
	 Matrix4f mx4;
	 
	 Vector3f pt;
	 Vector3f center;
	 float i;
	 int r, g, b;
	 int count = 0;
	 int linecount = 0;
	 
	 ptxFile.open(ptxPath.c_str());
	 
	 // Read width and height
	 ptxFile >> width >> height;
	 
	 // Iterate over each scan in file
	 while(ptxFile.good()){
		 
		 // Reset cloud data
		 cloud.width = width*height;
		 cloud.height = 1;
		 cloud.is_dense = false;
		 cloud.points.resize(cloud.width * cloud.height);
		 
		 // Read scanner location
		 ptxFile >> loc.x() >> loc.y() >> loc.z();
		 
		 // Info
		 cout << "Cloud #: " << count << endl;
		 cout << "Size: " << width*height << " [" << width << "x" << height << "]" << endl;
		 cout << "Position: " << loc.x() << ", " << loc.y() << ", " << loc.z() << endl;
		 
		 // Read 3x3 rotation matrix
		 ptxFile >> mx3(0, 0) >> mx3(0, 1) >> mx3(0, 2);
		 ptxFile >> mx3(1, 0) >> mx3(1, 1) >> mx3(1, 2);
		 ptxFile >> mx3(2, 0) >> mx3(2, 1) >> mx3(2, 2);
		 
		 // Read 4x5 rotation matrix
		 ptxFile >> mx4(0, 0) >> mx4(0, 1) >> mx4(0, 2) >> mx4(0, 3);
		 ptxFile >> mx4(1, 0) >> mx4(1, 1) >> mx4(1, 2) >> mx4(1, 3);
		 ptxFile >> mx4(2, 0) >> mx4(2, 1) >> mx4(2, 2) >> mx4(2, 3);
		 ptxFile >> mx4(3, 0) >> mx4(3, 1) >> mx4(3, 2) >> mx4(3, 3);
		 
		 // Read rest of last line
		 getline(ptxFile, line);
		 
		 // Create transformation matrix
		 mx4.transposeInPlace();
		 Affine3f transform(mx4);
		 
		 // Info
		 center = transform * Vector3f(0, 0, 0);
		 cout << "Center: " << center.x() << ", " << center.y() << ", " << center.z() << endl;
		 
		 vector<string> tokens;
		 size_t idx = 0;
		 
		 // Whether or not the scan has RGB data
		 bool hasColor = false;
		 
		 // Iterate over all points
		 for (size_t n = 0; n < width*height; ++n){
			 getline(ptxFile, line);
			 if(!ptxFile.good()){
				 cerr << "ERROR: Reading File " << linecount << endl;
				 return 1;
			 }
			 
			 linecount++;
			 
			 // Tokenize line
			 tokens.clear();
			 istringstream iss(line);
			 copy(istream_iterator<string>(iss),
						istream_iterator<string>(),
						back_inserter<vector<string> >(tokens));
			 
			 // Get XYZI point data
			 pt.x() = atof(tokens[0].c_str());
			 pt.y() = atof(tokens[1].c_str());
			 pt.z() = atof(tokens[2].c_str());
			 i = atof(tokens[3].c_str());
			 
			 // Get RGB data if it exists
			 if(tokens.size() > 4){
				 r = atoi(tokens[4].c_str());
				 g = atoi(tokens[5].c_str());
				 b = atoi(tokens[6].c_str());
			 }
			 
			 // If point is NaN skip
			 if (pt.x() == 0 && pt.y() == 0 && pt.z() == 0 && i == 0.5)
				 continue;
			 
			 // Transform point
				 pt = transform * pt;
				 
				 // Set cloud XYZI point data
				 cloud.points[idx].x = pt.x();
				 cloud.points[idx].y = pt.y();
				 cloud.points[idx].z = pt.z();
				 cloud.points[idx].intensity = i;
				 
				 // Set XYZI point data
				 if(tokens.size() > 4){
					 hasColor = true;
					 cloud.points[idx].r = r;
					 cloud.points[idx].g = g;
					 cloud.points[idx].b = b;
				 }
				 idx++;
		 }
		 
		 // Update cloud size
		 cloud.width = idx;
		 cloud.points.resize(idx);
		 
		 // Output scans as NAME_#.pcd, where # is the index of the scan
		 stringstream pcdPathStream;
		 pcdPathStream << pcdPath << "_" << count++ << ".pcd";
		 
		 // Create an XYZ cloud to estimate normals
		 PointCloud<PointXYZ>::Ptr cloud_xyz (new PointCloud<PointXYZ>);
		 copyPointCloud<PointXYZIRGB, PointXYZ>(cloud, *cloud_xyz);
		 
		 // Estimate Normals
		 cout << "Estimating Normals" << endl;
		 NormalEstimation<PointXYZ, Normal> n;
		 PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
		 pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);
		 tree->setInputCloud (cloud_xyz);
		 n.setInputCloud (cloud_xyz);
		 n.setSearchMethod (tree);
		 n.setKSearch (20);
		 n.setViewPoint (center.x(), center.y(), center.z());
		 n.compute (*normals);
		 
		 // Concatenate the XYZ and normal fields
		 PointCloud<PointXYZIRGBNormal>::Ptr cloud_with_normals (new PointCloud<PointXYZIRGBNormal>);
		 concatenateFields<PointXYZIRGB, Normal, PointXYZIRGBNormal> (cloud, *normals, *cloud_with_normals);
		 
		 // If the cloud doesn't have color, strip out the RGB channels and save
		 if(!hasColor){
			 copyPointCloud<PointXYZIRGBNormal, PointXYZINormal>(*cloud_with_normals, cloudi);
			 io::savePCDFileBinary(pcdPathStream.str(), cloudi);
			 // Otherwise save all the channels
		 } else {
			 io::savePCDFileBinary(pcdPathStream.str(), *cloud_with_normals);
		 }
		 
		 cout << "Saved "<< (int)cloud.points.size () << " data points to " << pcdPathStream.str() << endl;
		 
		 // Read the width and height of the next scan
		 ptxFile >> width >> height;
	 }
	 
	 ptxFile.close();
	 return 0;
 }


 
 INITIALIZE_EASYLOGGINGPP

 int main(int argc, char ** argv)
 {
	 START_EASYLOGGINGPP(argc, argv);
	 if(argc-1 < 1 || argc-1 > 2){
		 cerr << "USAGE" << endl;
		 cerr << "This program is used to convert Lidar ptx files into Point Cloud Library pcd files" << endl << endl;
		 cerr << "\t" << argv[0] << " \"/Volumes/Black Caviar/Lidar Scans/Mini Cooper/Mini Cooper\"" << endl;
		 cerr << "\t" << argv[0] << " \"/Volumes/Black Caviar/Lidar Scans/Mini Cooper/Mini Cooper\" \"/tmp/Mini Cooper.pcd\"" << endl;
	 } else {
		 
		 string ptxPath = argv[1];
		 string pcdPath;
		 
		 if(argc-1 == 2){
			 pcdPath = argv[2];
		 } else {
			 pcdPath = ptxPath.substr(0, ptxPath.size()-4);
		 }
		 
		 return ptx2pcd(ptxPath, pcdPath);
	 }
 }
