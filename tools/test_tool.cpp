#include <iostream>
#include <spc/methods/common.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>




int main (int argc, char ** argv)
{
    spc::TimeSeriesEquallySpaced<float> ts;
	std::cout << "test tool" << std::endl; 
	
    //Sleep(2000); // this works only on win


	pcl::console::print_error("error\n ciao" );
	return 1;
}
