#include <iostream>
#include <spc/common/common.h>
#include <spc/time_series/equally_spaced_time_series.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>




int main (int argc, char ** argv)
{
	spc::EquallySpacedTimeSeries<float> ts;
	std::cout << "test tool" << std::endl; 
	
	Sleep(2000);


	pcl::console::print_error("error\n ciao" );
	return 1;
}
