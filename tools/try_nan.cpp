 #include <math.h>
#include <iostream>
// #include "lidarlib.h"
#include <limits>
int main(int argc, char * argv[])
{
	float ecco = std::numeric_limits<float>::signaling_NaN ();
	std::cout << ecco << std::endl;
	std::cout << ecco * 2.0f<< std::endl;
	std::cout << ecco /2.0f<< std::endl;
	std::cout << ecco - 2.0f << std::endl;
	std::cout << ecco + 2.0f << std::endl;
	std::cout << (ecco !=  std::numeric_limits<float>::quiet_NaN ()) << std::endl;
	std::cout << isnan(ecco) << std::endl;
	return 1;
}