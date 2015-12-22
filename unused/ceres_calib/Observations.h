#ifndef OBSERVATIONS_H
#define OBSERVATIONS_H


#include <spc/elements/EigenTable.h>


namespace spc
{


class Observation
{
public:
    double intensity;
    double intensity_std;
    double distance;
    double angle;
    double eigen_ratio;
    size_t cloud_id;
    size_t core_id;

};

std::vector<Observation> table2observations(const spc::EigenTable & table);

}//end nspace
#endif // OBSERVATIONS_H
