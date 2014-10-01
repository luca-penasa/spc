#include "Observations.h"

namespace spc
{

std::vector<spc::Observation> table2observations(const spc::EigenTable &table)
{
    std::vector<Observation> out(table.getNumberOfRows()) ;

    for (int i = 0; i < table.getNumberOfRows(); ++i)
    {
        Observation ob;

        ob.intensity = table.row(i)(table.getColumnId("intensity"));
        ob.intensity_std = table.row(i)(table.getColumnId("intensity_std"));
        ob.distance = table.row(i)(table.getColumnId("distance"));
        ob.angle = table.row(i)(table.getColumnId("angle"));
        ob.eigen_ratio = table.row(i)(table.getColumnId("eigen_ratio"));
        ob.cloud_id = table.row(i)(table.getColumnId("cloud_id"));
        ob.core_id = table.row(i)(table.getColumnId("core_id"));

        out.at(i) = ob;
    }

    return out;

}
}//end nspace
