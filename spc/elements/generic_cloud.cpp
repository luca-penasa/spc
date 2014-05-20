#include "generic_cloud.h"

namespace spc
{

spcGenericCloud::spcGenericCloud()
{
}

std::vector<float> spc::spcGenericCloud::getField(const std::string fieldname, std::vector<int> indices)
{
    std::vector<float> out;

    if (!hasField(fieldname))
    {
        pcl::console::print_warn("[Error in %s] asked for field %s", getClassName().c_str(), fieldname.c_str());
        return out;
    }

    float val;
    spcForEachMacro(int i, indices)
    {
        getFieldValue(i, fieldname, val);
        out.push_back(val);
    }

    return out;

}





}//end nspace
