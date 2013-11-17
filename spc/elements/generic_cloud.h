#ifndef GENERIC_CLOUD_H
#define GENERIC_CLOUD_H

#include "element_base.h"

namespace spc
{

class GenericCloud: public ElementBase
{
public:
    GenericCloud();

    /// a generic cloud must implement this method
    virtual void getPoint(const int id, float &x, float &y, float &z) = 0 ;

    virtual void getFieldValue(const int id, const std::string fieldname, float &val) = 0;

    virtual std::vector<float> getField(const std::string fieldname, std::vector<int> indices)
    {
        std::vector<float> out;

        float val;
        for (int i : indices)
        {
            getFieldValue(i, fieldname, val);
            out.push_back(val);
        }

        return out;


    }

    virtual int getSize() const = 0 ;

};



}//end nspace

#endif // GENERIC_CLOUD_H
