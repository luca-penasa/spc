#ifndef INTENSITYMODELBASE_H
#define INTENSITYMODELBASE_H


#include <Eigen/Dense>
#include <map>
#include <spc/elements/macros_ptr.h>
#include <iostream>

namespace spc
{


class FieldToFieldNameMapper
{
public:

    enum FIELD_ENUM {DISTANCE, ANGLE, INTENSITY};


    FieldToFieldNameMapper()
    {
        field_to_name_[DISTANCE] = "distance";
        field_to_name_[ANGLE] = "angle";
        field_to_name_[INTENSITY] = "intensity";
    }


    void setFieldName(const FIELD_ENUM field, const std::string & fieldname)
    {
        field_to_name_[field] = fieldname;
    }

    const bool operator ()(const FIELD_ENUM field, std::string &f_name) const
    {
        if (field_to_name_.find(field) != field_to_name_.end())
        {
            f_name  = field_to_name_.at(field);
            return true; //sucessful lookup
        }
        else
        {
            f_name.clear(); // ensure we are returning it void
            std::cout << "it looks like you did not provide a name mapping for this field!" << std::endl;
            return false;
        }
    }

    const bool operator ()(const std::vector<FIELD_ENUM> fields, std::vector<std::string> &f_names) const
    {
        f_names.clear();
        for (auto field: fields)
        {
            std::string field_name;
            bool ok = operator ()(field, field_name);
            if (!ok)
                return false;

            f_names.push_back(field_name);
        }

        return true;
    }


protected:
    std::map<FIELD_ENUM, std::string> field_to_name_;

};


class IntensityModelBase
{
public:

    spcTypedefSharedPtrs(IntensityModelBase)

    //! this is the key method which provides the modeling of intensity on the basis of the
    //! fields given by getRequiredFields()
    virtual float getPredictedIntensity( const Eigen::VectorXf &point_in_par_space) const = 0;

    //! each model derived from this class may require a given set of fields to work
    //! e.g. only [distance] or [distance, scatterin_angle] or others....
    virtual std::vector<FieldToFieldNameMapper::FIELD_ENUM> getRequiredFields() const = 0;

};
} // end nspace

#endif // INTENSITYMODELBASE_H
