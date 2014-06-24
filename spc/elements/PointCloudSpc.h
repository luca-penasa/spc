#pragma once
#ifndef POINTCLOUDSPC_H
#define POINTCLOUDSPC_H
#include <spc/elements/PointCloudBase.h>
//#include <spc/elements/Fields.h>

#include <spc/elements/EigenTable.h>

namespace spc
{

class PointCloudSpc : public PointCloudBase
{
public:
    PointCloudSpc();

    SPC_OBJECT(PointCloudSpc)

    // PointCloudBase interface
public:
    virtual void getPoint(const int id, float &x, float &y, float &z) const
    {
            x = fields_manager_->atScalar (x_field_name_, id, 0);
            y = fields_manager_->atScalar (y_field_name_, id, 1);
            z = fields_manager_->atScalar (z_field_name_, id, 2);
    }
    virtual void setPoint(const int id, const float x, const float y,
                          const float z)
    {
        fields_manager_->atScalar(x_field_name_, id, 0) = x;
        fields_manager_->atScalar(y_field_name_, id, 1) = y;
        fields_manager_->atScalar(z_field_name_, id, 2) = z;
    }
    virtual void getFieldValue(const int id, const std::string fieldname,
                               float &val)
    {
        val = fields_manager_->atScalar(fieldname, id);
    }
    virtual int size() const
    {
        return fields_manager_->getNumberOfRows();
    }

    virtual bool hasField(const std::string fieldname)
    {
        int id = fields_manager_->getColumnId(fieldname);
        return (id >= 0);
    }

    virtual void resize(size_t s)
    {
        fields_manager_->resize(s);
    }

    EigenTable::Ptr getFieldsManager() const
    {
        return fields_manager_;
    }

    spcSetMacro(XFieldName, x_field_name_, std::string)
    spcGetMacro(XFieldName, x_field_name_, std::string)

    spcSetMacro(YFieldName, y_field_name_, std::string)
    spcGetMacro(YFieldName, y_field_name_, std::string)

    spcSetMacro(ZFieldName, z_field_name_, std::string)
    spcGetMacro(ZFieldName, z_field_name_, std::string)

    void setFieldsManager(const spc::EigenTable::Ptr man)
    {
        fields_manager_ = man;
    }

private:
    EigenTable::Ptr fields_manager_;

    std::string x_field_name_;
    std::string y_field_name_;
    std::string z_field_name_;

    // PointCloudBase interface
public:
    virtual std::vector<std::string> getFieldNames()
    {
        return fields_manager_->getScalarColumnsNames();
    }
};

} // end nspace
#endif // POINTCLOUDSPC_H
