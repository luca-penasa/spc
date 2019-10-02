#pragma once
#ifndef POINTCLOUDSPC_H
#define POINTCLOUDSPC_H
#include <spc/elements/PointCloudBase.h>
//#include <spc/elements/Fields.h>

//#include <spc/elements/EigenTable.h>

namespace spc
{

class  PointCloudSpc : public PointCloudBaseWithSensor
{
    using PointCloudBaseWithSensor::IndexT;
public:
    PointCloudSpc();

//    //! from an eigen table constructor
//    PointCloudSpc(EigenTable::Ptr table): fields_manager_(table)
//    {

//    }

    SPC_ELEMENT(PointCloudSpc)

    // PointCloudBase interface
public:

    virtual void getFieldValue(const IndexT id, const std::string fieldname,
                               float &val) const override
    {
        val = fields_manager_->atScalar(fieldname, id);
    }

    virtual void setFieldValue(const IndexT id, const std::string fieldname,
                               const float &val) override
    {
       fields_manager_->atScalar(fieldname, id) = val;
    }

    virtual IndexT getNumberOfPoints() const override
    {
        return fields_manager_->getNumberOfRows();
    }

    virtual bool hasField(const std::string fieldname) const override
    {
        int id = fields_manager_->getColumnId(fieldname);
        return (id >= 0);
    }

    virtual void resize(const IndexT s) override
    {
        fields_manager_->resize(s);
    }

//    EigenTable::Ptr getFieldsManager() const
//    {
//        return fields_manager_;
//    }

    void setFieldsManager(const spc::EigenTable::Ptr man)
    {
        fields_manager_ = man;
    }



private:
    EigenTable::Ptr fields_manager_;

    // PointCloudBase interface
public:
    virtual std::vector<std::string> getFieldNames() const override
    {
        return fields_manager_->getScalarColumnsNames();
    }

    // PointCloudBase interface
public:
    virtual void addField(const std::string &name) override
    {
        fields_manager_->addNewComponent(name , 1);
    }
};

} // end nspace
#endif // POINTCLOUDSPC_H
