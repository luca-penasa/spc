#pragma once
#ifndef POINTCLOUDSPC_H
#define POINTCLOUDSPC_H
#include <spc/elements/PointCloudBase.h>
//#include <spc/elements/Fields.h>

#include <spc/elements/EigenTable.h>

namespace spc
{

class  PointCloudSpc : public PointCloudBase
{
public:
    PointCloudSpc();

    SPC_OBJECT(PointCloudSpc)

    // PointCloudBase interface
public:

    virtual void getFieldValue(const int id, const std::string fieldname,
                               float &val) const
    {
        val = fields_manager_->atScalar(fieldname, id);
    }

    virtual void setFieldValue(const int id, const std::string fieldname,
                               const float &val)
    {
       fields_manager_->atScalar(fieldname, id) = val;
    }

    virtual int size() const
    {
        return fields_manager_->getNumberOfRows();
    }

    virtual bool hasField(const std::string fieldname) const
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

    void setFieldsManager(const spc::EigenTable::Ptr man)
    {
        fields_manager_ = man;
    }

private:
    EigenTable::Ptr fields_manager_;

    // PointCloudBase interface
public:
    virtual std::vector<std::string> getFieldNames()
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