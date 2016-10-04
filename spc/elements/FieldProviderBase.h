#pragma once
#ifndef FIELDPROVIDERBASE_H
#define FIELDPROVIDERBASE_H

#include <spc/core/ElementBase.h>

namespace spc

{

template<typename ScalarT>
class FieldProviderBase
{
public:

    spcTypedefSharedPtrs(FieldProviderBase)

//    typedef float ScalarT;
    typedef Eigen::Map<Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>> MapT;

    FieldProviderBase()
    {

    }

    virtual ~FieldProviderBase()
    {

    }

    MapT asEigenMap()
    {
        return MapT(data(), static_cast<int>(getNumberOfElements()),
                    static_cast<int>(getFieldDimension()));
    }

    virtual void resize(const size_t & n_elements) = 0;

    virtual ScalarT * data() = 0;

    virtual size_t getFieldDimension()  const = 0;

    virtual size_t getNumberOfElements() const = 0;

    virtual std::string getFieldName() const = 0;

    virtual void setFieldName(const std::string & name) = 0;

};

template<typename ScalarT>
class FieldProviderEigen: public FieldProviderBase<ScalarT>
{
    // FieldProviderBase interface
public:

    FieldProviderEigen(const std::string & name = "field",
                       const size_t & dim = 1,
                       const size_t & n_elements = 0): field_name_(name)
    {
        mat_.resize(n_elements, dim);
    }

    // FieldProviderBase interface
public:
    virtual void resize(const size_t &n_elements) override
    {
        mat_.conservativeResize(n_elements, Eigen::NoChange);
    }
    virtual ScalarT *data()  override
    {
        return mat_.data();
    }
    virtual size_t getFieldDimension() const override
    {
        return mat_.cols();
    }
    virtual size_t getNumberOfElements() const override
    {
        return mat_.rows();
    }
    virtual std::string getFieldName() const override
    {
        return field_name_;
    }

protected:
    Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> mat_;
    std::string field_name_;

    // FieldProviderBase interface
public:
    virtual void setFieldName(const std::string &name) override
    {
        field_name_ = name;
    }
};


}//end nspace
#endif // FIELDPROVIDERBASE_H
