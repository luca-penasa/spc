#ifndef DATABASE_H
#define DATABASE_H

#include <vector>
#include <typeinfo>
#include <string>
#include <map>
#include <boost/variant.hpp>
#include <Eigen/Dense>
#include <pcl/console/print.h>
#include <spc/elements/ElementBase.h>
#include <cereal/types/map.hpp>

namespace spc
{

class FieldBase : public ElementBase
{
public:
    typedef boost::variant
        <int, float, std::string, Eigen::Vector3f, Eigen::VectorXf,
         Eigen::VectorXi, std::vector<float>, std::vector<int>> VariantT;

    enum SUPPORTED_TYPES {
        INT,
        FLOAT,
        STRING,
        VEC3f,
        VECXf,
        VECXi,
        STDVECf,
        STDVECi
    };

    SPC_OBJECT(FieldBase)
    EXPOSE_TYPE

    virtual size_t size() const = 0;
    virtual bool set(const size_t idx, const VariantT &data) = 0;
    virtual VariantT at(const size_t idx) const = 0;

    virtual void resize(const size_t s) = 0;

    virtual bool isScalarField() const
    {
        return false;
    }

    virtual bool isVector3Field() const
    {
        return false;
    }

    template<typename T> T
    at(const size_t id) const
    {
        return boost::get<T>(at(id));
    }

    template<typename OT, typename RQ> // original type and requested type
    std::vector<RQ> getAsStdVector()
    {
        std::vector<RQ> out(size());
        for (int i = 0; i < size(); ++i)
        {
            out.at(i) = static_cast<RQ>(boost::get<OT>(at(i)));
        }

        return out;

    }

public:
    static std::map<SUPPORTED_TYPES, const std::type_info *> id_to_info_;
    static std::map<const std::type_info *, SUPPORTED_TYPES> info_to_id_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::ElementBase>(this));
    }
};


template <typename T> class FieldTemplate : public FieldBase
{
public:
    typedef T type_;

    virtual size_t size() const
    {
        return data_.size();
    }

    virtual bool set(const size_t idx, const VariantT &data)
    {
        if (data.type() != typeid(T)) {
            pcl::console::print_warn(
                "Wrong data type assignement in ScalarField\n");
            return false;
        } else
            data_.at(idx) = boost::get<T>(data);

        return true;
    }

    virtual VariantT at(const size_t idx) const
    {
        return VariantT(data_.at(idx));
    }

    virtual void resize(const size_t s)
    {
        if (data_.size() != s)
            data_.resize(s);
    }

    std::vector<T> getStdVector() const
    {
        return data_;
    }

//    virtual T at(const size_t s) const
//    {
//        return data_.at(s);
//    }

protected:
    std::vector<T> data_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::FieldBase>(this), CEREAL_NVP(data_));
    }
};

class FieldFloat : public FieldTemplate<float>
{
public:
    SPC_OBJECT(FieldFloat)
    EXPOSE_TYPE

    virtual bool isScalarField() const
    {
        return true;
    }
};

class FieldInt : public FieldTemplate<int>
{
public:
    SPC_OBJECT(FieldInt)
    EXPOSE_TYPE

    virtual bool isScalarField() const
    {
        return true;
    }
};

class FieldString : public FieldTemplate<std::string>
{
public:
    SPC_OBJECT(FieldString)
    EXPOSE_TYPE
};

class FieldVector3f : public FieldTemplate<Eigen::Vector3f>
{
public:
    SPC_OBJECT(FieldVector3f)
    EXPOSE_TYPE

    virtual bool isVector3Field() const
    {
        return true;
    }
};

class FieldVectorXf : public FieldTemplate<Eigen::VectorXf>
{
public:
    SPC_OBJECT(FieldVectorXf)
    EXPOSE_TYPE
};

class FieldVectorXi : public FieldTemplate<Eigen::VectorXi>
{
public:
    SPC_OBJECT(FieldVectorXi)
    EXPOSE_TYPE
};

class FieldVectorStdf : public FieldTemplate<std::vector<float>>
{
public:
    SPC_OBJECT(FieldVectorStdf)
    EXPOSE_TYPE
};

class FieldVectorStdi : public FieldTemplate<std::vector<int>>
{
public:
    SPC_OBJECT(FieldVectorStdi)
    EXPOSE_TYPE
};

class FieldFactory
{
public:
    // the factory
    static FieldBase::Ptr newField(const FieldBase::SUPPORTED_TYPES id)
    {
        if (id == FieldBase::INT)
            return FieldBase::Ptr(new FieldInt);

        else if (id == FieldBase::FLOAT)
            return FieldBase::Ptr(new FieldFloat);

        else if (id == FieldBase::STRING)
            return FieldBase::Ptr(new FieldString);

        else if (id == FieldBase::VEC3f)
            return FieldBase::Ptr(new FieldVector3f);

        else if (id == FieldBase::VECXf)
            return FieldBase::Ptr(new FieldVectorXf);

        else if (id == FieldBase::VECXi)
            return FieldBase::Ptr(new FieldVectorXi);

        else if (id == FieldBase::STDVECf)
            return FieldBase::Ptr(new FieldVectorStdf);

        else if (id == FieldBase::STDVECi)
            return FieldBase::Ptr(new FieldVectorStdi);
        else
            return FieldBase::Ptr();
    }
};

class FieldsManager : public ElementBase
{
public:
    SPC_OBJECT(FieldsManager)
    EXPOSE_TYPE

    typedef std::pair<std::string, spc::FieldBase::Ptr> PairT;

    FieldsManager() : table_size_(0)
    {
    }

    void clear()
    {
        fields_.clear();
        table_size_ = 0;
    }

    void resizeFields(const size_t s)
    {
        for (auto f : fields_) {
            f.second->resize(s);
        }

        table_size_ = s;
    }

    size_t getNumberOfRows()
    {
        return table_size_;
    }

    FieldBase::VariantT getValue(const std::string field_name, size_t idx)
    {
        return getField(field_name)->at(idx);
    }
    //! will automatically crete the field if it does not exists
    void setValue(const std::string field_name, const size_t idx,
                  const FieldBase::VariantT val)
    {
        if (hasField(field_name))
            getField(field_name)->set(idx, val);

        else // we must create a good field
        {
            FieldBase::Ptr new_field
                = newField(field_name, FieldBase::info_to_id_[&val.type()]);
            if (!new_field)
                pcl::console::print_error(
                    "Autocreation of field inpossible. "
                    "The requested field type cannot be created\n");
            new_field->resize(getNumberOfRows());
            new_field->set(idx, val);

            pcl::console::print_warn("Autocreation of field %s done.\n",
                                     field_name.c_str());
        }
    }

    FieldBase::Ptr getField(const std::string field_name)
    {
        if (hasField(field_name))
            return fields_.at(field_name);
    }

    FieldBase::Ptr newField(const std::string name,
                            const FieldBase::SUPPORTED_TYPES type)
    {
        FieldBase::Ptr out = FieldFactory::newField(type);
        out->resize(getNumberOfRows());
        fields_.insert(PairT(name, out));
        return out;
    }

    std::vector<std::string> getFieldNames() const
    {
        std::vector<std::string> out;
        for (auto f : fields_) {
            out.push_back(f.first);
        }
        return out;
    }

    std::vector<std::string> getScalarFieldNames() const
    {
        std::vector<std::string> out;
        for (auto f : fields_) {
            if (f.second->isScalarField())
                out.push_back(f.first);
        }
        return out;
    }

    std::vector<std::string> getVectorFieldNames() const
    {
        std::vector<std::string> out;
        for (auto f : fields_) {
            if (f.second->isVector3Field())
                out.push_back(f.first);
        }
        return out;
    }

    bool hasField(std::string fname)
    {
        if (fields_.find(fname) != fields_.end())
            return true;
        else
            return false;
    }

protected:
    std::map<std::string, spc::FieldBase::Ptr> fields_;

    size_t table_size_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::ElementBase>(this), CEREAL_NVP(fields_),
           CEREAL_NVP(table_size_));
    }

    // ElementBase interface

    // ISerializable interface
public:
    virtual bool isAsciiSerializable() const
    {
        return true;
    }
};

} // end nspace

#endif // DATABASE_H
