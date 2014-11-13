#ifndef EIGENTABLE_H
#define EIGENTABLE_H

#include <spc/core/spc_eigen.h>
#include <map>

#include <pcl/console/print.h>
#include <spc/elements/ElementBase.h>
#include <spc/io/eigen_serialization.hpp>

#include <cereal/types/vector.hpp>

#include <cereal/cereal.hpp>

#include <cereal/archives/binary.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace spc
{

class EigenTable : public ElementBase
{
public:
    SPC_ELEMENT(EigenTable)

    EXPOSE_TYPE

    EigenTable()
    {
    }



    EigenTable(const EigenTable &other, bool only_structure = false) ;




    Eigen::MatrixXf &mat()
    {
        return mat_;
    }

    Eigen::MatrixXf mat() const
    {
        return mat_;
    }

    EigenTable::Ptr getWithStrippedNANs(const std::vector
                                        <std::string> &columns_to_check) const;

    void addNewComponent(const std::string &name, size_t dimensionality = 1);

    std::string getNameOfComponentAtDimension(const std::string &basename,
                                              const size_t &dimension) const
    {
        return basename + "@" + boost::lexical_cast<std::string>(dimension);
    }

    size_t getNumberOfRows() const
    {
        return mat_.rows();
    }

    size_t getNumberOfColumns() const
    {
        return mat_.cols();
    }

    float &atScalar(const std::string &name, const size_t &row,
                    const size_t &dimension = 0)
    {
        size_t id = getColumnId(name);
        return mat_(row, id + dimension);
    }

    const float atScalar(const std::string &name, const size_t &row,
                         const size_t &dimension = 0) const;

    Eigen::Block<Eigen::Matrix<float, -1, -1>, 1, -1>
    atVector(const std::string &name, const size_t &row);

    size_t getColumnId(const std::string &name) const;

    size_t getColumnDimensionality(const std::string &name) const;

    void resize(const size_t &rows);

    Eigen::Block<Eigen::Matrix<float, -1, -1>, -1, 1, true> column(const size_t
                                                                   &id)
    {
        return mat_.col(id);
    }

    Eigen::Block<Eigen::Matrix<float, -1, -1>, -1, 1, true>
    column(const std::string &col_name);

    //! get back the  complete vector field.
    Eigen::Block<Eigen::Matrix<float, -1, -1>, -1, -1>
    getVectorField(const std::string &name);

    Eigen::Block<Eigen::Matrix<float, -1, -1>, 1, -1> row(const size_t &id)
    {
        return mat_.row(id);
    }

    Eigen::Block<const Eigen::Matrix<float, -1, -1>, 1, -1> row(const size_t
                                                                &id) const
    {
        return mat_.row(id);
    }

    virtual bool isAsciiSerializable() const
    {
        return true;
    }

    std::vector<std::string> getScalarColumnsNames() const;

    std::string getColumnName(const size_t &id) const;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<ElementBase>(this), CEREAL_NVP(mat_),
           CEREAL_NVP(names_to_col_), CEREAL_NVP(col_to_dim_),
           CEREAL_NVP(cols_to_name_));
    }

protected:
    Eigen::MatrixXf mat_;

    std::map<std::string, size_t> names_to_col_;
    std::map<size_t, std::string> cols_to_name_;
    std::map<std::string, size_t> col_to_dim_;
};

} // end nspace
#endif // EIGENTABLE_H
