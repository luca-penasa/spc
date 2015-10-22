#include "EigenTable.h"

#include <spc/core/logging.h>

namespace spc
{

DtiClassType EigenTable::Type = DtiClassType("EigenTable", &ElementBase::Type);

EigenTable::EigenTable(const EigenTable &other, bool only_structure): ElementBase(other)
{
    names_to_col_ = other.names_to_col_;
    col_to_dim_ = other.col_to_dim_;
    cols_to_name_ = other.cols_to_name_;

    if (only_structure)
        mat_.resizeLike(other.mat());
    else
        mat_ = other.mat_;
}

EigenTable::Ptr
EigenTable::getWithStrippedNANs(const std::vector
                                <std::string> &columns_to_check) const
{
    EigenTable::Ptr newtable(new EigenTable(*this, true));

    std::vector<int> cols_to_check_ids;

    for (std::string s : columns_to_check)
        cols_to_check_ids.push_back(this->getColumnId(s));

    int counter = 0;
    for (int i = 0; i < this->getNumberOfRows(); ++i) {
        Eigen::VectorXf myrow = this->row(i);
        bool good = true;
        for (int id : cols_to_check_ids) {
            if (!std::isfinite(myrow(id))) {
                good = false;
                break;
            }
        }

        if (good)
            newtable->row(counter++) = myrow;
    }

    newtable->resize(counter);

    LOG(INFO) << "Stripped  " <<
                             this->getNumberOfRows()
                             - newtable->getNumberOfRows() <<
                 "  nans from table";

    return newtable;
}

void EigenTable::addNewComponent(const std::string &name, size_t dimensionality)
{
    if (names_to_col_.find(name) != names_to_col_.end())
        LOG(ERROR) << "A column with this name " << name << " yet exists. "
                                  <<"Cannot create a new one. ";

    else {

        size_t next_id
            = getNumberOfColumns(); // correspond to the next "free" column

        DLOG(INFO) << "Adding new componet "<<name;
        DLOG(INFO) << "dimensionality asked " << dimensionality;
        DLOG(INFO) << "next free id: " << next_id;
        names_to_col_[name] = next_id;
        col_to_dim_[name] = dimensionality;
        cols_to_name_[next_id] = name;


        if (dimensionality != 1) {
            for (int i = 0; i < dimensionality; ++i)
            {
                std::string newname = getNameOfComponentAtDimension(name, i);
                DLOG(INFO) << "added subcomponent: " << newname;
                names_to_col_[newname] = next_id + i;
                col_to_dim_[newname] = 1;
                cols_to_name_[next_id + i] = newname;
            }
        }

        mat_.conservativeResize(getNumberOfRows(),
                                getNumberOfColumns() + dimensionality);

        mat_.block(0, next_id, getNumberOfRows(), dimensionality)
            = Eigen::MatrixXf::Zero(getNumberOfRows(), dimensionality);
    }
}

const float EigenTable::atScalar(const std::string &name, const size_t &row,
                                 const size_t &dimension) const
{
    size_t id = getColumnId(name);
    return mat_(row, id + dimension);
}

Eigen::Block<Eigen::Matrix<float, -1, -1>, 1, -1>
EigenTable::atVector(const std::string &name, const size_t &row)
{
    size_t id = getColumnId(name);
    size_t dim = getColumnDimensionality(name);

    return Eigen::Block
        <Eigen::Matrix<float, -1, -1>, 1, -1>(mat_, row, id, 1, dim);
}

size_t EigenTable::getColumnId(const std::string &name) const
{
    std::vector<std::string> splitted;
    boost::split(splitted, name, boost::is_any_of("@"));

    if (splitted.size() > 1) // we found a good split
    {
        std::string basename = splitted.at(0);
        size_t dim_id = boost::lexical_cast
            <size_t>(splitted.at(splitted.size() - 1));

        if (names_to_col_.find(basename) != names_to_col_.end()) {
            size_t col_id = names_to_col_.at(basename) + dim_id;
            return col_id;
        } else
            return -1; // not found

    } else if (splitted.size() == 1) {
        if (names_to_col_.find(name) != names_to_col_.end())
            return names_to_col_.at(name);
        else
            return -1;
    }

    return -1;
}

size_t EigenTable::getColumnDimensionality(const std::string &name) const
{
    return col_to_dim_.at(name);
}

void EigenTable::resize(const size_t &rows)
{
    size_t old_rows = getNumberOfRows();

    DLOG(INFO) << "resized from " << old_rows << " to " << rows;
    mat_.conservativeResize(rows, Eigen::NoChange);

    if (rows > old_rows) // init to zero the matrix
    {
        mat_.block(old_rows, 0, rows - old_rows, getNumberOfColumns()).fill(0);
    }
}

Eigen::Block<Eigen::Matrix<float, -1, -1>, -1, 1, true>
EigenTable::column(const std::string &col_name)
{
    size_t id = getColumnId(col_name);
    if (id == -1)
        LOG(ERROR) <<
            "Cannot find such a column" << col_name <<" Is it the name set and right?";
    else
        return column(id);

}

Eigen::Block<Eigen::Matrix<float, -1, -1>, -1, -1>
EigenTable::getVectorField(const std::string &name)
{
    size_t id = getColumnId(name);
    size_t dim = getColumnDimensionality(name);
    return Eigen::Block<Eigen::Matrix<float, -1, -1>, -1, -1>(
        mat_, 0, id, this->getNumberOfRows(), dim);
}

std::string EigenTable::getColumnName(const size_t &id) const
{
    if (cols_to_name_.find(id) != cols_to_name_.end())
        return cols_to_name_.at(id);
}

std::vector<std::string> EigenTable::getScalarColumnsNames() const
{
    std::vector<std::string> names;

    size_t n_cols = getNumberOfColumns();

    for (size_t i = 0; i < n_cols; ++i) {
        names.push_back(getColumnName(i));
    }

    return names;
}
}


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::EigenTable)
