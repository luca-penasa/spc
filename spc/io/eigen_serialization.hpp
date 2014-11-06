#ifndef EIGEN_SERIALIZATION_HPP
#define EIGEN_SERIALIZATION_HPP

#include <spc/core/spc_eigen.h>
#include <fstream>

#include <spc/core/cerea_requested_types.hpp>

#include <cereal/cereal.hpp>

//#include <cereal/archives/xml.hpp>


namespace cereal
{
// we should specialize this templates for binary only dump:
// like they do here: http://stackoverflow.com/questions/22884216/serializing-eigenmatrix-using-cereal-library
// see first answer.

//! the generic eigen type. Any eigen type will be saved in this way
//! if you dont provide an overloading as below for xml and json
template <int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
void
save(BinaryOutputArchive & ar, Eigen::Matrix<float, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const & m, const std::uint32_t version)
{
    int32_t rows = m.rows();
    int32_t cols = m.cols();
    ar(rows);
    ar(cols);

    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            ar(m(i,j));
}

template < int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
void load(BinaryInputArchive & ar, Eigen::Matrix<float, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m, const std::uint32_t version)
{
    int32_t rows;
    int32_t cols;
    ar(rows);
    ar(cols);

    m.resize(rows, cols);

    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            ar(m(i,j));

}

//! this macro permits to specialize load and save coupelts for a given format and shape of matrices
//! it is needed to force cereal to output some matrices always as string and not binary
#define SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(ARTYPE, ROWS, COLS)\
template <class _Scalar> inline \
void \
save(ARTYPE##OutputArchive & ar, Eigen::Matrix<_Scalar, ROWS, COLS> const & m, const std::uint32_t version)\
{\
    for (int i = 0; i <  ROWS; i++)\
        for (int j= 0; j < COLS; j++)\
            ar(m(i,j));\
}\
template <class _Scalar> inline \
void \
load(ARTYPE## InputArchive & ar, Eigen::Matrix<_Scalar, ROWS, COLS>  & m, const std::uint32_t version)\
{\
    for (int i = 0; i <  ROWS; i++)\
        for (int j= 0; j < COLS; j++)\
        { \
            _Scalar v; \
            std::cout << typeid(_Scalar).name() << std::endl;\
            std::cout << sizeof(_Scalar) << std::endl;\
            ar(v); \
            m(i,j) = v;\
        } \
}

SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(XML, 2, 1)
SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(JSON, 2, 1)
SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(XML, 2, 2)
SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(JSON, 2, 2)
SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(XML, 3, 1)
SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(JSON, 3, 1)
SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(XML, 4, 1)
SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(JSON, 4, 1)
SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(XML, 3, 3)
SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(JSON, 3, 3)
SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(XML, 4, 4)
SPECIALIZE_CEREAL_EIGEN_MATRIX_TO_STRING(JSON, 4, 4)







//! all the remaining eigen types will be saved a as binary dumps in xml
template <class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
void
save(cereal::XMLOutputArchive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const & m, const std::uint32_t version)
{
    int32_t rows = m.rows();
    int32_t cols = m.cols();
    ar(rows);
    ar(cols);

    ar.saveBinaryValue( m.data(), sizeof(_Scalar) * m.size(), "EigenMatrixBase64" );

    //      ar(binary_data(m.data(), rows * cols * sizeof(_Scalar)));
}

template < class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
void
load(cereal::XMLInputArchive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m, const std::uint32_t version)
{
    int32_t rows;
    int32_t cols;
    ar(rows);
    ar(cols);

    m.resize(rows, cols);

    ar.loadBinaryValue(m.data(), sizeof(_Scalar) * m.size(), "EigenMatrixBase64" );

    //      ar(binary_data(m.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
}


////! all the remaining eigen types will be saved a as binary dumps in xml
template <class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
void
save(cereal::JSONOutputArchive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const & m, const std::uint32_t version)
{
    int32_t rows = m.rows();
    int32_t cols = m.cols();
    ar(rows);
    ar(cols);

    ar.saveBinaryValue( m.data(), sizeof(_Scalar) * m.size(), "EigenMatrixBase64" );

    //      ar(binary_data(m.data(), rows * cols * sizeof(_Scalar)));
}

template < class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
void
load(cereal::JSONInputArchive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m, const std::uint32_t version)
{
    int32_t rows;
    int32_t cols;
    ar(rows);
    ar(cols);

    m.resize(rows, cols);

    ar.loadBinaryValue(m.data(), sizeof(_Scalar) * m.size(), "EigenMatrixBase64" );

    //      ar(binary_data(m.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
}





}// end nspace

#endif // EIGEN_SERIALIZATION_HPP
