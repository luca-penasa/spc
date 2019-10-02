#pragma once
#ifndef EIGEN_SERIALIZATION_HPP
#define EIGEN_SERIALIZATION_HPP

#include <spc/core/spc_eigen.h>
#include <fstream>

#include <spc/core/cerea_requested_types.hpp>

#include <cereal/cereal.hpp>

//#include <cereal/archives/xml.hpp>

namespace cereal {

template <int Rows, int Cols>
struct has_at_least_one_dynamic_dimension {
    static bool const value = (Rows == Eigen::Dynamic
        || Cols == Eigen::Dynamic);
};

template <int Rows, int Cols>
struct has_fixed_dimensions {
    static bool const value = (Rows != Eigen::Dynamic && Cols != Eigen::Dynamic);
};

///// for QUATERNIONS ////////////////////////
template <class Archive>
inline void
CEREAL_SAVE_FUNCTION_NAME(Archive& ar, const Eigen::Quaternionf& quaternion, const std::uint32_t version)
{
    ar(quaternion.coeffs());
}

template <class Archive>
inline void CEREAL_LOAD_FUNCTION_NAME(Archive& ar, Eigen::Quaternionf& quaternion, const std::uint32_t version)
{
    ar(quaternion.coeffs());
}


///////// for MATRIXES ///////////////////////////

//////// BINARY /////////////
template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline
    typename std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value && !traits::is_text_archive<Archive>::value, void>::type
    CEREAL_SAVE_FUNCTION_NAME(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const& m)
{
    int32_t rows = m.rows();
    int32_t cols = m.cols();
    ar(rows);
    ar(cols);
    ar(binary_data(m.data(), rows * cols * sizeof(_Scalar)));
}



template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline
    typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value && !traits::is_text_archive<Archive>::value, void>::type
    CEREAL_LOAD_FUNCTION_NAME(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m)
{
    int32_t rows;
    int32_t cols;
    ar(rows);
    ar(cols);

    m.resize(rows, cols);

    ar(binary_data(m.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
}

//////////// ASCII - text archives ////////////////
template <class Archive, class T, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline
    typename std::enable_if<traits::is_text_archive<Archive>::value
            && std::is_arithmetic<T>::value,
        void>::type
    CEREAL_SAVE_FUNCTION_NAME(Archive& ar, Eigen::Matrix<T, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const& m)
{
    ar(static_cast<int>(m.rows())); // number of elements
    ar(static_cast<int>(m.cols())); // number of elements

    for (int i = 0; i < m.rows(); i++)
        for (int j = 0; j < m.cols(); j++)
            ar(m(i, j));
}

template <class Archive, class T, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline
    typename std::enable_if<traits::is_text_archive<Archive>::value
            && std::is_arithmetic<T>::value,
        void>::type
    CEREAL_LOAD_FUNCTION_NAME(Archive& ar, Eigen::Matrix<T, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m)
{

//    DLOG(INFO) << "loading matrix";
    int n_rows, n_cols;
    ar(n_rows);
    ar(n_cols);
//    DLOG(INFO) << "size is " << n_rows << " per " << n_cols;

    m.resize(n_rows, n_cols);

    for (int i = 0; i < m.rows(); i++)
        for (int j = 0; j < m.cols(); j++)
            ar(m(i, j));

//    DLOG(INFO) << "done";
}

} // end nspace

#endif // EIGEN_SERIALIZATION_HPP
