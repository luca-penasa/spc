#ifndef EIGEN_SERIALIZATION_HPP
#define EIGEN_SERIALIZATION_HPP


#include <cereal/cereal.hpp>
#include <Eigen/Dense>
#include <fstream>

namespace cereal
{
template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
typename std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
save(Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const & m)
{
    int rows = m.rows();
    int cols = m.cols();
    ar(make_size_tag(static_cast<size_type>(rows * cols)));
    ar(rows);
    ar(cols);
    ar(binary_data(m.data(), rows * cols * sizeof(_Scalar)));
}

template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
load(Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const & m)
{
    size_type size;
    ar(make_size_tag(size));

    int rows;
    int cols;
    ar(rows);
    ar(cols);

    const_cast<Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &>(m).resize(rows, cols);

    ar(binary_data(const_cast<_Scalar *>(m.data()), static_cast<std::size_t>(size * sizeof(_Scalar))));
}


template <class Archive> inline
void
save (Archive & ar, const Eigen::Vector3f &v)
{

    float x,y,z;
    x = v(0);
    y = v(1);
    z = v(2);
    ar( cereal::make_nvp("x",x ),cereal::make_nvp("y",y  ), cereal::make_nvp("z",z  ));

}

template <class Archive> inline
void
load (Archive & ar, Eigen::Vector3f &v)
{
    ar( cereal::make_nvp("x",v(0)) );
    ar( cereal::make_nvp("y",v(1))  );
    ar( cereal::make_nvp("z",v(2))  );
}




}
#endif // EIGEN_SERIALIZATION_HPP
