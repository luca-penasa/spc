#ifndef EIGEN_SERIALIZATION_HPP
#define EIGEN_SERIALIZATION_HPP

#include <spc/core/spc_eigen.h>
#include <fstream>


#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>

namespace cereal
{
// we should specialize this templates for binary only dump:
// like they do here: http://stackoverflow.com/questions/22884216/serializing-eigenmatrix-using-cereal-library
// see first answer.
  template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
void
    save(Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const & m)
    {
      int32_t rows = m.rows();
      int32_t cols = m.cols();
      ar(rows);
      ar(cols);

      for (int i = 0; i < rows; i++)
          for (int j = 0; j < cols; j++)
              ar(m(i,j));
    }

  template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
void    load(Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m)
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
}



#endif // EIGEN_SERIALIZATION_HPP
