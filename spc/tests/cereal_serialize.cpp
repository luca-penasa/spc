#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <cereal/types/polymorphic.hpp>
//#include <spc/io/eigen_serialization.hpp>

#include <boost/shared_ptr.hpp>

#include <spc/elements/macros.h>


namespace cereal
{
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



class ClassBase
{

public:
    typedef spcSharedPtrMacro<ClassBase> Ptr;

    virtual void printSomething() = 0;
};

class ClassA : public ClassBase
{
public:
    typedef spcSharedPtrMacro<ClassA> Ptr;

    ClassA()
    {
        x = 2.0;
        v = Eigen::Vector3f::Random();
    }

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::make_nvp("x", x));
//        ar(v);
    }

    float x;
    Eigen::Vector3f v;

    // ClassBase interface
public:
    virtual void printSomething()
    {
        std::cout << "I am class A, derived from Base" << std::endl;
    }
};

class ClassB : public ClassA
{
public:
    typedef spcSharedPtrMacro<ClassB> Ptr;

    ClassB()
    {
        v2 = Eigen::Vector3f::Random();
    }

    Eigen::Vector3f v2;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<ClassA>(this), v2);
    }

    // ClassBase interface
public:
    virtual void printSomething()
    {
        std::cout << "I am class B, derived from class A, derived from Base"
                  << std::endl;
    }
};

//CEREAL_REGISTER_TYPE(ClassA)
//CEREAL_REGISTER_TYPE(ClassB)

int main()
{

      Eigen::MatrixXd test = Eigen::MatrixXd::Random(10, 3);

      Eigen::Vector3f v = Eigen::Vector3f::Random();


 {
      std::ofstream out("eigen.cereal");
      cereal::JSONOutputArchive archive_o(out);
      archive_o(test);

}
//    {

////        ClassBase::Ptr a(new ClassA);

//        std::ofstream out("test.json");
//        cereal::BinaryOutputArchive archive_o(test);
//        archive_o(a);
//    }
}
