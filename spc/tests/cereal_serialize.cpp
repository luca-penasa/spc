#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <cereal/types/polymorphic.hpp>
//#include <spc/common/eigen_serialization.hpp>

#include <boost/shared_ptr.hpp>

#include <spc/common/common_includes.h>


namespace cereal
{
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

class ClassBase
{

public:
    typedef SHARED_PTR<ClassBase> Ptr;

    virtual void printSomething() = 0;

};

class ClassA: public ClassBase
{
public:
    typedef SHARED_PTR<ClassA> Ptr;

    ClassA()
    {
        x = 2.0;
        v = Eigen::Vector3f::Random();
    }


    template <class Archive>
    void serialize(Archive & ar)
    {
        ar(cereal::make_nvp("x",x));
        ar(v);
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

class ClassB: public ClassA
{
public:

    typedef boost::shared_ptr<ClassB> Ptr;

    ClassB() { v2 = Eigen::Vector3f::Random();}

    Eigen::Vector3f v2;

    template <class Archive>
    void serialize(Archive & ar)
    {
        ar( cereal::base_class<ClassA>( this ), v2 );

    }


    // ClassBase interface
public:
    virtual void printSomething()
    {
        std::cout << "I am class B, derived from class A, derived from Base" << std::endl;
    }
};


CEREAL_REGISTER_TYPE(ClassA)
CEREAL_REGISTER_TYPE(ClassB)


int main() {



    {

        ClassBase::Ptr  a (new ClassA);


        std::ofstream out ("test.json");
        cereal::JSONOutputArchive archive_o(out);
        archive_o(a);


    }



//    std::ifstream in ("test.json");

//    cereal::JSONInputArchive archive_i(in);

//    ClassB b;
//    archive_i( b);

//    //    std::cout << b.x << "\n" << b.v << std::endl;
//    std::cout << b.x << "\n" << b.v << " v2\n" <<b.v2 << std::endl;



}

//int main()
//{
////    std::stringstream ss; // any stream can be used


//    {

//        std::ofstream out ("test.xml");

//        cereal::XMLOutputArchive oarchive(out); // Create an output archive

//        MyClass m1, m2, m3;
//        oarchive(m1, m2, m3); // Write the data to the archive

////        out.close();


//    }

//    {

//        std::ifstream in("test.xml");
//        cereal::XMLInputArchive iarchive(in); // Create an input archive

//        MyClass a, b, c;
//        iarchive(a);
//        iarchive(b);
//        iarchive(c); // Read the data from the archive
//    }
//}

