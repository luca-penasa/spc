#include <iostream>
class A
{
public:
    virtual int getValue(int i)
    {
        std::cout << "called class A getvalue" << std::endl;
    }

};


template<class F>
class D: public A
{
public:
    D(F * myclass): myclass_(myclass)
    {

    }

    template <typename T> T value(T par)
    {
        return myclass_->getValue(par);
    }

    F * myclass_;

};


class B: public A
{
    virtual int getValue(int i) const
    {
        std::cout << "called class B method" << std::endl;
        return 1;
    }

};

class C: public A
{
    // A interface
public:
    template<typename T>
    T getValue(T i) const
    {
        std::cout << "called class C method" << std::endl;
        return 2;
    }

//    virtual int getValue(int i) const
//    {
//        std::cout << "called duplicate class C method" << std::endl;
//        return 1;
//    }

};

//template<int> C::getValue<int>(int i);

template int C::getValue(int) const;


int main (int argc, char ** argv)
{

    D<B>* d_ptr = new D<B>(new B);
    A * b_ptr =  d_ptr;
    A * c_ptr = new C;

    double dn = 2.2;
    int in = 2;
    b_ptr->getValue(dn);
    c_ptr->getValue(dn);

    return 1;
}
