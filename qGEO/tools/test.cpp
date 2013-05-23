#include <iostream>
 
using namespace std;
 
 
template <int X>
struct Foo;
 
template<>
struct Foo<0> {
    int i;
};
 
template <int X>
struct Foo : public Foo<X-1> {
    Foo(){i = X;}
    int i;
};
 
 
int main()
{
    Foo<10> f;
    Foo<23> c;
    f.i = 12;
    std::cout << f.i << std::endl;
    std::cout << c.i << std::endl;
 
    return 0;
}
