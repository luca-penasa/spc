#include <spc/elements/attitude.h>
#include <iostream>


void printVector(Vector3f a, std::string name = std::string("") )
{
    std::cout << name.c_str() << ": " << a(0)  << " " << a(1)  << " "<< a(2)  << std::endl;

}

void printAttitude(spc::Attitude a)
{

    printVector(a.getUnitNormal(), "Normal");

    printVector(a.getDipDirectionVector(), "Dip direction Vector");


    std::cout << "Dip Angle/Dip:" << std::endl;
    std::cout << a.getDipAngle() << "/" << a.getDip() << "\n " <<  std::endl;

}


int main(int argc, char ** argv)

{

    using namespace spc;

    Attitude a (0, 0);
    printAttitude(a);

    Attitude b( 1, 45);
    printAttitude(b);

    Attitude c( 1,135);
    printAttitude(c);

    Attitude d(1, 225);
    printAttitude(d);


    Attitude e(1, 315);
    printAttitude(e);


    Attitude f(-10, 0);
    printAttitude(f);


    std::cout << f << std::endl;



    return 1;
}


std::ostream &spc::operator<<(std::ostream &os, const spc::Attitude &obj)
{
    os << obj.getDipAngle() << " deg  /" << obj.getDip() << " deg N" << std::endl;

    return os;
}
