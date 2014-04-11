#include <spc/elements/cylinder.h>

int test_point1( const spc::Cylinder &cyl, const Vector3f &point)
{
    std::cout << "\nPoint:\n" << point << "\ninside test results " << cyl.isPointWithinCylinder(point) << std::endl;
}


int main (int argc, char ** argv)
{

    // create a cylinder
    spc::Cylinder cyl;

    spc::Cylinder::Ptr cyl_ptr (new spc::Cylinder);

    cyl_ptr->setDirection(Vector3f(0,0,1));
    cyl_ptr->setLength(1);
    cyl_ptr->setRadius(1);
    cyl_ptr->setPosition(Vector3f(0,0,0));

    Vector3f tp1 (0,0,0.5);
    Vector3f tp2 (0,0,1.01);
    Vector3f tp3 (1.1,0,0.5);
    Vector3f tp4 (0,0,-0.001);
    Vector3f tp5 (0,0,0.001);
    Vector3f tp6 (0.5,0.5,0.001);


    test_point1(*cyl_ptr, tp1);
    test_point1(*cyl_ptr, tp2);
    test_point1(*cyl_ptr, tp3);
    test_point1(*cyl_ptr, tp4);
    test_point1(*cyl_ptr, tp5);
    test_point1(*cyl_ptr, tp6);


    return 1;
}
