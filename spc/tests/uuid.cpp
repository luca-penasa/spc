#include <spc/elements/UniversalUniqueObject.h>

#include <iostream>
#include <ostream>

int main(int argc, char ** argv)

{


    spc::UniversalUniqueObject a;

    std::cout << a.getUUIDAsString() << std::endl;






    return 1;
}


