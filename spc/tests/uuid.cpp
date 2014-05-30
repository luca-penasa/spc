#include <spc/elements/UniversalUniqueID.h>
#include <iostream>
#include <ostream>

int main(int argc, char **argv)

{

    spc::UniversalUniqueID a;

    std::cout << a.getUUIDAsString() << std::endl;

    return 1;
}
