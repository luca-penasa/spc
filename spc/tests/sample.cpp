//#include "spc/elements/UniversalUniqueObject.h"

#include <spc/elements/VariantDataRecord.h>

#include <iostream>
//#include <ostream>

int main(int argc, char **argv)

{

    spc::VariantDataRecord record;
    record.property("ciao") = 2.2;
    record.property("ciao") = 1;
    record.property("ecco") = "well done";
    record.property("vector") = Eigen::Vector3f(0.0, 0.1, 290.29);

    std::cout << record << std::endl;

    return 1;
}
