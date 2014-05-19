#include <spc/elements/attitude.h>
#include <iostream>


#include <spc/elements/movable_element.h>
#include <fstream>

#include <spc/io/element_io.h>



int main(int argc, char ** argv)

{
    spc::PositionableElement  element (0,1,2);

    spc::ElementsIO saver;
    saver.serialize(element, "/home/luca/test.xml");








    return 1;
}


