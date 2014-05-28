#include <spc/elements/attitude.h>
#include <iostream>


#include <spc/elements/movable_element.h>
#include <fstream>

#include <spc/io/element_io.h>
#include <spc/elements/UniversalUniqueObject.h>

#include <cereal/archives/json.hpp>

#include <spc/elements/attitude.h>
#include <spc/time_series/sparse_time_series.h>
#include <spc/time_series/equally_spaced_time_series.h>

int main(int argc, char ** argv)
{
    {

        std::vector<float> x {0, 1, 2, 3};
        std::vector<float> y {0, 1, 2, 3};

        spc::spcObject::Ptr  element (new  spc::EquallySpacedTimeSeries<float>(0.0f, 100.0f, 10.0f) );


        spc::ElementsIO::ARCHIVE_TYPE type = spc::ElementsIO::XML;

        std::string a;
        spc::ElementsIO::serializeToString(element, a, type);


        std::cout << a.c_str() << std::endl;

        spc::spcSerializableObject::Ptr b = spc::ElementsIO::deserializeFromString(a, type);

        std::cout << "done" << std::endl;

        std::stringstream newstream ;
        spc::ElementsIO::serializeToStream(b, newstream, spc::ElementsIO::JSON);

        std::cout << newstream.str().c_str() << std::endl;




        spc::ElementsIO::serializeToFile(element, "/home/luca/test_serial", spc::ElementsIO::JSON);

        spc::ElementsIO::serializeToFile(element, "/home/luca/test_serial", spc::ElementsIO::XML);

        spc::ElementsIO::serializeToFile(element, "/home/luca/test_serial", spc::ElementsIO::SPC);


        spc::spcSerializableObject::Ptr obj_xml = spc::ElementsIO::deserializeFromFile("/home/luca/test_serial.xml");
        spc::spcSerializableObject::Ptr obj_spc = spc::ElementsIO::deserializeFromFile("/home/luca/test_serial.spc");
        spc::spcSerializableObject::Ptr obj_json = spc::ElementsIO::deserializeFromFile("/home/luca/test_serial.json");


        spc::EquallySpacedTimeSeries<float>::Ptr ts_xml =  spcStaticPointerCast<spc::EquallySpacedTimeSeries<float>> (obj_xml );

        spc::EquallySpacedTimeSeries<float>::Ptr ts_spc =  spcStaticPointerCast<spc::EquallySpacedTimeSeries<float>> (obj_spc );

        spc::EquallySpacedTimeSeries<float>::Ptr ts_json =  spcStaticPointerCast<spc::EquallySpacedTimeSeries<float>> (obj_json );

        std::vector<float> x_json = ts_json->getX();
        std::vector<float> x_spc = ts_spc->getX();

        for (int i = 0; i < x_json.size(); ++i)
        {
            std::cout << x_json.at(i) << " " << x_spc.at(i) << std::endl;
        }


    }





    return 1;
}


