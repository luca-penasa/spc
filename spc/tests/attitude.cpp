#include <spc/elements/Attitude.h>
#include <iostream>


#include <spc/elements/MovableElement.h>
#include <fstream>

#include <spc/io/element_io.h>
#include <cereal/archives/json.hpp>
#include <spc/elements/Attitude.h>
#include <spc/elements/TimeSeriesSparse.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>

int main(int argc, char ** argv)
{
    {

        std::vector<float> x {0, 1, 2, 3};
        std::vector<float> y {0, 1, 2, 3};

        spc::spcObject::Ptr  element (new  spc::EquallySpacedTimeSeries<float>(0.0f, 100.0f, 10.0f) );


        spc::io::ARCHIVE_TYPE type = spc::io::XML;

        std::string a;
        spc::io::serializeToString(element, a, type);


        std::cout << a.c_str() << std::endl;

        spc::spcObject::Ptr b = spc::io::deserializeFromString(a, type);

        std::cout << "done" << std::endl;

        std::stringstream newstream ;
        spc::io::serializeToStream(b, newstream, spc::io::JSON);

        std::cout << newstream.str().c_str() << std::endl;




        spc::io::serializeToFile(element, "/home/luca/test_serial", spc::io::JSON);

        spc::io::serializeToFile(element, "/home/luca/test_serial", spc::io::XML);

        spc::io::serializeToFile(element, "/home/luca/test_serial", spc::io::SPC);


        spc::spcObject::Ptr obj_xml = spc::io::deserializeFromFile("/home/luca/test_serial.xml");
        spc::spcObject::Ptr obj_spc = spc::io::deserializeFromFile("/home/luca/test_serial.spc");
        spc::spcObject::Ptr obj_json = spc::io::deserializeFromFile("/home/luca/test_serial.json");


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


