#include <spc/elements/Attitude.h>
#include <iostream>

#include <spc/elements/MovableElement.h>
#include <fstream>

#include <spc/io/element_io.h>
#include <cereal/archives/json.hpp>
#include <spc/elements/Attitude.h>
#include <spc/elements/TimeSeriesSparse.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>

int main(int argc, char **argv)
{
    {


        spc::MovableElement::Ptr obj(new spc::MovableElement);
        std::string a1;
        spc::io::serializeToString(obj, a1, spc::io::XML);

        std::cout << a1.c_str() << std::endl;

        return 1;

        std::vector<float> x{ 0, 1, 2, 3 };
        std::vector<float> y{ 0, 1, 2, 3 };

        spc::ElementBase::Ptr element(new spc::TimeSeriesEquallySpaced
                                    <float>(0.0f, 100.0f, 10.0f));

        spc::io::ARCHIVE_TYPE type = spc::io::XML;

        std::string a;
        spc::io::serializeToString(element, a, type);

        std::cout << a.c_str() << std::endl;

        spc::ElementBase::Ptr b = spc::io::deserializeFromString(a, type);

        std::cout << "done" << std::endl;

        std::stringstream newstream;
        spc::io::serializeToStream(b, newstream, spc::io::JSON);

        std::cout << newstream.str().c_str() << std::endl;

        spc::io::serializeToFile(element, "/home/luca/test_serial",
                                 spc::io::JSON);

        spc::io::serializeToFile(element, "/home/luca/test_serial",
                                 spc::io::XML);

        spc::io::serializeToFile(element, "/home/luca/test_serial",
                                 spc::io::SPC);

        spc::ElementBase::Ptr obj_xml
            = spc::io::deserializeFromFile("/home/luca/test_serial.xml");
        spc::ElementBase::Ptr obj_spc
            = spc::io::deserializeFromFile("/home/luca/test_serial.spc");
        spc::ElementBase::Ptr obj_json
            = spc::io::deserializeFromFile("/home/luca/test_serial.json");

        spc::TimeSeriesEquallySpaced<float>::Ptr ts_xml = spcStaticPointerCast
            <spc::TimeSeriesEquallySpaced<float>>(obj_xml);

        spc::TimeSeriesEquallySpaced<float>::Ptr ts_spc = spcStaticPointerCast
            <spc::TimeSeriesEquallySpaced<float>>(obj_spc);

        spc::TimeSeriesEquallySpaced<float>::Ptr ts_json = spcStaticPointerCast
            <spc::TimeSeriesEquallySpaced<float>>(obj_json);

        std::vector<float> x_json = ts_json->getX();
        std::vector<float> x_spc = ts_spc->getX();

        for (int i = 0; i < x_json.size(); ++i) {
            std::cout << x_json.at(i) << " " << x_spc.at(i) << std::endl;
        }
    }

    return 1;
}
