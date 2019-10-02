#include <spc/elements/Attitude.h>
#include <iostream>

#include <spc/elements/MovableElement.h>
#include <fstream>

#include <spc/io/element_io.h>
#include <cereal/archives/json.hpp>
#include <spc/elements/Attitude.h>
#include <spc/elements/TimeSeriesSparse.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>

#include <spc/elements/SamplesDB.h>

#include <spc/elements/StratigraphicModelSingleAttitude.h>

#include <spc/methods/AlignSingleAttitudeStratigraphicModelToSamples.h>

#include <spc/elements/Fields.h>
int main(int argc, char **argv)
{



    return 1;


    spc::Attitude::Ptr myatt(new spc::Attitude);
    std::cout << myatt->getType()->getClassName() << std::endl;

    spc::ElementBase::Ptr myptr = myatt;

    std::cout << myptr->getType()->getClassName() << std::endl;

        std::cout << myptr->getType()->getParent()->getClassName() << std::endl;

        std::cout << myptr->getType()->getParent()->getParent()->getClassName() << std::endl;



    return 1;


    spc::StratigraphicModelSingleAttitude::Ptr model(
        new spc::StratigraphicModelSingleAttitude);

    model->setNormal(Eigen::Vector3f(0, 0, 100));

    model->setAdditionalShift(2);

    Eigen::Vector3f v1(0, 0, 1);
    float sp1 = model->getScalarFieldValue(v1);
    std::cout << v1.transpose() << ": " << sp1 << std::endl;

    Eigen::Vector3f v2(0, 0, 2);
    float sp2 = model->getScalarFieldValue(v2);
    std::cout << v2.transpose() << ": " << sp2 << std::endl;

    spc::AlignSingleAttitudeStratigraphicModelToSamples aligner;

    spc::SamplesDB::Ptr db(new spc::SamplesDB);

    spc::Sample::Ptr s = db->addSample();
    s->setPosition(Eigen::Vector3f(0, 0, 1));
    s->variantPropertyValue("sp") = 2.1f;

    spc::Sample::Ptr s2 = db->addSample();
    s2->setPosition(Eigen::Vector3f(0, 0, 2));
    s2->variantPropertyValue("sp") = 2.9f;

    spc::Sample::Ptr s3 = db->addSample();
    s3->setPosition(Eigen::Vector3f(0, 0, 3));
    s3->variantPropertyValue("sp") = 4.12f;

    aligner.setStraigraphicModel(model);
    aligner.setFieldName("sp");
    aligner.setSamplesDB(db);

    std::cout << "before " << model->getAdditionalShift() << std::endl;
    aligner.compute();

    std::cout << "after " << model->getAdditionalShift() << std::endl;
    std::cout << "after residual " << aligner.getResiduals().at(0) << " "
                 << aligner.getResiduals().at(1) << " "
              << aligner.getResiduals().at(2) << std::endl;

    return 1;

    spc::Sample::Ptr core(new spc::Sample);
    core->setVariantPropertyValue("test", 10);

    db->pushBack(core);

    //        spc::MovableElement::Ptr obj(new spc::MovableElement);
    std::string a1;
    spc::io::serializeToString(core, a1, spc::io::XML);

    std::cout << a1.c_str() << std::endl;

    return 1;

    std::vector<float> x{ 0, 1, 2, 3 };
    std::vector<float> y{ 0, 1, 2, 3 };

    spc::ElementBase::Ptr element(new spc::TimeSeriesEquallySpaced
                                  (0.0f, 100.0f, 10.0f));

    spc::io::ARCHIVE_TYPE type = spc::io::XML;

    std::string a;
    spc::io::serializeToString(element, a, type);

    std::cout << a.c_str() << std::endl;

    spc::ISerializable::Ptr b = spc::io::deserializeFromString(a, type);

    std::cout << "done" << std::endl;

    std::stringstream newstream;
    spc::io::serializeToStream(b, newstream, spc::io::JSON);

    std::cout << newstream.str().c_str() << std::endl;

    spc::io::serializeToFile(element, "/home/luca/test_serial", spc::io::JSON);

    spc::io::serializeToFile(element, "/home/luca/test_serial", spc::io::XML);

    spc::io::serializeToFile(element, "/home/luca/test_serial", spc::io::SPC);

    spc::ISerializable::Ptr obj_xml
        = spc::io::deserializeFromFile("/home/luca/test_serial.xml");
    spc::ISerializable::Ptr obj_spc
        = spc::io::deserializeFromFile("/home/luca/test_serial.spc");
    spc::ISerializable::Ptr obj_json
        = spc::io::deserializeFromFile("/home/luca/test_serial.json");

    spc::TimeSeriesEquallySpaced::Ptr ts_xml = spcStaticPointerCast
        <spc::TimeSeriesEquallySpaced>(obj_xml);

    spc::TimeSeriesEquallySpaced::Ptr ts_spc = spcStaticPointerCast
        <spc::TimeSeriesEquallySpaced>(obj_spc);

    spc::TimeSeriesEquallySpaced::Ptr ts_json = spcStaticPointerCast
        <spc::TimeSeriesEquallySpaced>(obj_json);

    std::vector<float> x_json = ts_json->getX();
    std::vector<float> x_spc = ts_spc->getX();

    for (int i = 0; i < x_json.size(); ++i) {
        std::cout << x_json.at(i) << " " << x_spc.at(i) << std::endl;
    }

    return 1;
}
