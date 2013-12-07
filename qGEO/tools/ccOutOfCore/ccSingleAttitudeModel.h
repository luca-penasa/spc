#ifndef CC_SINGLE_ATTITUDE_MODEL_H
#define CC_SINGLE_ATTITUDE_MODEL_H

#include <ccHObject.h>

#include <spc/stratigraphy/single_attitude_model.h>
#include <ccCylinder.h>
#include <ccOutOfCore/ccEditableHObject.h>

#include <QDialog>

#include <ccOutOfCore/ccMyBaseObject.h>
#include <QIcon>
#include <ccPointCloud.h>

#include <spc/elements/salvable_object.h> //needed to serialize Vector3f
#include <boost/serialization/vector.hpp>

class ccSingleAttitudeModel: public QObject,  public ccMyBaseObject, public spc::spcSingleAttitudeModel
{
    Q_OBJECT

public:


    ccSingleAttitudeModel();

    // copy const
    ccSingleAttitudeModel(const ccSingleAttitudeModel &model);


    ccSingleAttitudeModel(const spc::spcAttitude & att);

    spc::spcSingleAttitudeModel::Ptr asSPCClass()
    {
        spc::spcSingleAttitudeModel * spcPtr =  static_cast<spc::spcSingleAttitudeModel *> (this);
        return boost::make_shared<spc::spcSingleAttitudeModel>(*spcPtr);
    }



    virtual bool isSerializable() const { return true; }

    virtual bool hasColors() const { return true; }

    virtual ccBBox getMyOwnBB() {return ccBBox();}

    void updateInternals()
    {
        updateMajorBreaks();
    }
    virtual QIcon * getIcon() const
    {
        return new QIcon(QString::fromUtf8(":/toolbar/icons/AttitudeToModel.png"));
    }




public slots:

    float getMinSp() const;
    void setMinSp(float min_sp);

    float getMaxSp() const;
    void setMaxSp(float max_sp);

    float getStep() const;
    void setStep(float step);

    int getLineWidth() const;
    void setLineWidth(int line_width);

    float getMajorThicksLength() const;
    void setMajorThickLength(float major_thick_length);

    void setAdditionalShiftSlot(double additional_shift)
    {
        std::cout << "alled with " << additional_shift << std::endl;
        setAdditionalShift((float) additional_shift);
        updateMajorBreaks();
    }



signals:
    void needRedrawing();


protected:

    virtual void drawMeOnly(CC_DRAW_CONTEXT &context);

    void drawMajorThicks(CC_DRAW_CONTEXT &context);

    void drawMajorThicksText(CC_DRAW_CONTEXT &context);

    virtual void applyGLTransformation(const ccGLMatrix& trans) {}

    virtual void setGLTransformation(const ccGLMatrix& trans) {}

//    ccCylinder getScalePiece( const colorType *color,const float min_sp, const float max_sp);

    void initMetadata();

    void initParameters();

    virtual void initEditDlg();



    void updateMajorBreaks() ;


    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(m_min_sp);
        ar & BOOST_SERIALIZATION_NVP(m_max_sp);
        ar & BOOST_SERIALIZATION_NVP(m_step);
        ar & BOOST_SERIALIZATION_NVP(m_line_width);
        ar & BOOST_SERIALIZATION_NVP(m_major_thicks_length);
        ar & BOOST_SERIALIZATION_NVP(m_breaks);
        ar & BOOST_SERIALIZATION_NVP(m_major_thicks_positions);
        ar & BOOST_SERIALIZATION_NVP(m_major_thicks_vector);
        ar & BOOST_SERIALIZATION_NVP(m_dynamic_scale);


        ar & boost::serialization::make_nvp("spcSingleAttitudeModel", boost::serialization::base_object<spc::spcSingleAttitudeModel> (*this));
        ar & boost::serialization::make_nvp("ccMyBaseObject", boost::serialization::base_object<ccMyBaseObject> (*this));

    }

    ////// user accessible props ////////////////////
    float m_min_sp;
    float m_max_sp;
    float m_step;

    int m_line_width;

    float m_major_thicks_length;


    //// these for internal use only /////////////////
    std::vector<float> m_breaks;

    std::vector<Eigen::Vector3f> m_major_thicks_positions;

    Eigen::Vector3f m_major_thicks_vector;

    float m_dynamic_scale;

};



#endif // CC_SINGLE_ATTITUDE_MODEL_H
