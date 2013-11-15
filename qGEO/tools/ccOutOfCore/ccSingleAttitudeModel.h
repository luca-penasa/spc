#ifndef CC_SINGLE_ATTITUDE_MODEL_H
#define CC_SINGLE_ATTITUDE_MODEL_H

#include <ccHObject.h>

#include <spc/stratigraphy/single_attitude_model.h>
#include <ccCylinder.h>
#include <ccOutOfCore/ccEditableHObject.h>

#include <QDialog>

#include <ccOutOfCore/ccMyBaseObject.h>
#include <QIcon>

class ccSingleAttitudeModel: public QObject,  public ccMyBaseObject, public spc::SingleAttitudeModel
{
    Q_OBJECT

public:


    ccSingleAttitudeModel();

    ccSingleAttitudeModel(const spc::SingleAttitudeModel &model);

    ccSingleAttitudeModel(const Plane *att);

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

    ////// user accessible props ////////////////////
    float m_min_sp;
    float m_max_sp;
    float m_step;

    int m_line_width;

    float m_major_thicks_length;


    //// these for internal use only /////////////////
    std::vector<float> m_breaks;

    std::vector<Vector3f> m_major_thicks_positions;

    Vector3f m_major_thicks_vector;

    float m_dynamic_scale;

};



#endif // CC_SINGLE_ATTITUDE_MODEL_H
