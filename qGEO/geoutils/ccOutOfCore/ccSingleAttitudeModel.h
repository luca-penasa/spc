#ifndef CC_SINGLE_ATTITUDE_MODEL_H
#define CC_SINGLE_ATTITUDE_MODEL_H

#include <ccHObject.h>

#include <spc/stratigraphy/single_attitude_model.h>
#include <ccCylinder.h>

class ccSingleAttitudeModel:  public ccHObject, public spc::SingleAttitudeModel
{
public:
    ccSingleAttitudeModel();

    ccSingleAttitudeModel(const SingleAttitudeModel &model);

    ccSingleAttitudeModel(const Plane *att);
    virtual bool isSerializable() const { return true; }
    virtual bool hasColors() const { return true; }
    virtual ccBBox getMyOwnBB();

protected:

    virtual void drawMeOnly(CC_DRAW_CONTEXT &context);

    virtual void applyGLTransformation(const ccGLMatrix& trans);

    virtual void setGLTransformation(const ccGLMatrix& trans);

    void drawScalePiece(CC_DRAW_CONTEXT &context, const colorType *color,const float radius,const float min_sp,const float max_sp);

    void drawScale(CC_DRAW_CONTEXT &context, const float radius,const float min_sp, const float max_sp);

    void initMetadata();

    void initParameters();


    unsigned int getNumberOfSegmentsBetween(const float min, const float max);


    float m_min_sp;
    float m_max_sp;
    float m_step;
    float m_pipe_radius;

    float m_dynamic_scale;


};

#endif // CC_SINGLE_ATTITUDE_MODEL_H
