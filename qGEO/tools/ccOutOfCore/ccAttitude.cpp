#include "ccAttitude.h"
#include <iostream>



ccAttitude::ccAttitude(CCVector3 center, CCVector3 orientation):
    spcAttitude(asEigenVector(orientation), asEigenVector(center))
{
    initMetadata();
    initParameters();
}



ccAttitude::ccAttitude(spc::spcAttitude att): spc::spcAttitude(att)
{    
    initMetadata();
    initParameters();
}


ccAttitude::ccAttitude()
{
    initMetadata();
    initParameters();
}

ccBBox ccAttitude::getMyOwnBB()
{
    CCVector3 center (position_.data());
    CCVector3 min_corner(center - m_scale * 0.5 * m_scale_factor);
    CCVector3 max_corner(center + m_scale * 0.5* m_scale_factor);
    ccBBox box(min_corner, max_corner);

    return box;
}

void ccAttitude::initParameters()
{
    m_scale_factor = 20;
    m_width = 4;
    m_scale = 0.0;
//    m_oldTransform.toIdentity();
}

void ccAttitude::initMetadata()
{
    QVariant var("An attitude of a geologic plane in space");
    setMetaData(QString("[qGEO][ccAttitude]"), var);
}

void ccAttitude::drawMeOnly(CC_DRAW_CONTEXT &context)
{
    m_scale = context.pickedPointsRadius;
    //we draw here a little 3d representation of the sensor
    if (MACRO_Draw3D(context))
    {
        bool pushName = MACRO_DrawEntityNames(context);

        if (pushName)
        {
            //not particulary fast
            if (MACRO_DrawFastNamesOnly(context))
                return;
            glPushName(getUniqueID());
        }


        glPushAttrib(GL_LINE_BIT);
        glLineWidth(m_width);


        //we draw the segments
        if (isSelected())
            glColor3ubv(ccColor::red);
        else
            glColor3ubv(ccColor::green);

        Vector3f pos = position_;

        Vector3f dip_v = this->getDipVector();
        Vector3f strike_v = this->getStrikeVector();


        Vector3f arr_shaft = pos + dip_v * m_scale * m_scale_factor ;
        Vector3f strike_dir = pos + strike_v * m_scale * 0.5 * m_scale_factor ;
        Vector3f s_opp = pos - strike_v * m_scale * 0.5* m_scale_factor;

        context._win->display3DLabel(getDipAndDipAngleAsString().c_str(), CCVector3(pos(0), pos(1), pos(2)), ccColor::red);


        glBegin(GL_LINES);
        glColor3ubv(ccColor::red);

        glVertex3fv( pos.data() );
        glVertex3fv( arr_shaft.data() );


        glColor3ubv(ccColor::blue);


        glVertex3fv( pos.data());
        glVertex3fv( strike_dir.data());

        glVertex3fv( pos.data());
        glVertex3fv( s_opp.data() );
        glEnd();

        glPopAttrib();

        if (pushName)
            glPopName();

    }
}

void ccAttitude::applyGLTransformation(const ccGLMatrix &trans)
{

//    std::cout << " called apply gl trans" << std::endl;
//    Vector3f p = getPosition();
//    Vector3f n = getNormal();


//    CCVector3 position (p(0), p(1), p(2));
//    CCVector3 normal (n(0), n(1), n(2));

//    trans.apply(position);
//    trans.transposed().applyRotation(normal);

//    this->setNormal(Vector3f(normal.x, normal.y, normal.z));
//    this->setPosition(Vector3f(position.x, position.y, position.z ));



}

void ccAttitude::setGLTransformation(const ccGLMatrix &trans)
{
//    ccGLMatrix oldmatrix = m_oldTransform;
//    ccGLMatrix newmatrix =  oldmatrix.inverse() * trans ;

//    applyGLTransformation(newmatrix);
//    m_oldTransform = trans;

}



BOOST_CLASS_EXPORT_GUID(ccAttitude, "ccAttitude")


