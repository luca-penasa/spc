#include "ccSingleAttitudeModel.h"
#include <ccPointCloud.h>
static QSharedPointer<ccCylinder> cyl(0);

ccSingleAttitudeModel::ccSingleAttitudeModel()
{

    initMetadata();
    initParameters();
}



ccSingleAttitudeModel::ccSingleAttitudeModel(const spc::SingleAttitudeModel &model)
{
    initMetadata();
    setPosition(model.getPosition());
    setNormal(model.getUnitNormal());
    initParameters();

}

ccSingleAttitudeModel::ccSingleAttitudeModel(const spc::Plane *att)
{
    initMetadata();
    setPosition(att->getPosition());
    setNormal(att->getUnitNormal());
    initParameters();
}

ccBBox ccSingleAttitudeModel::getMyOwnBB()
{
    CCVector3 center (position_.data());
    CCVector3 min_corner(center - 0.5);
    CCVector3 max_corner(center + 0.5);
    ccBBox box(min_corner, max_corner);

    return box;
}

void ccSingleAttitudeModel::drawMeOnly(CC_DRAW_CONTEXT &context)
{

    m_dynamic_scale = context.pickedPointsRadius;

    std::cout << "drawing model" << std::endl;
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


        drawScale(context, m_pipe_radius, m_min_sp, m_max_sp );

        if (pushName)
            glPopName();



    }
}

void ccSingleAttitudeModel::applyGLTransformation(const ccGLMatrix &trans)
{

}

void ccSingleAttitudeModel::setGLTransformation(const ccGLMatrix &trans)
{

}

void ccSingleAttitudeModel::drawScalePiece(CC_DRAW_CONTEXT &context, const colorType * color, const float radius, const float min_sp, const float max_sp)
{
    Vector3f b = getPointAtStratigraphicPosition(min_sp);

    float height = max_sp - min_sp;

    ccGLMatrix  T ;
    T.setTranslation(CCVector3(0,0,height*0.5)); //now the base of the cyclinder is in 0 0 0

    //the rotation aligning the cyclinder with the normal
    ccGLMatrix R = ccGLMatrix::FromToRotation(CCVector3(getUnitNormal().data()), CCVector3(0.0,0.0,1.0));
    //the rotation must be applied on the center
    R.shiftRotationCenter(CCVector3(0,0,0));

    ccGLMatrix T2;
    T2.setTranslation(CCVector3(b.data())); // now move the cylinder to its positionS

    ccGLMatrix finalTr = T2*R*T;



    cyl = QSharedPointer<ccCylinder>(new ccCylinder(radius, height, &finalTr));

    std::cout << cyl->getAssociatedCloud()->size() << std::endl;

    cyl->setDisplay(this->getDisplay());
    cyl->setTempColor(color);
    cyl->showColors(true);


    cyl->draw(context);
}

void ccSingleAttitudeModel::drawScale(CC_DRAW_CONTEXT &context, const float radius, const float min_sp, const float max_sp)
{
    assert(max_sp > min_sp);

    std::vector<float> breaks;

    float intpart;
    modf((min_sp / m_step), &intpart);
    float start_break = intpart * m_step;

    std::cout << "break start at " << start_break << std::endl;



    breaks.push_back(min_sp);
    float curr_pos = start_break;
    while (curr_pos < max_sp)
    {
        breaks.push_back(curr_pos);
        curr_pos +=m_step;
    }

    breaks.push_back(max_sp);

    for (int i = 0; i < breaks.size() - 1; ++i)
    {
        if (i % 2)
            drawScalePiece(context, ccColor::blue, m_pipe_radius, breaks.at(i), breaks.at(i+1));
        else
            drawScalePiece(context, ccColor::yellow, m_pipe_radius, breaks.at(i), breaks.at(i+1));
    }

}

void ccSingleAttitudeModel::initMetadata()
{
    QVariant var(QString("A stratigrahic model"));

    setMetaData(QString("[qGEO][ccSingleAttitudeModel]"), var);
}

void ccSingleAttitudeModel::initParameters()
{
    m_min_sp =-10;
    m_max_sp = 10;
    m_step = 1;
    m_pipe_radius = 0.1;
    setVisible(true);

}

unsigned int ccSingleAttitudeModel::getNumberOfSegmentsBetween(const float min, const float max)
{
    assert (max > min);

    float r = max -min;
    r / m_step;
    return floor(r);
}





