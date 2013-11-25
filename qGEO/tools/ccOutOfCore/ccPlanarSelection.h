#ifndef CCPLANARSELECTION_H
#define CCPLANARSELECTION_H


#include <spc/elements/spcPlanarSelection.h>
#include <ccOutOfCore/ccMyBaseObject.h>
#include <ccPolyline.h>

#include <pcl/filters/extract_indices.h>

class ccPlanarSelection: public ccMyBaseObject, public spc::spcPlanarSelection
{
public:
    ccPlanarSelection();

    virtual bool isSerializable() const { return true; }
    virtual bool hasColors() const { return true; }
    virtual ccBBox getMyOwnBB()
    {
        Vector4f mincorner;
        Vector4f maxcorner;

        pcl::getMinMax3D(getVertices(), mincorner, maxcorner);

        CCVector3 min = CCVector3(mincorner.data());
        CCVector3 max = CCVector3(maxcorner.data());
        return ccBBox(min, max);
    }


    virtual QIcon * getIcon() const
    {
        return new QIcon(QString::fromUtf8(":/toolbar/icons/selection.png"));
    }

    virtual void drawMeOnly(CC_DRAW_CONTEXT &context)
    {
        unsigned vertCount = getVertices().size();
        if (vertCount < 2)
            return;

        if (colorsShown())
            glColor3ubv(m_rgbColor);

        if (m_width != 0)
        {
            glPushAttrib(GL_LINE_BIT);
            glLineWidth(static_cast<GLfloat>(m_width));
        }

        glBegin(GL_LINE_LOOP);

        for (auto p: getVertices())
            ccGL::Vertex3v(p.data);


        glEnd();

        if (m_width != 0)
        {
            glPopAttrib();
        }
    }

    void setColor(const colorType col[])
    {
        memcpy(m_rgbColor,col,sizeof(colorType)*3);
    }

    void setWidth(PointCoordinateType width)
    {
        m_width = width;
    }

    const colorType* getColor() const
    {
        return m_rgbColor;
    }




protected:
    virtual void applyGLTransformation(const ccGLMatrix& trans) {}
    virtual void setGLTransformation(const ccGLMatrix& trans) {}

    //! Unique RGB color
    colorType m_rgbColor[3];

    //! Width of the line
    PointCoordinateType m_width;


    //! Whether poyline should draws itself in background (false) or foreground (true)
    bool m_foreground;


};

#endif // CCPLANARSELECTION_H
