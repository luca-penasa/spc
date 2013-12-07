#include "ccPlanarSelection.h"

ccPlanarSelection::ccPlanarSelection()
{

    QVariant var("An area selection");
    setMetaData(QString("[qGEO][ccPlanarSelection]"), var);

    m_foreground = false;
    m_width  = 0.0;
    memcpy(m_rgbColor, ccColor::green, sizeof(colorType) * 3);

    setVisible(true);

}



BOOST_CLASS_EXPORT_GUID(ccPlanarSelection, "ccPlanarSelection")
