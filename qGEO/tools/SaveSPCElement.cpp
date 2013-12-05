#include "SaveSPCElement.h"


SaveSPCElement::SaveSPCElement(ccPluginInterface * parent_plugin): BaseFilter(FilterDescription(   "Save selected as SPC elments",
                                                                                                   "Save selected as SPC elments",
                                                                                                   "Save selected as SPC elments",
                                                                                                   ":/toolbar/icons/save.png")
                                                                              , parent_plugin)
{

    setShowProgressBar(false);

}
