#include "SaveSPCElement.h"


SaveSPCElement::SaveSPCElement(ccPluginInterface * parent_plugin): BaseFilter(FilterDescription(   "Save selected as SPC elemnts",
                                                                                                   "Save selected as SPC elemnts",
                                                                                                   "Save selected as SPC elemnts",
                                                                                                   ":/toolbar/icons/save.png")
                                                                              , parent_plugin)
{

    setShowProgressBar(false);

}
