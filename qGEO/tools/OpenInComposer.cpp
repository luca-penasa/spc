#include "OpenInComposer.h"


SaveSPCElement::SaveSPCElement(ccPluginInterface * parent_plugin): BaseFilter(FilterDescription(   "Open Selected Time series in Composer",
                                                                                                   "Open Selected Time series in Composer",
                                                                                                   "Open Selected Time series in Composer",
                                                                                                   ":/toolbar/icons/tseries_generator.png")
                                                                              , parent_plugin)
{

    setShowProgressBar(false);

}
