#include "PointCloudSpc.h"
namespace spc
{
PointCloudSpc::PointCloudSpc()
    : fields_manager_(new EigenTable), x_field_name_("x"),
      y_field_name_("y"), z_field_name_("z")
{
}

} // end nspace
