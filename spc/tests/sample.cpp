#include <spc/elements/EigenTable.h>

int main(int argc, char **argv)

{

   spc::EigenTable table;
   table.resize(10);

      table.addNewComponent("x", 1);
      table.addNewComponent("n", 3);

   std::cout << table.mat() << std::endl;

   return 1;



   std::cout << table.getColumnDimensionality("n") << std::endl;
   for (auto s: table.getScalarColumnsNames())
       std::cout << s.c_str() << std::endl;

   std::cout << table.getColumnId("x") << std::endl;

   std::cout << table.getColumnId("n") << std::endl;

    Eigen::Vector3f v;
    v(0) = 7;
    v(1) = 3;
    v(2) = 5;

   table.atVector("n" , 2) = v;
   std::cout << table.atVector("n", 2) << std::endl;

   std::cout << table.mat() << std::endl;

    table.resize(12);

std::cout << table.mat() << std::endl;

    return 1;
}
