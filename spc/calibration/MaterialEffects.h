#ifndef MATERIALEFFECTS_H
#define MATERIALEFFECTS_H
#include <Eigen/Eigen>

namespace spc
{


template <typename ScalarT>
class MaterialEffects
{
public:

    typedef Eigen::Matrix<ScalarT, -1, 1> VectorT;


    MaterialEffects()
    {
    }

    ScalarT operator ()(const size_t &mat_index) const
    {
        if (mat_index == fixed_material_)
            return fixed_mat_value;
        else
            return material_coefficients_(mat_index);
    }



protected:
    VectorT material_coefficients_;

    size_t fixed_material_ = 0;

    ScalarT fixed_mat_value = ScalarT(1);


};
}//end nspace
#endif // MATERIALEFFECTS_H
