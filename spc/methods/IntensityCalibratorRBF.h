#ifndef INTENSITYCALIBRATORRBF_H
#define INTENSITYCALIBRATORRBF_H

#include <spc/core/macros_ptr.h>
#include <spc/core/spc_eigen.h>

#include <spc/elements/RBFModel.h>

namespace spc
{



namespace calibration
{


spcFwdDeclSharedPtr(DataHolder)
spcFwdDeclSharedPtr(KeyPoint)
spcFwdDeclSharedPtr(Observation)


class IntensityCalibratorRBF
{


public:
    IntensityCalibratorRBF();



    void setCalibrationData(const DataHolderPtr clouddata)
    {
        calibration_data_ = clouddata;
    }


    static  void
    extractVariablesAsMatrix(DataHolderPtr holder,
                             Eigen::Matrix<float, -1, -1> &points,
                             Eigen::Matrix<float, -1, 1> &intensity,
                             Eigen::Matrix<float, -1, 1> &weights,
                             bool also_angle,
                             bool extract_weights);

	//! compute a weighting coefficient for the given observation
    static float
    computeWeight(ObservationPtr per_cloud_data);

    static void
    //! extract the variables distance, angle and intensity for the "data" keypoint.
    //! if also_angle == true also the angle is extracted into the matrix
    //! points matrix will contain the distance and optionally the angle as a n x 1 (or 2) column vector.
    //! the intensity will be placed in the intensity column vector
    extractVariablesAsMatrix(KeyPointPtr data,
                             Eigen::Matrix<float, -1, -1> &points,
                             Eigen::Matrix<float, -1, 1> &intensity,
                             Eigen::Matrix<float, -1, 1> &weights,
                             bool also_angle,
                             bool extract_weights
                             );


    int calibrate();


    RBFModel<float>::Ptr getModel() const
    {
        return model_;
    }

    spcSetMacro (UseAngle, use_angle, bool)
    spcSetMacro (UseWeights, use_weights, bool)
    spcSetMacro (AppendUndefMaterialsConstraints, append_undef_material_constraints, bool)
    spcSetMacro (AppendAdditionalMaterialsConstraints, append_additional_materials_constraints, bool)
    spcSetMacro (NAngleSplits, n_splits_angle_, size_t)
    spcSetMacro (NDistanceSplits, n_splits_distance_, size_t)

    spcSetMacro (Lambda, lambda_, float)
    spcSetMacro (ManualSigma, sigma_, float)

	spcSetMacro (IntensiytShift, intensity_shift, float)
	spcGetMacro (IntensiytShift, intensity_shift, float)


    spcSetMacro (InitSetMaterial, init_set_material_id, int)

	spcSetMacro(AdditionalMaterialsWeight, additional_materials_weight, float)
	spcGetMacro(AdditionalMaterialsWeight, additional_materials_weight, float)

	spcSetMacro(UndefMaterialsWeight, undef_material_weight, float)
	spcGetMacro(UndefMaterialsWeight, undef_material_weight, float)

	spcSetMacro(PolyOrder, poly_order_, size_t)
	spcGetMacro(PolyOrder, poly_order_, size_t)

protected:

    //! we keep the calibration data in its cloud representation cause its easier to
    //! use and more portable (and a cloud representation can be easily created from
    //! the CalibrationDataHolder, but not vice-versa for now)
    DataHolderPtr calibration_data_;


    bool use_angle = true;
    bool use_weights = true;

    bool append_undef_material_constraints = true;
	float undef_material_weight = 1;

    bool append_additional_materials_constraints = true;
	float additional_materials_weight = 1;

    bool has_materials;

    size_t n_splits_angle_ = 5;
    size_t n_splits_distance_ = 6;

    size_t poly_order_ = 1;

    float lambda_ = 0.00; //! < no lambda by def
    float sigma_ =0.0; //! < this will force the auto-mode by default


    int init_set_material_id = 0;

    RBFModel<float>::Ptr model_;

	//! u_shift is summed with the intensity, before modeling. This permits to shift to the positive side
	//! the intensities of some devices which are not in a positive range.
	float intensity_shift = 0;

};





}
}
#endif // INTENSITYCALIBRATORRBF_H
