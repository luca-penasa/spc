#ifndef INTENSITYCALIBRATORRBFNONLINEAR_H
#define INTENSITYCALIBRATORRBFNONLINEAR_H


#include <spc/elements/RBFModel.h>
#include <spc/elements/calibration/CalibrationDataHolder.h>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>




namespace spc
{
namespace calibration
{



template <typename _Scalar, int NX = Dynamic, int NY = Dynamic> struct Functor
{
	typedef _Scalar Scalar;
	enum {
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};
	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix
	<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	const int m_inputs, m_values;

	Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime)
	{
	}
	Functor(int inputs, int values) : m_inputs(inputs), m_values(values)
	{
	}

	int inputs() const
	{
		return m_inputs;
	}
	int values() const
	{
		return m_values;
	}
};



class IntensityCalibratorRBFNonLinear
{
public:
	IntensityCalibratorRBFNonLinear();


	spcSetObjectMacro(Model, model_, RBFModel<float>)
	spcGetObjectMacro(Model, model_, RBFModel<float>)

	spcSetObjectMacro(CalibrationData, keypoint_holder_, CalibrationDataHolder)
	spcGetObjectMacro(CalibrationData, keypoint_holder_, CalibrationDataHolder)


	spcSetMacro(FixedMaterialId, fixed_material_id_, size_t)
	spcGetMacro(FixedMaterialId, fixed_material_id_, size_t)

	spcSetMacro(ValueOfFixedMat, value_for_fixed_material_, float)
	spcGetMacro(ValueOfFixedMat, value_for_fixed_material_, float)



	void optimize()
	{

		int outsize = keypoint_holder_->getTotalNumberOfEntries();
		my_functor Functor(this, model_->getCoefficients().rows(), outsize);

		Eigen::NumericalDiff<my_functor> numDiff(Functor);
		LevenbergMarquardt<Eigen::NumericalDiff<my_functor>, float> lm(numDiff);

		LOG(INFO) << "lev marq initialized";

		Eigen::VectorXf coeffs = model_->getCoefficients();
		LOG(INFO) << "starting minimization";
		int info = lm.minimize(coeffs);

		LOG(INFO) << "Done. Exit status: " << info;
	}

protected:

	struct my_functor : Functor<float>
	{
		my_functor(IntensityCalibratorRBFNonLinear * calibrator, int xsize, int ysize) : Functor<float>(xsize, ysize)
		{
			calibrator_ = calibrator;
		}

		int operator()(const VectorXf &x, VectorXf &fvec) const
		{
//			LOG(INFO)<< "CALLED functor, new pars: " << x.transpose();

			calibrator_->getModel()->setCoefficients(x);

			calibrator_->updateSecondary();

			fvec = calibrator_->getResiduals();

			LOG(INFO)<< "CALLED functor, residuals: " << fvec.sum();

			return 0;
		}

		/// a pointer to the estimator itself
		IntensityCalibratorRBFNonLinear * calibrator_;
	};

	void updateSecondary()
	{
		// update the corrected intensity for each observation.
		for (Observation::Ptr obs: keypoint_holder_->getAllObservations())
			obs->intensity_corrected = getCorrectiedIntensity(obs);


		// updating the material_factor for each material and keypoint
		Eigen::VectorXi materials = keypoint_holder_->getDefinedMaterials();

		for (int i = 0; i < materials.rows(); ++i)
		{
			int mat_id = materials(i);

			float mat_factor = spcNANMacro;

			if (mat_id == -1) // these are uncategorized materials
			{
				CalibrationDataHolder::Ptr kpoints = keypoint_holder_->getKeypointsOnMaterial(-1);
				for (CalibrationKeyPoint::Ptr kpoint: kpoints->getData())
				{
					float avg = 0;
					for (Observation::Ptr ob: kpoint->per_cloud_data)
						avg += ob->intensity_corrected;

					avg /= kpoint->per_cloud_data.size();

					kpoint->intensity_expected = avg;
				}
			}

			else
			{
				if (mat_id == fixed_material_id_)
				{
					mat_factor =  value_for_fixed_material_;
				}

				else
				{
					std::vector<Observation::Ptr> obs = keypoint_holder_->getKeypointsOnMaterial(mat_id)->getAllObservations();
					mat_factor = 0;

					for (Observation::Ptr ob: obs)
						mat_factor += ob->intensity_corrected;

					mat_factor /= obs.size();
				}


				CalibrationDataHolder::Ptr kpoints = keypoint_holder_->getKeypointsOnMaterial(mat_id);
				for (CalibrationKeyPoint::Ptr kpoint: kpoints->getData())
				{
					kpoint->intensity_expected = mat_factor;
				}
			}


		}



	}

	//! comptue the corrected intensity for the given observation
	float getCorrectiedIntensity(Observation::Ptr ob) const
	{
		bool also_angle = false;
		if (model_->getDimensionality() == 2)
			also_angle = true;

		Eigen::VectorXf p = ob->getAsEigenPoint(also_angle);

		float corrected = ob->intensity / model_->operator ()(p);

		if (ob->getParent()->material_id != -1)
			LOG(INFO) << "o: " << ob->intensity << " m: " << model_->operator ()(p)  <<  " c: " << corrected;

		return corrected;
	}

	Eigen::VectorXf getResiduals() const
	{
		Eigen::VectorXf out(keypoint_holder_->getTotalNumberOfEntries());

		size_t counter = 0;
		for (Observation::Ptr ob: keypoint_holder_->getAllObservations())
		{
			float diff = ob->getParent()->intensity_expected - ob->intensity_corrected;
			out(counter++) =  diff * diff;
		}

		return out;
	}


	RBFModel<float>::Ptr model_;

	CalibrationDataHolder::Ptr keypoint_holder_;

	size_t fixed_material_id_ = 0;

	float value_for_fixed_material_ = 1;
};


}
} // end nspaces


#endif // INTENSITYCALIBRATORRBFNONLINEAR_H
