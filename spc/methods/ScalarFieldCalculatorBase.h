#ifndef SCALARFIELDCALCULATORBASE_H
#define SCALARFIELDCALCULATORBASE_H

#include <spc/core/spc_eigen.h>
#include <spc/elements/PointCloudBase.h>
#include <spc/core/strings.h>

namespace spc {
namespace methods {

	class FilterDescriptor {
	public:
		FilterDescriptor()
		{
		}

		FilterDescriptor(const std::string& filter_name,
			const std::string& filter_description,
			const int number_of_input_fields = -1,
			const int number_of_output_fields = -1,
			const std::vector<std::string>& in_fields_names = {},
			const std::vector<std::string>& in_fields_descriptions = {},
			const std::vector<std::string>& out_fields_names = {},
			const std::vector<std::string>& out_fields_descriptions = {})
			: filter_name_(filter_name)
			, filter_description_(filter_description)
			, number_of_input_fields_(number_of_input_fields)
			, number_of_output_fields_(number_of_output_fields)
			, in_fields_names_(in_fields_names)
			, in_fields_descriptions_(in_fields_descriptions)
			, out_fields_names_(out_fields_names)
			, out_fields_descriptions_(out_fields_descriptions)
		{
		}

		std::string filter_name_;
		std::string filter_description_;

		int number_of_input_fields_; //-1 is unknow at compile time
		int number_of_output_fields_; // -1 is unknown at compile time

		std::vector<std::string> in_fields_names_; // not required if unknown at compile time
		std::vector<std::string> in_fields_descriptions_; // not required if unknown at compile time

		std::vector<std::string> out_fields_names_; // as above
		std::vector<std::string> out_fields_descriptions_; // as above
	};

	class ScalarFieldCalculatorBase {
	public:
		ScalarFieldCalculatorBase(const FilterDescriptor& descriptor)
		{
			descriptor_ = descriptor;
		}

		void SetInputCloud(const spc::PointCloudBase::Ptr& cloud)
		{
			cloud_ = cloud;
		}
		spc::PointCloudBase::Ptr getCloud() const
		{
			if (!cloud_)
				LOG(WARNING) << "Returning null pointer";

			return cloud_;
		}

		std::vector<std::string> getAvailableFields() const
		{
			if (!cloud_)
				LOG(WARNING) << "No cloud was set. returning empty list";

			return cloud_->getFieldNames();
		}

		Eigen::VectorXf getField(const std::string& fname) const
		{

			Eigen::VectorXf v;
			cloud_->getField(fname, v);

			if (v.rows() == 0)
				LOG(WARNING) << "looks like the field was returned empty";

			return v;
		}

		/** this must be reimplemented by other calculators
	 **/
		virtual void compute() = 0;

	private:
		spc::PointCloudBase::Ptr cloud_;
		FilterDescriptor descriptor_;
	};



} //end nspace methods
} // end nspace spc

#endif // SCALARFIELDCALCULATORBASE_H
