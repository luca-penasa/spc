#include <spc/methods/TimeSeriesGenerator.h>
#include <spc/elements/CloudDataSourceOnDisk.h>
#include <spc/elements/StratigraphicModelSingleAttitude.h>
#include <spc/io/element_io.h>

using namespace spc;

int main (int argc, char ** argv)
{
	// cloud data source on disk from which to read
	CloudDataSourceOnDisk cloud_ref("path/to/cloud.pcd");

	// load the cloud
	PointCloudBase::Ptr cloud = cloud_ref.load();

	// we set up the stratigraphic reference frame using a single attitude,
	// in this case an horizontal plane, centered in [0,0,0]
	Attitude attitude ({0,0,1}, {0,0,0});

	// we also need a stratigraphic reference frame, we use a single attitude model
	StratigraphicModelSingleAttitude::Ptr model(new StratigraphicModelSingleAttitude(attitude));

	// set up a generator
	TimeSeriesGenerator generator;
	generator.setInputCloud(cloud);
	generator.setStratigraphicModel(model);

	// we could also set up a region of interest
	// if we wanted to use a smaller region to perform the reconstruction
	//generator.setSelection(...);

	// setthe time series parameters
	generator.setBandwidth(0.01); // bandwidth 1 cm
	generator.setSamplingStep(0.01); // sampling step 1 cm

	// set the name of the scalar field to be logged
	// we could use any scalar field we have within the cloud
	generator.setYFieldName("intensity");

	// do the time series computation
	generator.compute();

	// we have now a time series
	TimeSeriesEquallySpaced::Ptr time_series  = generator.getOutputSeries();

	// we can save it in an ASCII format so to use in any extenal
	// time series analysis package, but we could use XML, or JSON also
	io::serializeToFile(time_series, "time-series.csv", io::ASCII);

	return 1;
}
