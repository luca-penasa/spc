#include <spc/elements/Attitude.h> // for accessing the attitude class
#include <spc/elements/StratigraphicModelSingleAttitude.h> // and the class defining a type of reference frame

#include <spc/io/element_io.h> // we will demonstrate also IO capabilities of SPC
#include <spc/core/logging.h> // and we need this to log to console using GLOG

using namespace spc;

int main( int argc, char ** argv)
{
	// we use google logging for flexible logs
	google::InitGoogleLogging(argv[0]);
	google::LogToStderr(); // force to log to stderr

	// create (a pointer to) an attiude $140N/40$, centerd in origin we use smart pointers to help manage memory in a good way
	Attitude::Ptr attitude (new Attitude(40, 140, {0,0,0}));

	// print out the dip and the dip-angle
	LOG(INFO) << "dip: " << attitude->getDip() << " dip angle: " << attitude->getDipAngle();

	// and the corrispondent unit normal
	LOG(INFO) << "normal: " << attitude->getUnitNormal().transpose();

	// and the position
	LOG(INFO) << "position: " << attitude->getPosition().transpose();

	// we can move the attitude in another location:
	Eigen::Vector3f new_position;
	new_position << 1.2, 10.22, 4.2;
	attitude->setPosition(new_position);

	// and the position will be
	LOG(INFO) << "position after moving: " << attitude->getPosition().transpose();

	// obtain the strike vector, which is obviously in the $XY$ plane:
	LOG(INFO) << "the strike vector: " << attitude->getStrikeVector().transpose();

	// we can create a Stratigraphic Reference Frame, in this case a model based on a single measure of the local attitude:
	StratigraphicModelSingleAttitude::Ptr stratigraphic_reference_frame (new StratigraphicModelSingleAttitude(*attitude));

	// the reference frame will have the same normal of the attitude:
	LOG(INFO) << "srf normal: " << stratigraphic_reference_frame->getNormal().transpose();

	// we can now use the reference frame to project any 3d point into the stratigraphic domain
	Eigen::Vector3f point;
	point << 2, 1, 5.5;

	// the stratigraphic position is a scalar field defied as the distance from a plane
	float stratigraphic_position = stratigraphic_reference_frame->getScalarFieldValue(point);

	LOG(INFO) << "the point is: " << point.transpose() << " and its strat. position: " << stratigraphic_position;

	// if we use this scalar to set the $d$ parameter of the model we can force the model to a given stratigraphic position
	stratigraphic_reference_frame->setStratigraphicShift(- stratigraphic_position);

	// now the model will predict a $0.0$ stratigraphic position for the same point
	stratigraphic_position = stratigraphic_reference_frame->getScalarFieldValue(point);

	// basically we have defined the model so that a point in space will have $0.0$ stratigraphic position
	LOG(INFO) << "the point is: " << point.transpose() << " and its strat. position: " << stratigraphic_position;


	// we can save the objects we created, the extension will be automatically added, depending on the format
	io::serializeToFile(attitude, "attitude", io::XML ); // using XML in this case

	// which can be loaded back:
	ISerializable::Ptr reloaded = io::deserializeFromFile("attitude.xml");
	Attitude::Ptr reloaded_attitude  = spcDynamicPointerCast<spc::Attitude> (reloaded);

	if (reloaded_attitude != NULL)
	{
		LOG(INFO) << "reloading successfull, normal is: " << reloaded_attitude->getUnitNormal().transpose();
	}

	return 1;
}
