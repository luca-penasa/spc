#include "normal_estimator.h"

namespace spc
{

NormalEstimator::NormalEstimator(): avg_rms_(0.0)
{
    model_ = StratigraphicNormalModel();

}


int NormalEstimator::estimateNormalAsAverage()
{
    normals_.clear();
    rmss_.clear();
    for (int i  = 0; i < planar_clouds_.size(); ++i)
    {
        float rms;
        GeologicPlane::Ptr normal = fromSingleCloud(planar_clouds_.at(i), rms);
        if (!normal) //if we cannot get a normal for this plane, exlude it from computations!
        {
            std::cout << "WARN: cannot estimate the plane, not enough points?" << std::endl;
            continue; //we go ahead with the other planes, that may be good!
        }
        normals_.push_back(*normal);
        rmss_.push_back(rms);
    }

    size_t n_planes = normals_.size();

    if (n_planes == 0)
        return 0; //we do not have any estimated normal.

    //now we do an average of both the rmms and the normals we have estimated
    GeologicPlane n_avg = std::accumulate(normals_.begin() + 1, normals_.end(), *normals_.begin());


    n_avg.setNormal(n_avg.getNormal() / n_planes);
    n_avg.normalize();

    n_avg.setPosition(n_avg.getPosition() / n_planes);

    avg_rms_ = std::accumulate(rmss_.begin(), rmss_.end(), 0.0);
    avg_rms_ /= n_planes;

    //set up the model!
    model_.setNormal(n_avg.getNormal());
    model_.setPosition(n_avg.getPosition());

    return 1;

}

int NormalEstimator::estimateNormalAsNonlinearOpt()
{

//        get a first estimate of the normal
    int status = estimateNormalAsAverage(); //could be a good initial approx
    if (status == 0)
    {
        std::cout << "cannot estimate normal as average for init the nonlinear opt";
        return 0;
    }


//    this->estimateStratigraphicPositionsOfPlanes();
//    this->computeSquaredErrors();



    VectorXf start = this->model_.getNormal();


    my_functor Functor(this);



    std::cout << "BEFORE: "<< start(0) << " " << start(1) << " " << start(2) << std::endl;
    Eigen::NumericalDiff<my_functor> numDiff(Functor);
    LevenbergMarquardt<Eigen::NumericalDiff<my_functor>, float> lm(numDiff);


    int info = lm.minimize(start);

    std::cout << "AFTER: " << start(0) << " " << start(1) << " " << start(2) << "with info " << info << std::endl;


    return info;
}


void NormalEstimator::estimateStratigraphicPositionsOfPlanes()
{
    stratigraphic_positions_.clear();
    stratigraphic_positions_.resize(planar_clouds_.size());


    for (int i =0 ; i < planar_clouds_.size(); ++i)
    {

        pcl::PointCloud<pcl::PointXYZ> cloud = *planar_clouds_[i];

        float accumulator = 0;
        for (int j = 0; j < cloud.size(); ++j)
        {
            pcl::PointXYZ p = cloud.at(j);
            Eigen::Vector3f ep(p.x, p.y, p.z);

            accumulator += model_.getStratigraphicPosition(ep);
        }


        //now do the average and save the result
        accumulator /= cloud.size();



        stratigraphic_positions_.at(i) = accumulator;

    }

}


//for the current model
float NormalEstimator::getOverallSquaredError()
{
    if (plane_squared_errors_.size() == 0)
        computeSquaredErrors();


    float err = std::accumulate(plane_squared_errors_.begin(), plane_squared_errors_.end(), 0.0f);
    return err;
}

void NormalEstimator::computeSquaredErrors()
{
    plane_squared_errors_.clear();
    plane_squared_errors_.resize(planar_clouds_.size());

    if (stratigraphic_positions_.empty())
        estimateStratigraphicPositionsOfPlanes();

    for (int i =0 ; i < planar_clouds_.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud = *(planar_clouds_[i]);

        float accumulator = 0.0f;
        for (int j = 0; j < cloud.size(); ++j)
        {

            pcl::PointXYZ p = cloud.at(j);
            Eigen::Vector3f ep(p.x, p.y, p.z);

            float sp = stratigraphic_positions_.at(i);

            float sp_model =  model_.getStratigraphicPosition(ep);
            float diff = sp - sp_model;

            accumulator += (diff*diff); //squared deviation
        }

        plane_squared_errors_.at(i) = accumulator;
    }

}


GeologicPlane::Ptr
NormalEstimator::fromSingleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr one_plane_cloud, float &rms)
{

    if (one_plane_cloud->size() < 3)
    {
        return GeologicPlane::Ptr(); //null pointer
    }

    GeologicPlane::Ptr out (new GeologicPlane);
    Eigen::Vector4f n, centroid;
    float c;
    pcl::computePointNormal(*one_plane_cloud, n, c);
    pcl::compute3DCentroid(*one_plane_cloud, centroid);


    Eigen::Vector3f no = n.segment(0,3);
    Eigen::Vector3f ce = centroid.segment(0,3);


    if(no(2) < 0.0)
        no *= -1.0; //always with a positive 'Z' by default!

    out->setPosition(ce);
    out->setNormal(no);
    out->normalize();

    return out;
}


}
