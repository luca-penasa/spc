#include "attitude_estimator.h"

namespace spc
{

AttitudeEstimator::AttitudeEstimator()
{
}

std::vector<spcAttitude> AttitudeEstimator::getEstimatedAttitudes()
{
    std::vector<spcAttitude> atts;
    spcAttitude att = getEstimatedAttitude();
    Vector3f n =  att.getUnitNormal();

    for (int i = 0 ; i < clouds_.size(); ++i)
    {
        spcAttitude new_att(n, centroids_.at(i));
        atts.push_back(new_att);
    }

    return atts;
}

void AttitudeEstimator::initializeModel()
{
    //for each cloud we estimate a model. then we average
    //clearly only for the clouds that have at least 3 points

    int counter = 0;
    std::vector<Vector3f> normals;

    for (int i = 0; i < clouds_.size(); ++i)
    {
        CloudPtrT cloud = clouds_.at(i);
        if (cloud->size() < 3)
            continue;
        else
        {
            spcNormal3D n;
            n.normalFromBestFit(*cloud);
            normals.push_back(n.getUnitNormal());
        }
    }


    //acumualte normals
    Vector3f sum = std::accumulate(normals.begin(), normals.end(), Vector3f (0,0,0));

    sum /= normals.size(); //the average

    model_.setNormal(sum); //setunitnormal automatically normalize the vector
}

void AttitudeEstimator::updateSPs()
{
    s_positions_.clear();
    for (CloudPtrT cloud: clouds_)
    {
        auto vec = model_.getStratigraphicPositions(cloud);
        s_positions_.push_back(vec);
    }
}

void AttitudeEstimator::computeAveragedSPs()
{
    updateSPs(); //be sure are updated

    est_s_positions_.clear(); //clear it

    for (std::vector<float> sps: s_positions_)
    {
        float avg_sp = std::accumulate(sps.begin(), sps.end(), 0.0);
        avg_sp /= sps.size();
        est_s_positions_.push_back(avg_sp);
    }
}

void AttitudeEstimator::updateDeviations()
{
    computeAveragedSPs(); //be sure are updated
    sq_deviations_.clear();

    for (int i = 0; i < clouds_.size(); ++i)
    {
        std::vector<float> devs;
        CloudPtrT cloud = clouds_.at(i);
        float est_sp = est_s_positions_.at(i); //its sp
        for (float this_sp: s_positions_.at(i))
        {
            float diff = est_sp - this_sp;
            devs.push_back(diff*diff);
        }

        sq_deviations_.push_back(devs);

    }
}

void AttitudeEstimator::addInputCloud(AttitudeEstimator::CloudPtrT cloud)
{
    clouds_.push_back(cloud);
    //compute its centroid

    Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    centroids_.push_back(centroid.head(3));

}

float AttitudeEstimator::getAveragedSquareDeviations()
{
    updateDeviations(); //be sure are updated
    float acc = 0;
    int counter =0;
    for (std::vector<float> devs: sq_deviations_)
    {
        acc += std::accumulate(devs.begin(), devs.end(), 0.0f);
        counter += devs.size();
    }
    acc /= counter; //the avg
    return acc;
}

std::vector<float> AttitudeEstimator::getStratigraphicPositionsOfClouds()
{
    return est_s_positions_;
}

int AttitudeEstimator::estimate()
{
    initializeModel();

    VectorXf start = model_.getUnitNormal();

    my_functor Functor(this);

    std::cout << "BEFORE: "<< start(0) << " " << start(1) << " " << start(2) << std::endl;
    Eigen::NumericalDiff<my_functor> numDiff(Functor);
    LevenbergMarquardt<Eigen::NumericalDiff<my_functor>, float> lm(numDiff);


    int info = lm.minimize(start);

    std::cout << "AFTER: " << start(0) << " " << start(1) << " " << start(2) << "with info " << info << std::endl;

    return info;
}

}//end nspace
