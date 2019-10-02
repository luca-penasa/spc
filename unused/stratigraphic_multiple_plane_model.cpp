//#include <spc/geology/stratigraphic_multiple_plane_model.h>

//namespace spc
//{

//template <typename ScalarT>
//StratigraphicMultiplePlanesModel<ScalarT>::StratigraphicMultiplePlanesModel()
//{
//}

//template<typename ScalarT>
//ScalarT StratigraphicMultiplePlanesModel<ScalarT>::getAverageSPForReferenceCloud(int id)
//{

//    std::vector<double> predicted_sp = getPredictedSPForReferenceCloud(id);

//    double average = 0;
//    for (int i = 0 ;  i < predicted_sp.size(); ++i)
//    {
//        average += predicted_sp.at(i);
//    }
//    average /= predicted_sp.size();
//    return average;
//}

//template<typename ScalarT>
//std::vector<ScalarT>
//StratigraphicMultiplePlanesModel<ScalarT>::getAverageSPForAllClouds()
//{
//    std::vector<ScalarT> averages;
//    averages.resize(m_reference_clouds.size());
//    for (int i = 0; i < m_reference_clouds.size(); ++i)
//    {
//        averages.at(i) = this->getAverageSPForReferenceCloud(i);
//    }

//}

//template <typename ScalarT>
//std::vector<ScalarT>
//StratigraphicMultiplePlanesModel<ScalarT>::getPredictedSPForReferenceCloud(int id)
//{
//    std::vector<ScalarT> predicted_sp;
//    ccPointCloud * cloud = m_reference_clouds.at(id);
//    int n_points = cloud->size();

//    for (int i = 0 ;  i < n_points; ++i)
//    {
//        CCVector3 point;
//        cloud->getPoint(i, point);
//        predicted_sp.push_back( this->getSPAt(point) );

//    }

//    return predicted_sp;
//}

//std::vector< std::vector<double> >
//StratigraphicMultiplePlanesModel::getPredictedSPForAllReferenceClouds()
//{
//    std::vector< std::vector<double> >    container;
//    for (int i = 0; i < m_reference_clouds.size(); ++i)
//    {
//        container.push_back(getPredictedSPForReferenceCloud(i));
//    }

//    return container;
//}

//std::vector<double>
//StratigraphicMultiplePlanesModel::getPredictionErrorsForReferenceCloud(int id)
//{
//    std::vector<double> errors;

//    std::vector<double> predicted = getPredictedSPForReferenceCloud(id);
//    double averaged = getAverageSPForReferenceCloud(id);

//    for (int i = 0; i < predicted.size(); ++i)
//    {
//        errors.push_back(predicted.at(i) - averaged);
//    }

//    return errors;
//}

//template<typename ScalarT>
//auto
//ccStratigraphicMultiplePlanesModel::getOverallPredictionErrorsForAllReferenceClouds() -> ScalarField
//{
//    std::vector<double> all_errors;
//    for (int i = 0; i < m_reference_clouds.size(); ++i)
//    {
//        std::vector<double> this_errors = getPredictionErrorsForReferenceCloud(i);
//        all_errors.insert(all_errors.end(), this_errors.begin(), this_errors.end());
//    }
//    return all_errors;
//}

//template<typename ScalarT>
//auto StratigraphicMultiplePlanesModel<ScalarT>::getOverallSumOfSquares() -> ScalarT
//{
//    std::vector<double> all_errors;
//    double sum = 0;
//    for (int i = 0; i < all_errors.size(); ++i)
//    {
//        sum += all_errors.at(i) * all_errors.at(i);
//    }

//    return sum;
//}






//}//end nspace


