#pragma once
#ifndef STRATIGRAPHICLINK_H
#define STRATIGRAPHICLINK_H

#include<spc/elements/MovableElement.h>
#include <spc/elements/templated/PointSet.h>

#include <spc/elements/StratigraphicPositionableElement.h>

#include <spc/elements/PolyLine3D.h>

#include <cereal/cereal.hpp>
#include <spc/elements/LinearGeologicalFeature.h>
#include <spc/elements/Sample.h>

namespace spc
{


class StratigraphicConstrain: public ElementBase
{
public:

    SPC_ELEMENT(StratigraphicConstrain)
    EXPOSE_TYPE

    StratigraphicConstrain()
    {

    }

    void addVertex(StratigraphicPositionableElement::Ptr v)
    {
        vertices_.push_back(v);
        updatePointSetRepresentation();
    }

    size_t getNumberOfConstrains() const
    {
        return vertices_.size();
    }


    void updatePointSetRepresentation()
    {

        polyline_rep_.resize(0); // like cleaning

        LOG(INFO) << "updating point representation";
        for (StratigraphicPositionableElement::Ptr p: vertices_)
        {

            if (p->isA(&spc::LinearGeologicalFeature::Type))
            {
                spc::LinearGeologicalFeature::Ptr aslin = spcDynamicPointerCast<spc::LinearGeologicalFeature> (p);

                if (aslin->getPolyline().getNumberOfPoints() != 0)
                    polyline_rep_.addPoint(aslin->getPolyline().getPoint(0));
            }
            else if (p->isA(&spc::Sample::Type))
            {
                spc::Sample::Ptr sample = spcDynamicPointerCast<spc::Sample> (p);

                polyline_rep_.addPoint(sample->getPosition());
            }



        }
    }

    std::vector<StratigraphicPositionableElement::Ptr> getVertices() const
    {
        return vertices_;
    }


    PolyLine3D getPolyLineRep() const
    {
        return polyline_rep_;
    }

    // each constrain may provide a variable number of residuals
    // depending on the number of entities it is constraining
    Eigen::VectorXf getSquaredResiduals() const
    {

        Eigen::VectorXf residuals;

        float avg = 0;
        for (int i = 0; i < vertices_.size(); ++i)
        {
            spc::StratigraphicPositionableElement::Ptr el = vertices_.at(i);
            avg += el->getSquaredResidual();


        }

          residuals.resize(1);
          residuals(0) = avg/ vertices_.size();


//        std::map<spc::StratigraphicModelBase::Ptr, std::vector<spc::StratigraphicPositionableElement::Ptr>> per_model;

//        // we subdivide each vertex depending on the pertinent model
//        for(int i = 0 ; i < vertices_.size(); ++i)
//        {
//            StratigraphicPositionableElement::Ptr vert = vertices_.at(i);

//            StratigraphicModelBase::Ptr vert_model = vert->getStratigraphicModel();
//            per_model[vert_model].push_back(vert);
//        }

//        size_t n_elements = per_model.size();
//        residuals.resize(n_elements - 1);

//        LOG(INFO) << "number of models for this constrain: " << n_elements;

//        size_t counter = 0;
//        float previous_avg = spcNANMacro;
//        for (std::pair<spc::StratigraphicModelBase::Ptr, std::vector<spc::StratigraphicPositionableElement::Ptr>>  group: per_model)
//        {
//            // for each vertex
//            float avg = 0;
//            for (StratigraphicPositionableElement::Ptr vert: group.second)
//            {
//                float mypos = vert->getStratigraphicPosition();
//                LOG(INFO) << "position of this vertex " << mypos;
//                avg += mypos;
//            }

//            avg /= group.second.size(); // avg stratigraphic position for this group of vertexes

//            LOG(INFO) << "avg position for this group " << avg;

//            if (counter == 0)
//            {
//                counter++;
//                previous_avg = avg;

//                LOG(INFO) << "setted previou_avg as " << previous_avg;
//            }
//            else
//            {
//                float diff = previous_avg - avg;

//                LOG(INFO) << "diff is " << diff << " with avg " << avg << " and previous_avg " << previous_avg ;
//                residuals(counter - 1) = diff*diff;
//                counter++;


//            }
//        }

        LOG(INFO) << "residuals for this constrain: " << residuals.transpose();




        return residuals;
    }

protected:
    std::vector<StratigraphicPositionableElement::Ptr> vertices_;

    // a representation os polylin of the points defining the constrain
    PolyLine3D polyline_rep_;

private:
    friend class cereal::access;




    template <class Archive> void load(Archive &ar, std::uint32_t const version)
    {
        ar(cereal::base_class<ElementBase>(this), CEREAL_NVP(vertices_));
        updatePointSetRepresentation();

    }

    template <class Archive> void save(Archive &ar, std::uint32_t const version) const
    {
        ar(cereal::base_class<ElementBase>(this), CEREAL_NVP(vertices_));
    }

};



}


CEREAL_CLASS_VERSION(spc::StratigraphicConstrain, 1)

#endif // STRATIGRAPHICLINK_H
