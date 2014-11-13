#pragma once
#ifndef SAMPLES_DB_H
#define SAMPLES_DB_H

#include <spc/elements/Sample.h>

#include <spc/core/std_helpers.hpp>

#include <fstream>

#include <boost/tokenizer.hpp>

#include <boost/algorithm/string.hpp>
#include <iostream>

#include <iostream>

#include <cereal/types/vector.hpp>
#include <set>
namespace spc
{

class SamplesDB: public ElementBase
{
public:
    SPC_ELEMENT(SamplesDB)
    EXPOSE_TYPE

    //! empty constructor
    SamplesDB()
    {
    }

    Sample::Ptr at(const size_t id) const
    {
        return db_.at(id);
    }

    Sample::Ptr addSample()
    {
        Sample::Ptr sample (new Sample);
        pushBack(sample);
        return sample;
    }


    void pushBack(Sample::Ptr data)
    {
        db_.push_back(data);
    }

    size_t size() const
    {
        return db_.size();
    }

    std::vector<Sample::Ptr> getSamplesDB() const
    {
        return db_;
    }

    size_t getNumberOfDifferentClouds() const
    {
        return this->getVectorOfUniqueClouds().size();
    }

    std::vector<std::string> getUniqueKeys() const
    {
        std::set<std::string> tmp_set;
       for (Sample::Ptr s: db_)
       {
           std::vector<std::string> keys = s->getVariantPropertiesRecord().getKeysList();

           for (std::string k: keys)
           {
               tmp_set.insert(k);
           }
       }

       std::vector<std::string> out;
       out.assign(tmp_set.begin(), tmp_set.end());

       return out;
    }

    template <typename T>
    SamplesDB getNotNanEntries(const std::string fieldname) const
    {
        SamplesDB out_db;

        for (Sample::Ptr s: db_)
            if (std::isfinite(s->variantPropertyValue<T>(fieldname)))
                out_db.pushBack(s);

        return out_db;
    }


    std::vector<Sample::Ptr> getDataForSampleID(size_t core_point_id)
    {
        if (core_ids_indices_list_.empty())
            buildCoreIdFastAccessor();

        return core_ids_indices_list_.at(core_point_id);
    }

    template <typename T>
    std::vector<Sample::Ptr> getDataWithValue(const std::string key, T value) const
    {
        std::vector<Sample::Ptr> out;
        for(Sample::Ptr ptr: this->getSamplesDB())
        {
            T val = ptr->variantPropertyValue<T>(key);
            if (val == value)
                out.push_back(ptr);
        }

        return out;

    }

    void buildCoreIdFastAccessor()
    {
        core_ids_indices_list_.resize(getVectorOfUniqueSamples().size());

        for(Sample::Ptr core: db_)
        {
            int current_id = core->variantPropertyValue<int>("core_id");
            core_ids_indices_list_.at(current_id).push_back(core);
        }
    }

    std::vector<int> getVectorOfUniqueSamples()
    {
        // first get the core points ids as a vector list
        std::vector<int> core_points_ids;

        for(Sample::Ptr core: db_)
        {
            int current_id = core->variantPropertyValue<int>("core_id");
            if (!spc::element_exists<int>(core_points_ids, current_id))
                core_points_ids.push_back(current_id);
        }

        return core_points_ids;
    }

    std::vector<int> getVectorOfUniqueClouds() const
    {
        // first get the core points ids as a vector list
        std::vector<int> cloud_ids;

        for(Sample::ConstPtr core: db_)
        {
            int current_id = core->variantPropertyValue<int>("cloud_id");
            if (!spc::element_exists<int>(cloud_ids, current_id))
                cloud_ids.push_back(current_id);
        }

        return cloud_ids;
    }

    void clear()
    {
        db_.clear();
        core_ids_indices_list_.clear();
    }

    template <typename T>
    static std::vector<T> extractPropertyAsVector(std::vector<Sample::Ptr>samples, const std::string prop_name)
    {
        std::vector<T> out;
        for(Sample::Ptr entry: samples)
            out.push_back(entry->variantPropertyValue<T>(prop_name));

        return out;
    }

private:
    std::vector<Sample::Ptr> db_;

    std::vector<std::vector<Sample::Ptr>> core_ids_indices_list_;
private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::ElementBase>(this),
           CEREAL_NVP(db_), CEREAL_NVP(core_ids_indices_list_));
    }
};

} // end nspace

#endif
