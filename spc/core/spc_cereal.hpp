#pragma once
#ifndef SPC_CEREAL_HPP
#define SPC_CEREAL_HPP

//! all he header includes needed for registering cereal stuff

#include <cereal/cereal.hpp>
//#include <cereal/archives/binary.hpp>
//#include <cereal/archives/xml.hpp>
//#include <cereal/archives/json.hpp>
//#include <cereal/archives/portable_binary.hpp>

#include <spc/core/cerea_requested_types.hpp>


#include <spc/io/cereal_types.hpp>
#include <cereal/types/polymorphic.hpp>
#include <spc/io/eigen_serialization.hpp>


#include <cereal/types/map.hpp>
#include <cereal/types/vector.hpp>


#endif // SPC_CEREAL_HPP
