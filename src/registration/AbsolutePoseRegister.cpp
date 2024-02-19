#include <mars_interfaces/sim/AbsolutePose.hpp>
#include <envire_core/plugin/Plugin.hpp>


BOOST_SERIALIZATION_SPLIT_FREE(mars::interfaces::AbsolutePose)

namespace boost
{
    namespace serialization
		{

        template<class Archive> inline void save(Archive & ar, const mars::interfaces::AbsolutePose & value, const unsigned int file_version)
				{
            throw std::runtime_error("mars::interfaces::AbsolutePose::save not implemented");
        }

        template<class Archive> inline void load(Archive & ar, mars::interfaces::AbsolutePose & value, const unsigned int file_version)
				{
            throw std::runtime_error("mars::interfaces::AbsolutePose::load not implemented");
        }

    }
}

ENVIRE_REGISTER_ITEM(mars::interfaces::AbsolutePose)
