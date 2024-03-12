#include <mars_interfaces/sim/DynamicObject.hpp>
#include <envire_core/plugin/Plugin.hpp>


BOOST_SERIALIZATION_SPLIT_FREE(mars::interfaces::DynamicObjectItem)

namespace boost
{
    namespace serialization
    {

        template<class Archive> inline void save(Archive & ar, const mars::interfaces::DynamicObjectItem & value, const unsigned int file_version)
        {
            std::string buffer;
            buffer = value.pluginName;
            ar << buffer;
        }

        template<class Archive> inline void load(Archive & ar, mars::interfaces::DynamicObjectItem & value, const unsigned int file_version)
        {
            std::string buffer;
            ar >> buffer;
            value.pluginName = buffer;
        }

    }
}

ENVIRE_REGISTER_ITEM(mars::interfaces::DynamicObjectItem)
