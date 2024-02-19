#include <mars_interfaces/sim/JointInterface.h>
#include <envire_core/plugin/Plugin.hpp>


BOOST_SERIALIZATION_SPLIT_FREE(mars::interfaces::JointInterfaceItem)

namespace boost
{
    namespace serialization
		{

        template<class Archive> inline void save(Archive & ar, const mars::interfaces::JointInterfaceItem & value, const unsigned int file_version)
				{
            std::string buffer;
            buffer = value.pluginName;
            ar << buffer;
        }

        template<class Archive> inline void load(Archive & ar, mars::interfaces::JointInterfaceItem & value, const unsigned int file_version)
				{
            std::string buffer;
            ar >> buffer;
            value.pluginName = buffer;
        }

    }
}

ENVIRE_REGISTER_ITEM(mars::interfaces::JointInterfaceItem)
