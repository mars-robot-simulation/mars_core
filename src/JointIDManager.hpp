#pragma once

#include <string>
#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_interfaces/sim/IDManager.hpp>
#include <mars_interfaces/sim/JointInterface.h>
#include <envire_core/events/GraphItemEventDispatcher.hpp>
#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>

namespace mars
{
    namespace core
    {
        class JointIDManager :  public interfaces::IDManager,
                                public envire::core::GraphItemEventDispatcher<envire::core::Item<interfaces::JointInterfaceItem>>
        {
        public:
            JointIDManager(std::shared_ptr<envire::core::EnvireGraph> envireGraph)
            {
                GraphItemEventDispatcher<envire::core::Item<interfaces::JointInterfaceItem>>::subscribe(envireGraph.get());
            }

            ~JointIDManager()
            {
                GraphItemEventDispatcher<envire::core::Item<interfaces::JointInterfaceItem>>::unsubscribe();
            }

            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<interfaces::JointInterfaceItem>>& e) override
            {
                std::string jointName;
                e.item->getData().jointInterface->getName(&jointName);
                add(jointName);
            }

            virtual void itemRemoved(const envire::core::TypedItemRemovedEvent<envire::core::Item<interfaces::JointInterfaceItem>>& e) override
            {
                std::string jointName;
                e.item->getData().jointInterface->getName(&jointName);
                removeEntry(jointName);
            }
        };
    }
}
